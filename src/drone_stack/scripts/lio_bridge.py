#!/usr/bin/env python3
"""Convert the LIO sensor pose to an ENU/FLU vehicle pose for MAVROS.

MAVROS owns the ENU/FLU to NED/FRD conversion. This node never performs it.
"""
import copy
import math
from collections import deque

import numpy as np
import rospy
import tf
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool
from tf.transformations import (concatenate_matrices, inverse_matrix,
                                quaternion_from_matrix, quaternion_matrix,
                                translation_matrix)
import tf2_ros


class LioBridge:
    def __init__(self):
        self.last_stamp = rospy.Time(0)
        # A 10 Hz scan plus mapping can be ~0.25 s old when published.
        self.max_age = rospy.get_param('~max_age_s', 0.6)
        self.future_tolerance = rospy.get_param('~future_tolerance_s', 0.02)
        self.max_pose_step = rospy.get_param('~max_pose_step_m', 0.5)
        self.max_pose_speed = rospy.get_param('~max_pose_speed_mps', 3.0)
        self.max_rotation_step = rospy.get_param('~max_rotation_step_rad', 0.3)
        self.max_rotation_speed = rospy.get_param('~max_rotation_speed_radps', 4.0)
        self.pose_fault = False
        self.last_clock = rospy.Time(0)
        offset = rospy.get_param('~sensor_xyz_body', [0.27, 0.0, 0.10])
        pitch = math.radians(rospy.get_param('~sensor_pitch_deg', 30.0))
        # Body and sensor use ROS FLU. Positive Y rotation points sensor X down.
        q = tf.transformations.quaternion_from_euler(0.0, pitch, 0.0)
        self.base_sensor = concatenate_matrices(translation_matrix(offset), quaternion_matrix(q))
        self.sensor_base = inverse_matrix(self.base_sensor)
        # Faster-LIO starts camera_init aligned with the tilted sensor. Rotate
        # that whole world into horizontal ROS ENU before publishing to MAVROS.
        self.world_align = quaternion_matrix(q)
        self.pose_cov = rospy.get_param('~pose_covariance_diagonal',
                                        [0.04, 0.04, 0.09, 0.09, 0.09, 0.04])
        self.odom_pub = rospy.Publisher('/drone/lio/odom', Odometry, queue_size=10)
        self.mavros_pub = rospy.Publisher('/mavros/odometry/out', Odometry, queue_size=10)
        self.cloud_pub = rospy.Publisher('/drone/cloud_world', PointCloud2, queue_size=2)
        self.fcu_cloud_pub = rospy.Publisher('/drone/cloud_fcu_world', PointCloud2, queue_size=2)
        self.fcu_odom_pub = rospy.Publisher('/drone/fcu/odom', Odometry, queue_size=10)
        self.valid_pub = rospy.Publisher('/drone/lio/valid', Bool, queue_size=1, latch=True)
        self.tf_pub = tf2_ros.TransformBroadcaster()
        rospy.Subscriber('/Odometry', Odometry, self.on_odom, queue_size=10)
        self.lio_poses = deque(maxlen=30)
        self.fcu_poses = deque(maxlen=100)
        rospy.Subscriber('/mavros/local_position/odom', Odometry, self.on_fcu_odom, queue_size=30)
        rospy.Subscriber('/cloud_registered', PointCloud2, self.on_cloud, queue_size=2)
        rospy.Subscriber('/clock', Clock, self.on_clock, queue_size=1, tcp_nodelay=True)
        rospy.Timer(rospy.Duration(0.1), self.on_health, reset=True)

    def on_clock(self, msg):
        if (self.last_clock-msg.clock).to_nsec()/1e9 > self.future_tolerance:
            self.pose_fault = True
            self.valid_pub.publish(Bool(data=False))
            rospy.logerr('ROS clock moved backwards; bridge restart required after landing')
        self.last_clock = msg.clock

    def on_health(self, _event):
        age = (rospy.Time.now() - self.last_stamp).to_nsec()/1e9 if self.last_stamp != rospy.Time(0) else float('inf')
        self.valid_pub.publish(Bool(data=not self.pose_fault and -self.future_tolerance <= age <= self.max_age))

    def on_odom(self, msg):
        if self.pose_fault:
            return
        stamp = msg.header.stamp
        if stamp != rospy.Time(0) and (self.last_stamp-stamp).to_nsec()/1e9 > self.future_tolerance:
            self.pose_fault = True
            self.valid_pub.publish(Bool(data=False))
            rospy.logerr('LIO acquisition time moved backwards; bridge restart required after landing')
            return
        if stamp <= self.last_stamp or stamp == rospy.Time(0):
            return
        if msg.header.frame_id != 'camera_init':
            rospy.logerr_throttle(2.0, 'Expected Faster-LIO parent camera_init, got %s', msg.header.frame_id)
            return
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        if not all(math.isfinite(v) for v in values) or abs(sum(v * v for v in values[3:]) - 1.0) > 0.05:
            rospy.logerr_throttle(2.0, 'Rejected invalid Faster-LIO pose')
            return
        if not -self.future_tolerance <= (rospy.Time.now() - stamp).to_nsec()/1e9 <= self.max_age:
            rospy.logwarn_throttle(2.0, 'Rejected stale Faster-LIO pose')
            return
        sensor_pose = concatenate_matrices(translation_matrix([p.x, p.y, p.z]),
                                           quaternion_matrix([q.x, q.y, q.z, q.w]))
        base_pose = concatenate_matrices(self.world_align, sensor_pose, self.sensor_base)
        if self.lio_poses:
            previous_stamp, previous_pose = self.lio_poses[-1]
            dt = (stamp-previous_stamp).to_sec()
            distance = np.linalg.norm(base_pose[:3,3]-previous_pose[:3,3])
            rotation = np.dot(previous_pose[:3,:3].T, base_pose[:3,:3])
            angle = math.acos(float(np.clip((np.trace(rotation)-1)/2, -1, 1)))
            if (distance > self.max_pose_step+self.max_pose_speed*dt or
                    angle > self.max_rotation_step+self.max_rotation_speed*dt):
                self.pose_fault = True
                self.valid_pub.publish(Bool(data=False))
                rospy.logerr('Rejected LIO pose jump; bridge restart required after landing')
                return
        quat = quaternion_from_matrix(base_pose)
        out = Odometry()
        out.header.stamp = stamp
        out.header.frame_id = 'odom'
        out.child_frame_id = 'base_link'
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = base_pose[:3, 3]
        out.pose.pose.orientation.x, out.pose.pose.orientation.y = quat[0], quat[1]
        out.pose.pose.orientation.z, out.pose.pose.orientation.w = quat[2], quat[3]
        for i, variance in enumerate(self.pose_cov):
            out.pose.covariance[i * 7] = variance
            # Unknown LIO velocity must never be advertised as precise.
            out.twist.covariance[i * 7] = 1e6
        self.last_stamp = stamp
        self.lio_poses.append((stamp, base_pose))
        self.odom_pub.publish(out)
        self.mavros_pub.publish(out)
        transform = TransformStamped()
        transform.header = out.header
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = out.pose.pose.position.x
        transform.transform.translation.y = out.pose.pose.position.y
        transform.transform.translation.z = out.pose.pose.position.z
        transform.transform.rotation = out.pose.pose.orientation
        self.tf_pub.sendTransform(transform)

    @staticmethod
    def pose_matrix(pose):
        p, q = pose.position, pose.orientation
        return concatenate_matrices(translation_matrix([p.x, p.y, p.z]),
                                    quaternion_matrix([q.x, q.y, q.z, q.w]))

    def on_fcu_odom(self, msg):
        if msg.header.frame_id not in ('map', 'odom'):
            return
        self.fcu_poses.append((msg.header.stamp, self.pose_matrix(msg.pose.pose)))
        # Display the FCU pose in the same numeric frame as cloud_fcu_world.
        # Keep MAVROS's original map-labelled topic intact for the controller.
        display_odom = copy.deepcopy(msg)
        display_odom.header.frame_id = 'odom'
        display_odom.child_frame_id = 'fcu_base_link'
        self.fcu_odom_pub.publish(display_odom)

    @staticmethod
    def transform_cloud(msg, matrix, frame_id):
        """Apply a rigid transform to XYZ without a Python loop per point.

        tf2_sensor_msgs.do_transform_cloud iterates over every PointCloud2
        record in Python.  This bridge transforms two clouds for every lidar
        scan, so that implementation can consume multiple CPU cores worth of
        wall time and delay planning.  A strided NumPy view preserves every
        original PointCloud2 field while changing only x/y/z.
        """
        fields = {field.name: field for field in msg.fields}
        if not all(name in fields for name in ('x', 'y', 'z')):
            raise ValueError('PointCloud2 has no x/y/z fields')
        endian = '>' if msg.is_bigendian else '<'
        formats = []
        offsets = []
        for name in ('x', 'y', 'z'):
            field = fields[name]
            if field.count != 1 or field.datatype not in (PointField.FLOAT32, PointField.FLOAT64):
                raise ValueError('PointCloud2 %s is not a scalar float field' % name)
            formats.append(endian + ('f4' if field.datatype == PointField.FLOAT32 else 'f8'))
            offsets.append(field.offset)
        dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': formats,
                          'offsets': offsets, 'itemsize': msg.point_step})
        raw = bytearray(msg.data)
        shape = (msg.height, msg.width)
        strides = (msg.row_step, msg.point_step)
        source = np.ndarray(shape=shape, dtype=dtype, buffer=msg.data, strides=strides)
        target = np.ndarray(shape=shape, dtype=dtype, buffer=raw, strides=strides)
        xyz = np.stack((source['x'].reshape(-1), source['y'].reshape(-1),
                        source['z'].reshape(-1)), axis=0)
        transformed = np.dot(matrix[:3, :3], xyz) + matrix[:3, 3:4]
        target['x'][...] = transformed[0].reshape(shape)
        target['y'][...] = transformed[1].reshape(shape)
        target['z'][...] = transformed[2].reshape(shape)

        out = PointCloud2()
        out.header = copy.copy(msg.header)
        out.header.frame_id = frame_id
        out.height = msg.height
        out.width = msg.width
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.row_step
        out.data = bytes(raw)
        out.is_dense = msg.is_dense
        return out

    @staticmethod
    def nearest_pose(poses, stamp, tolerance=0.15):
        if not poses:
            return None
        sample = min(tuple(poses), key=lambda entry: abs((entry[0] - stamp).to_nsec()))
        return sample[1] if abs((sample[0] - stamp).to_nsec()) <= round(tolerance*1e9) else None

    def on_cloud(self, msg):
        # Rotate the points with the same camera_init -> odom transform used
        # for vehicle poses, so planner obstacles stay in the same ENU frame.
        if msg.header.frame_id != 'camera_init':
            rospy.logerr_throttle(2.0, 'Rejected cloud in unexpected frame %s', msg.header.frame_id)
            return
        if self.last_stamp == rospy.Time(0):
            return
        try:
            out = self.transform_cloud(msg, self.world_align, 'odom')
        except ValueError as exc:
            rospy.logerr_throttle(2.0, 'Rejected cloud: %s', exc)
            return
        self.cloud_pub.publish(out)
        # EGO uses PX4's fused position and IMU-derived velocity. Its obstacle
        # cloud must share those numeric ENU coordinates; PX4's local origin can
        # differ from Faster-LIO's origin even though both have ROS frames.
        lio_pose = self.nearest_pose(self.lio_poses, msg.header.stamp)
        fcu_pose = self.nearest_pose(self.fcu_poses, msg.header.stamp)
        if lio_pose is None or fcu_pose is None:
            return
        fcu_from_lio = concatenate_matrices(fcu_pose, inverse_matrix(lio_pose))
        self.fcu_cloud_pub.publish(self.transform_cloud(out, fcu_from_lio, 'odom'))


if __name__ == '__main__':
    clock_transport = rospy.Subscriber('/clock', Clock, queue_size=1, tcp_nodelay=True)
    rospy.init_node('drone_lio_bridge')
    LioBridge()
    rospy.spin()
