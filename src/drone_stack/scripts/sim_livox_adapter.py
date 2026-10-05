#!/usr/bin/env python3
"""Adapt instantaneous Gazebo ray scans to Livox CustomMsg for Faster-LIO.

All rays in one Gazebo scan describe the same instant. Marking them as a
rolling scan would create fictitious motion distortion during flight.
"""
import math

import rospy
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from sensor_msgs.msg import PointCloud, Imu
from std_srvs.srv import SetBool, SetBoolResponse


class Adapter:
    def __init__(self):
        self.publisher = rospy.Publisher('/livox/lidar', CustomMsg, queue_size=2)
        self.imu_publisher = rospy.Publisher('/livox/imu', Imu, queue_size=100)
        # Exclude spawn/contact impulses from Faster-LIO's one-shot gravity
        # and accelerometer scale initialisation. It needs stationary IMU data.
        self.startup_settle = rospy.get_param('~startup_settle_s', 2.0)
        self.drop_lidar = False
        self.drop_imu = False
        self.geometry_loss = False
        rospy.Service('/drone/sim/faults/lidar', SetBool, self.set_lidar_fault)
        rospy.Service('/drone/sim/faults/imu', SetBool, self.set_imu_fault)
        rospy.Service('/drone/sim/faults/lidar_geometry', SetBool, self.set_geometry_fault)
        rospy.Subscriber('/drone/sim/lidar/imu_raw', Imu, self.on_imu, queue_size=100)
        self.step = max(1, rospy.get_param('~sample_step', 1))
        self.scan_period_ns = int(rospy.get_param('~scan_period_s', 0.1) * 1e9)
        self.min_useful_range = rospy.get_param('~min_useful_range_m', 0.45)
        self.max_range = rospy.get_param('~max_range_m', 40.0)
        rospy.Subscriber('/drone/sim/lidar/points', PointCloud, self.on_cloud, queue_size=1)

    def on_cloud(self, cloud):
        if self.drop_lidar or cloud.header.stamp.to_sec() < self.startup_settle:
            return
        points = cloud.points[::self.step]
        if len(points) < 2:
            return
        out = CustomMsg()
        out.header = cloud.header
        out.header.stamp = cloud.header.stamp - rospy.Duration.from_sec(self.scan_period_ns * 1e-9)
        out.header.frame_id = 'lidar'
        out.timebase = out.header.stamp.to_nsec()
        out.lidar_id = 1
        out.rsvd = [0, 0, 0]
        for index, values in enumerate(points):
            if not all(math.isfinite(v) for v in (values.x, values.y, values.z)):
                continue
            range_sq = values.x * values.x + values.y * values.y + values.z * values.z
            if range_sq < self.min_useful_range ** 2 or range_sq >= (self.max_range - 0.05) ** 2:
                continue
            # Deliberately remove vertical constraints while preserving the
            # stream and original acquisition times. Nominal mount rotation
            # only; never use simulator truth to construct algorithm inputs.
            if self.geometry_loss and abs(-0.5 * values.x + math.sqrt(0.75) * values.z) >= 0.20:
                continue
            point = CustomPoint()
            # Every ray belongs to the scan end time, matching Gazebo's snapshot.
            point.offset_time = self.scan_period_ns
            point.x, point.y, point.z = values.x, values.y, values.z
            point.reflectivity = 100
            point.tag = 0x10
            point.line = index % 4
            out.points.append(point)
        out.point_num = len(out.points)
        if out.point_num >= 2:
            rospy.loginfo_throttle(10.0, 'Livox sim adapter: %d/%d valid rays', out.point_num, len(points))
            self.publisher.publish(out)

    def on_imu(self, msg):
        if not self.drop_imu and msg.header.stamp.to_sec() >= self.startup_settle:
            self.imu_publisher.publish(msg)

    def set_lidar_fault(self, request):
        self.drop_lidar = request.data
        return SetBoolResponse(success=True, message='Simulated lidar stream paused' if request.data else 'Lidar stream restored')

    def set_imu_fault(self, request):
        self.drop_imu = request.data
        return SetBoolResponse(success=True, message='Simulated lidar IMU paused' if request.data else 'Lidar IMU restored')

    def set_geometry_fault(self, request):
        self.geometry_loss = request.data
        return SetBoolResponse(success=True, message='Vertical lidar constraints removed' if request.data else 'Lidar geometry restored')


if __name__ == '__main__':
    rospy.init_node('sim_livox_adapter')
    Adapter()
    rospy.spin()
