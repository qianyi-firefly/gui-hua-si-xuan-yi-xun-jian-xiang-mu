#!/usr/bin/env python3
"""Single owner of PX4 OFFBOARD setpoints and operator flight commands."""
import math
import threading
from itertools import combinations

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import EstimatorStatus, ExtendedState, PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from rosgraph_msgs.msg import Clock
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import Bool, String, Header
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse

from drone_stack.srv import (LocalGoal, LocalGoalResponse, Takeoff,
                             TakeoffResponse)


class FlightManager:
    def __init__(self):
        self.lock = threading.RLock()
        self.authorized = False
        self.phase = 'DISCONNECTED'
        self.last_error = ''
        self.state = State()
        self.extended = ExtendedState()
        self.odom = None
        self.estimator = None
        self.pending_estimator = None
        self.lio_valid = False
        self.lio_health_time = rospy.Time(0)
        self.trajectory = None
        self.latest_command_id = 0
        self.required_trajectory_id = 1
        self.trajectory_time = rospy.Time(0)
        self.future_command_since = None
        self.goal_start = rospy.Time(0)
        self.pending_goal = None
        self.active_goal = None
        self.goal_reached_since = None
        self.goal_tolerance = rospy.get_param('~goal_tolerance_m', 0.15)
        self.goal_speed_tolerance = rospy.get_param('~goal_speed_tolerance_mps', 0.15)
        self.goal_min = rospy.get_param('~goal_min_xyz', [-8.0, -8.0, 0.5])
        self.goal_max = rospy.get_param('~goal_max_xyz', [8.0, 8.0, 2.5])
        self.max_takeoff_height = rospy.get_param('~max_takeoff_height_m', 2.0)
        self.map_resolution = rospy.get_param('/ego_planner_node/grid_map/resolution', 0.1)
        self.map_origin = [-rospy.get_param('/ego_planner_node/grid_map/map_size_x',30.0)/2,
                           -rospy.get_param('/ego_planner_node/grid_map/map_size_y',30.0)/2,
                           rospy.get_param('/ego_planner_node/grid_map/ground_height',0.5)]
        self.occupied_cells = set()
        self.map_time = rospy.Time(0)
        self.max_map_age = rospy.get_param('~max_map_age_s', 2.0)
        self.max_setpoint_step = rospy.get_param('~max_setpoint_step_m', 0.75)
        self.max_command_speed = rospy.get_param('~max_command_speed_mps', 0.75)
        self.prime_start = rospy.Time(0)
        self.arm_start = rospy.Time(0)
        self.hold_pose = None
        self.takeoff_pose = None
        self.takeoff_start = rospy.Time(0)
        self.takeoff_start_z = 0.0
        self.takeoff_speed = rospy.get_param('~takeoff_speed_mps', 0.3)
        self.land_start = rospy.Time(0)
        self.max_pose_age = rospy.get_param('~max_pose_age_s', 0.35)
        self.max_traj_age = rospy.get_param('~max_traj_age_s', 0.35)
        # SITL only: PX4 requires 500 timesync exchanges before it honours
        # acquisition timestamps. Real hardware uses normal synchronisation.
        self.startup_min_time = rospy.get_param('~startup_min_time_s', 0.0)
        self.future_tolerance = rospy.get_param('~future_tolerance_s', 0.02)
        self.clock_fault = False
        self.last_clock = rospy.Time(0)
        self.descent_rate = rospy.get_param('~fallback_descent_rate_mps', 0.35)
        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=1,
                                           tcp_nodelay=True)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=5)
        self.enabled_pub = rospy.Publisher('/drone/planning_enabled', Bool, queue_size=1, latch=True)
        self.phase_pub = rospy.Publisher('/drone/flight_state', String, queue_size=1, latch=True)
        self.auth_pub = rospy.Publisher('/drone/authorized', Bool, queue_size=1, latch=True)
        self.error_pub = rospy.Publisher('/drone/flight_error', String, queue_size=1, latch=True)
        self.heartbeat_pub = rospy.Publisher('/drone/manager_heartbeat', Header, queue_size=1)
        rospy.Subscriber('/mavros/state', State, self.on_state, queue_size=10)
        rospy.Subscriber('/mavros/extended_state', ExtendedState, self.on_extended, queue_size=10)
        rospy.Subscriber('/mavros/local_position/odom', Odometry, self.on_odom, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/mavros/estimator_status', EstimatorStatus, self.on_estimator, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/drone/lio/valid', Bool, self.on_lio, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/planning/pos_cmd', PositionCommand, self.on_trajectory, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/grid_map/occupancy_inflate_safety', PointCloud2, self.on_map, queue_size=1,
                         buff_size=4*1024*1024, tcp_nodelay=True)
        rospy.Service('/drone/set_authorized', SetBool, self.set_authorized)
        rospy.Service('/drone/arm', Trigger, self.arm)
        rospy.Service('/drone/disarm', Trigger, self.disarm)
        rospy.Service('/drone/takeoff', Takeoff, self.takeoff)
        rospy.Service('/drone/hold', Trigger, self.hold)
        rospy.Service('/drone/cancel_goal', Trigger, self.hold)
        rospy.Service('/drone/land', Trigger, self.land)
        rospy.Service('/drone/local_goal', LocalGoal, self.local_goal)
        self.arm_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.enabled_pub.publish(Bool(data=False))
        self.auth_pub.publish(Bool(data=False))
        rospy.Subscriber('/clock', Clock, self.on_clock, queue_size=1, tcp_nodelay=True)
        rospy.Timer(rospy.Duration(1.0 / 30.0), self.tick, reset=True)

    def on_clock(self, msg):
        with self.lock:
            if (self.last_clock-msg.clock).to_nsec()/1e9 > self.future_tolerance:
                self.clock_fault = True
            self.last_clock = msg.clock

    def set_phase(self, phase):
        if self.phase != phase:
            rospy.loginfo('Flight state: %s -> %s', self.phase, phase)
            self.phase = phase
            self.phase_pub.publish(String(data=phase))

    def error(self, message):
        self.last_error = message
        self.error_pub.publish(String(data=message))
        rospy.logwarn(message)

    def fresh_pose(self):
        now = rospy.Time.now()
        # ESTIMATOR_STATUS is a low-rate stream. Dropping one slightly early
        # sample outright can create a false two-second outage. A pending
        # sample cannot refresh health until its acquisition time has arrived.
        pending=self.pending_estimator
        if pending is not None:
            age=(now-pending.header.stamp).to_nsec()/1e9
            if age>=0:
                if age<=2.0 and (self.estimator is None or pending.header.stamp>self.estimator.header.stamp):
                    self.estimator=pending
                self.pending_estimator=None
        estimator = self.estimator
        # Duration.to_sec() forms -1 + fractional seconds for negative ages,
        # rounding an exact -20 ms to -0.020000000000000018. Convert the signed
        # integer nanoseconds once, so the inclusive boundary stays inclusive.
        estimator_age = (now-estimator.header.stamp).to_nsec()/1e9 if estimator is not None else float('inf')
        odom_age = (now-self.odom.header.stamp).to_nsec()/1e9 if self.odom is not None else float('inf')
        health_age = (now-self.lio_health_time).to_nsec()/1e9
        estimator_ok = (estimator is not None and
                        -self.future_tolerance <= estimator_age <= 2.0 and
                        (estimator.pos_horiz_rel_status_flag or estimator.pos_horiz_abs_status_flag) and
                        (estimator.pos_vert_abs_status_flag or estimator.pos_vert_agl_status_flag) and
                        estimator.velocity_horiz_status_flag and estimator.velocity_vert_status_flag)
        self.pose_health_detail = ('odom_age=%.6f estimator_age=%.6f health_age=%.6f '
                                   'lio_valid=%s estimator_ok=%s connected=%s now=%.9f' %
                                   (odom_age, estimator_age, health_age, self.lio_valid,
                                    estimator_ok, self.state.connected, now.to_sec()))
        return (not self.clock_fault and now.to_sec() >= self.startup_min_time and
                self.odom is not None and self.lio_valid and self.state.connected and estimator_ok and
                0 <= health_age <= 0.5 and
                -self.future_tolerance <= odom_age <= self.max_pose_age)

    def airborne(self):
        return (self.state.armed and
                (self.extended.landed_state == ExtendedState.LANDED_STATE_IN_AIR or
                 self.phase in ('TAKEOFF', 'HOLD', 'NAVIGATING', 'LANDING', 'DESCENDING')))

    def current_pose(self):
        return self.odom.pose.pose if self.odom else None

    def on_state(self, msg):
        with self.lock:
            self.state = msg
            if not msg.connected:
                self.set_phase('DISCONNECTED')
            elif not msg.armed and self.phase not in (
                    'DISCONNECTED', 'LOCALIZING', 'READY', 'PRIMING_ARM', 'WAIT_OFFBOARD', 'WAIT_ARM'):
                self.enabled_pub.publish(Bool(data=False))
                self.pending_goal = None
                self.set_phase('READY')

    def on_extended(self, msg):
        with self.lock:
            self.extended = msg

    def on_odom(self, msg):
        with self.lock:
            if msg.header.frame_id not in ('odom', 'map'):
                self.error('PX4 local odometry is not in expected ROS ENU frame')
                return
            if (rospy.Time.now()-msg.header.stamp).to_nsec()/1e9 < -self.future_tolerance:
                rospy.logwarn_throttle(2.0, 'Dropping future FCU odometry; keeping last valid sample')
                return
            self.odom = msg

    def on_estimator(self, msg):
        with self.lock:
            age=(rospy.Time.now()-msg.header.stamp).to_nsec()/1e9
            if age < -self.future_tolerance:
                if age>=-.1 and (self.pending_estimator is None or msg.header.stamp>self.pending_estimator.header.stamp):
                    self.pending_estimator=msg
                return
            if self.estimator is None or msg.header.stamp>self.estimator.header.stamp:
                self.estimator = msg

    def on_lio(self, msg):
        with self.lock:
            self.lio_valid = msg.data
            self.lio_health_time = rospy.Time.now()

    def map_cell(self, values):
        return tuple(math.floor((value-origin)/self.map_resolution) for value,origin in zip(values,self.map_origin))

    def fresh_map(self):
        return (self.map_time != rospy.Time(0) and
                -self.future_tolerance <= (rospy.Time.now()-self.map_time).to_nsec()/1e9 <= self.max_map_age)

    def in_flight_volume(self, values):
        return (all(math.isfinite(v) for v in values) and
                all(lo <= v <= hi for v, lo, hi in zip(values, self.goal_min, self.goal_max)))

    def segment_cells(self, start, end):
        """Traverse every closed voxel touched by a line, including edge ties.

        Uniform sampling can miss a short intersection near a voxel corner.
        DDA checks crossed faces and all neighbours at simultaneous crossings.
        """
        def grid(values):
            result = [(v-o)/self.map_resolution for v, o in zip(values, self.map_origin)]
            return [float(round(v)) if abs(v-round(v)) < 1e-10 else v for v in result]
        a, b = grid(start), grid(end)
        cell = [math.floor(v) for v in a]
        delta = [v-u for u, v in zip(a, b)]
        step = [1 if v > 0 else -1 if v < 0 else 0 for v in delta]
        stationary_faces = [i for i in range(3) if step[i] == 0 and abs(a[i]-round(a[i])) < 1e-10]
        start_faces = [i for i in range(3) if abs(a[i]-round(a[i])) < 1e-10]

        def neighbours(base, faces):
            for size in range(len(faces)+1):
                for axes in combinations(faces, size):
                    value = list(base)
                    for axis in axes:
                        value[axis] -= 1
                    yield tuple(value)

        yield from neighbours(cell, start_faces)
        crossing = [((cell[i]+1-a[i])/delta[i] if step[i] > 0 else
                     (cell[i]-a[i])/delta[i] if step[i] < 0 else math.inf)
                    for i in range(3)]
        interval = [abs(1/v) if v else math.inf for v in delta]
        while True:
            t = min(crossing)
            if t > 1.0+1e-10:
                break
            axes = [i for i in range(3) if abs(crossing[i]-t) < 1e-10]
            for size in range(1, len(axes)+1):
                for selected in combinations(axes, size):
                    other = list(cell)
                    for axis in selected:
                        other[axis] += step[axis]
                    yield from neighbours(other, stationary_faces)
            for axis in axes:
                cell[axis] += step[axis]
                crossing[axis] += interval[axis]

    def command_error(self, msg):
        age = (rospy.Time.now()-msg.header.stamp).to_nsec()/1e9
        if (msg.header.frame_id != 'odom' or msg.header.stamp == rospy.Time(0) or
                not -self.future_tolerance <= age <= self.max_traj_age):
            return ('Invalid or stale trajectory acquisition time/frame: '
                    'age=%.6f s stamp=%.9f frame=%s' %
                    (age, msg.header.stamp.to_sec(), msg.header.frame_id))
        values = [msg.position.x, msg.position.y, msg.position.z]
        if not self.in_flight_volume(values) or not math.isfinite(msg.yaw):
            return 'Trajectory setpoint is outside configured flight volume or nonfinite'
        velocity = [msg.velocity.x, msg.velocity.y, msg.velocity.z]
        if (not all(math.isfinite(v) for v in velocity) or
                math.sqrt(sum(v*v for v in velocity)) > self.max_command_speed):
            return 'Trajectory velocity is nonfinite or exceeds configured speed limit'
        if not self.fresh_map():
            return 'Obstacle map became stale; navigation cancelled'
        p = self.current_pose().position
        start = [p.x, p.y, p.z]
        if not self.in_flight_volume(start):
            return 'Vehicle is outside configured navigation volume'
        if math.sqrt(sum((v-u)**2 for u, v in zip(start, values))) > self.max_setpoint_step:
            return 'Trajectory setpoint jump exceeds tracking limit'
        if any(cell in self.occupied_cells for cell in self.segment_cells(start, values)):
            return 'Trajectory segment intersects an inflated obstacle voxel'
        return ''

    def stop_navigation(self, reason):
        response = self.hold(None)
        if response.success:
            if reason:
                self.error(reason)
            self.setpoint_pub.publish(self.setpoint(self.hold_pose))
        else:
            self.start_landing(reason + '; safe hold unavailable')

    def on_map(self, msg):
        if msg.header.frame_id != 'odom':
            return
        with self.lock:
            if msg.header.stamp != rospy.Time(0) and msg.header.stamp == self.map_time:
                return
        cells={self.map_cell(p) for p in point_cloud2.read_points(msg,field_names=('x','y','z'),skip_nans=True)}
        with self.lock:
            self.occupied_cells=cells
            self.map_time=msg.header.stamp

    def on_trajectory(self, msg):
        with self.lock:
            if (msg.trajectory_flag != PositionCommand.TRAJECTORY_STATUS_READY or
                    msg.header.frame_id != 'odom' or msg.trajectory_id < self.required_trajectory_id):
                return
            if self.phase != 'NAVIGATING':
                # Track dry-run generations, but ignore expired command streams.
                if msg.header.stamp != rospy.Time(0) and -self.future_tolerance <= (rospy.Time.now()-msg.header.stamp).to_nsec()/1e9 <= self.max_traj_age:
                    self.latest_command_id = max(self.latest_command_id, msg.trajectory_id)
                return
            age = (rospy.Time.now()-msg.header.stamp).to_nsec()/1e9
            if -self.max_traj_age <= age < -self.future_tolerance:
                # Separate topics have independent TCPROS delivery. Never use
                # an ahead-of-clock command, but a single delayed clock packet
                # must not cancel an otherwise fresh, safe command stream.
                if self.future_command_since is None:
                    self.future_command_since = rospy.Time.now()
                rospy.logwarn_throttle(2.0, 'Dropping future EGO command (age %.6f s)', age)
                return
            reason = self.command_error(msg)
            if reason:
                self.stop_navigation(reason)
                return
            self.future_command_since = None
            self.latest_command_id = max(self.latest_command_id, msg.trajectory_id)
            self.trajectory = msg
            self.trajectory_time = msg.header.stamp

    def set_authorized(self, request):
        with self.lock:
            if not request.data and self.airborne():
                self.start_landing('Operator revoked authorization')
            elif not request.data and self.phase in ('PRIMING_ARM', 'WAIT_OFFBOARD', 'WAIT_ARM'):
                self.set_phase('READY')
            elif not request.data and self.state.armed:
                self.disarm(None)
            self.authorized = request.data
            self.auth_pub.publish(Bool(data=self.authorized))
            return SetBoolResponse(success=True, message='Authorized' if self.authorized else 'Authorization revoked')

    def arm(self, _request):
        with self.lock:
            if (not self.authorized or not self.fresh_pose() or self.state.armed or
                    self.airborne() or self.phase != 'READY'):
                return TriggerResponse(success=False, message='Need authorization, fresh LIO/PX4 pose, and landed state')
            self.hold_pose = self.copy_pose()
            self.prime_start = rospy.Time.now()
            self.set_phase('PRIMING_ARM')
            return TriggerResponse(success=True, message='Priming OFFBOARD setpoints, then arming')

    def takeoff(self, request):
        with self.lock:
            if not self.authorized or not self.state.armed or not self.fresh_pose():
                return TakeoffResponse(success=False, message='Need authorization, ARM and fresh position')
            if (self.phase != 'ARMED' or self.state.mode != 'OFFBOARD' or
                    not math.isfinite(request.height_m) or not 0 < request.height_m <= self.max_takeoff_height):
                return TakeoffResponse(success=False, message='Invalid state or takeoff height')
            self.hold_pose = self.copy_pose()
            self.takeoff_pose = self.copy_pose()
            self.takeoff_pose.position.z += request.height_m
            self.takeoff_start_z = self.hold_pose.position.z
            self.takeoff_start = rospy.Time.now()
            self.set_phase('TAKEOFF')
            return TakeoffResponse(success=True, message='Takeoff setpoint streaming')

    def disarm(self, _request):
        with self.lock:
            if not self.state.armed or self.extended.landed_state != ExtendedState.LANDED_STATE_ON_GROUND:
                return TriggerResponse(success=False, message='Disarm requires confirmed ground contact and armed state')
            try:
                reply = self.arm_client(False)
                return TriggerResponse(success=reply.success, message='Ground disarm requested' if reply.success else 'PX4 rejected disarm')
            except rospy.ServiceException as exc:
                return TriggerResponse(success=False, message=str(exc))

    def hold(self, _request):
        with self.lock:
            if not self.fresh_pose() or not self.airborne():
                return TriggerResponse(success=False, message='Hold requires airborne vehicle and valid position')
            self.enabled_pub.publish(Bool(data=False))
            self.trajectory = None
            self.future_command_since = None
            self.required_trajectory_id = self.latest_command_id + 1
            self.pending_goal = None
            self.active_goal = None
            self.goal_reached_since = None
            self.hold_pose = self.copy_pose()
            self.set_phase('HOLD')
            return TriggerResponse(success=True, message='Current position held; previous trajectory ignored')

    def local_goal(self, request):
        with self.lock:
            goal = request.goal
            p = goal.pose.position
            values = [p.x, p.y, p.z]
            if (not self.authorized or not self.fresh_pose() or not self.airborne() or
                    self.phase not in ('HOLD', 'NAVIGATING') or goal.header.frame_id != 'odom' or
                    not all(math.isfinite(v) for v in values)):
                return LocalGoalResponse(success=False, message='Need authorized airborne HOLD, valid odom, and finite ENU goal')
            if not self.in_flight_volume(values):
                return LocalGoalResponse(success=False, message='Goal is outside configured flight volume')
            if not self.fresh_map():
                return LocalGoalResponse(success=False, message='Need fresh obstacle map before navigation')
            if any(cell in self.occupied_cells for cell in self.segment_cells(values, values)):
                return LocalGoalResponse(success=False, message='Goal is inside an inflated obstacle voxel')
            goal.header.stamp = rospy.Time.now()
            self.trajectory = None
            self.future_command_since = None
            self.required_trajectory_id = self.latest_command_id + 1
            self.goal_start = goal.header.stamp
            self.pending_goal = goal
            self.active_goal = goal
            self.goal_reached_since = None
            self.enabled_pub.publish(Bool(data=True))
            self.set_phase('NAVIGATING')
            return LocalGoalResponse(success=True, message='Local goal queued for EGO-Planner')

    def land(self, _request):
        with self.lock:
            if not self.state.armed:
                return TriggerResponse(success=False, message='Vehicle is not armed')
            self.start_landing('Operator requested landing')
            return TriggerResponse(success=True, message=self.phase)

    def start_landing(self, reason):
        self.enabled_pub.publish(Bool(data=False))
        self.trajectory = None
        self.pending_goal = None
        self.error(reason)
        self.active_goal = None
        self.goal_reached_since = None
        try:
            reply = self.mode_client(custom_mode='AUTO.LAND')
            if reply.mode_sent:
                self.set_phase('LANDING')
                return
        except rospy.ServiceException as exc:
            self.error('AUTO.LAND service failed: ' + str(exc))
        if self.fresh_pose():
            self.hold_pose = self.copy_pose()
            self.land_start = rospy.Time.now()
            self.set_phase('DESCENDING')
        else:
            self.set_phase('FAILSAFE')
            self.error('AUTO.LAND rejected and local position is unavailable; PX4 failsafe/manual takeover required')

    def copy_pose(self):
        import copy
        return copy.deepcopy(self.current_pose())

    @staticmethod
    def setpoint(pose):
        target = PositionTarget()
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = 'odom'
        # MAVLink enum names NED; numeric ROS fields remain ENU. MAVROS converts.
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        target.type_mask = (PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY |
                            PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_AFX |
                            PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                            PositionTarget.IGNORE_YAW_RATE)
        target.position = pose.position
        q = pose.orientation
        target.yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        return target

    @staticmethod
    def arming_setpoint(pose):
        # A ground position target can request takeoff after an EKF altitude
        # reset. Zero ENU velocity keeps OFFBOARD primed without requesting
        # ascent; only the explicit takeoff command enables position control.
        target = FlightManager.setpoint(pose)
        target.type_mask = (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY |
                            PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_AFX |
                            PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                            PositionTarget.IGNORE_YAW_RATE)
        return target

    def tick(self, event):
        with self.lock:
            now = rospy.Time.now()
            self.heartbeat_pub.publish(Header(stamp=now, frame_id='odom'))
            if not self.state.connected:
                return
            if not self.state.armed and self.phase in ('DISCONNECTED', 'LOCALIZING', 'READY'):
                self.set_phase('READY' if self.fresh_pose() else 'LOCALIZING')
            if self.airborne() and self.phase not in ('LANDING', 'DESCENDING', 'FAILSAFE') and not self.fresh_pose():
                self.start_landing('LIO or PX4 local position became stale: '+self.pose_health_detail)
            if self.phase == 'PRIMING_ARM':
                if not self.authorized or not self.fresh_pose():
                    self.set_phase('READY')
                    self.error('Arming cancelled because localization is unavailable')
                    return
                self.setpoint_pub.publish(self.arming_setpoint(self.hold_pose))
                if (now - self.prime_start).to_sec() >= 1.0:
                    try:
                        reply = self.mode_client(custom_mode='OFFBOARD')
                        if reply.mode_sent:
                            self.set_phase('WAIT_OFFBOARD')
                        else:
                            self.error('PX4 rejected OFFBOARD')
                            self.set_phase('READY')
                    except rospy.ServiceException as exc:
                        self.error('OFFBOARD service failed: ' + str(exc))
                        self.set_phase('READY')
                return
            if self.phase == 'WAIT_OFFBOARD':
                self.setpoint_pub.publish(self.arming_setpoint(self.hold_pose))
                if not self.authorized or not self.fresh_pose():
                    self.set_phase('READY')
                    self.error('Arming cancelled because localization is unavailable')
                elif self.state.mode == 'OFFBOARD':
                    try:
                        response = self.arm_client(True)
                        if response.success:
                            # The service ACK can precede /mavros/state by a
                            # heartbeat period. Confirm actual ARM before the
                            # UI enables takeoff or reports the ARMED phase.
                            self.arm_start = now
                            self.set_phase('WAIT_ARM')
                        else:
                            self.error('PX4 arming denied, result code %s' % response.result)
                            self.set_phase('READY')
                    except rospy.ServiceException as exc:
                        self.error('Arming service failed: ' + str(exc))
                        self.set_phase('READY')
                elif (now - self.prime_start).to_sec() > 4.0:
                    self.error('PX4 did not enter OFFBOARD within 4 seconds')
                    self.set_phase('READY')
                return
            if self.phase == 'WAIT_ARM':
                self.setpoint_pub.publish(self.arming_setpoint(self.hold_pose))
                if self.state.armed:
                    self.set_phase('ARMED')
                elif (now - self.arm_start).to_sec() > 4.0:
                    self.set_phase('READY')
                    self.error('PX4 ARM acknowledgement was not confirmed by state')
                return
            if self.phase == 'ARMED':
                if self.state.mode != 'OFFBOARD' or not self.fresh_pose():
                    self.error('OFFBOARD or localization lost before takeoff')
                    self.set_phase('READY')
                else:
                    self.setpoint_pub.publish(self.arming_setpoint(self.hold_pose))
                return
            if self.phase == 'TAKEOFF':
                if self.state.mode != 'OFFBOARD':
                    self.start_landing('PX4 left OFFBOARD during takeoff')
                    return
                import copy
                commanded = copy.deepcopy(self.takeoff_pose)
                commanded.position.z = min(self.takeoff_pose.position.z,
                    self.takeoff_start_z + self.takeoff_speed * (now - self.takeoff_start).to_sec())
                self.setpoint_pub.publish(self.setpoint(commanded))
                if (commanded.position.z == self.takeoff_pose.position.z and
                    abs(self.current_pose().position.z - self.takeoff_pose.position.z) < 0.10 and
                    abs(self.odom.twist.twist.linear.z) < 0.15):
                    self.hold_pose = copy.deepcopy(self.takeoff_pose)
                    self.set_phase('HOLD')
                return
            if self.phase == 'NAVIGATING':
                if self.state.mode != 'OFFBOARD':
                    self.start_landing('PX4 left OFFBOARD during navigation')
                    return
                if (self.future_command_since is not None and
                        (now-self.future_command_since).to_nsec()/1e9 >= self.max_traj_age):
                    self.stop_navigation('EGO command clock mismatch persisted; navigation cancelled')
                    return
                if not self.fresh_map():
                    self.stop_navigation('Obstacle map became stale; navigation cancelled')
                    return
                if self.active_goal:
                    p = self.current_pose().position
                    g = self.active_goal.pose.position
                    v = self.odom.twist.twist.linear
                    reached = (math.sqrt((p.x-g.x)**2+(p.y-g.y)**2+(p.z-g.z)**2) <= self.goal_tolerance and
                               math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z) <= self.goal_speed_tolerance)
                    if reached:
                        if self.goal_reached_since is None:
                            self.goal_reached_since = now
                        elif (now-self.goal_reached_since).to_sec() >= 1.0:
                            self.stop_navigation('')
                            return
                    else:
                        self.goal_reached_since = None
                if self.pending_goal and (now - self.goal_start).to_sec() >= 0.25:
                    self.goal_pub.publish(self.pending_goal)
                    self.pending_goal = None
                    self.goal_start = now
                if self.trajectory and (now - self.trajectory_time).to_sec() <= self.max_traj_age:
                    reason = self.command_error(self.trajectory)
                    if reason:
                        self.stop_navigation(reason)
                        return
                    pose = self.copy_pose()
                    pose.position = self.trajectory.position
                    quaternion = tf.transformations.quaternion_from_euler(0, 0, self.trajectory.yaw)
                    pose.orientation.x, pose.orientation.y = quaternion[0], quaternion[1]
                    pose.orientation.z, pose.orientation.w = quaternion[2], quaternion[3]
                    target = self.setpoint(pose)
                    # EGO's desired velocity is a controller feedforward, not
                    # an external velocity observation for the EKF. Position-
                    # only tracking lags ~v/Kp and can cut an obstacle corner.
                    target.type_mask &= ~(PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY |
                                          PositionTarget.IGNORE_VZ)
                    target.velocity.x = self.trajectory.velocity.x
                    target.velocity.y = self.trajectory.velocity.y
                    target.velocity.z = self.trajectory.velocity.z
                    self.setpoint_pub.publish(target)
                elif self.trajectory is not None:
                    self.stop_navigation('EGO-Planner trajectory stream became stale')
                elif (now - self.goal_start).to_sec() > 5.0:
                    self.stop_navigation('EGO-Planner trajectory timed out')
                else:
                    self.setpoint_pub.publish(self.setpoint(self.hold_pose))
                return
            if self.phase == 'HOLD':
                if self.state.mode != 'OFFBOARD':
                    self.start_landing('PX4 left OFFBOARD during hold')
                    return
                self.setpoint_pub.publish(self.setpoint(self.hold_pose))
            if self.phase == 'DESCENDING':
                if not self.fresh_pose():
                    self.set_phase('FAILSAFE')
                    self.error('Position lost during fallback descent')
                    return
                if self.extended.landed_state == ExtendedState.LANDED_STATE_ON_GROUND:
                    self.set_phase('READY')
                    return
                descent_dt=max(0.0,min(.1,(event.current_real-event.last_real).to_nsec()/1e9))
                self.hold_pose.position.z -= self.descent_rate * descent_dt
                self.setpoint_pub.publish(self.setpoint(self.hold_pose))
            if self.phase == 'LANDING' and self.extended.landed_state == ExtendedState.LANDED_STATE_ON_GROUND:
                self.set_phase('READY')


if __name__ == '__main__':
    # Register the hint before init_node establishes its automatic /clock
    # connection. Changing a hint afterwards does not renegotiate an already
    # connected TCPROS socket. This callback-free subscriber does not set time;
    # rospy's built-in simulation clock callback retains sole ownership.
    clock_transport = rospy.Subscriber('/clock', Clock, queue_size=1, tcp_nodelay=True)
    rospy.init_node('drone_flight_manager')
    FlightManager()
    rospy.spin()
