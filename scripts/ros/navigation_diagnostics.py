"""Read-only RViz diagnostics using one explicit TF time, never latest fallback."""
import json
import math
import time
from collections import deque
from html import escape
from pathlib import Path

import rclpy
from rclpy.duration import Duration
from rclpy.clock import Clock, ClockType
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import Point, PolygonStamped, Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String, Bool
from rcl_interfaces.msg import Log
from action_msgs.msg import GoalStatusArray
from lifecycle_msgs.msg import State, TransitionEvent
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from navigation_status import runtime_settings, render
from mission_display import mission_description


def planar(transform):
    t, q = transform.transform.translation, transform.transform.rotation
    return t.x, t.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def apply_pose(pose, points):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x+c*px-s*py, y+s*px+c*py) for px, py in points]


def age_seconds(now_ns, stamp):
    stamp_ns = stamp.sec*10**9+stamp.nanosec
    return None if stamp_ns == 0 else (now_ns-stamp_ns)/1e9


class NavigationDiagnostics(Node):
    def __init__(self, robot='', peer=None):
        super().__init__('edgeai_navigation_diagnostics'+('_'+robot if robot else ''))
        self.robot, self.peer = robot, peer
        self.prefix = '/'+robot if robot else ''
        self.frame_prefix = robot+'/' if robot else ''
        self.rendered = None
        self.declare_parameter('comparison_delay', 0.10)
        self.declare_parameter('show_global_outline', False)
        self.declare_parameter('static_map',False)
        self.declare_parameter('log_directory', '')
        self.declare_parameter('cooperative', False)
        directory = self.get_parameter('log_directory').value
        self.runtime_directory = Path(directory)/robot if directory and robot else None
        self.settings = runtime_settings(self.runtime_directory if robot else directory)
        if self.settings:
            self.runtime_directory = None
        self.cooperative = self.get_parameter('cooperative').value
        self.static_map_active = False
        self.mission_display = ''
        self.buffer = Buffer(cache_time=Duration(seconds=15))
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(MarkerArray, self.prefix+'/diagnostics/navigation', 5)
        self.status_publisher = (None if robot else
                                 self.create_publisher(String, '/diagnostics/navigation_status', 1))
        self.authority = None
        self.authority_received = None
        self.authority_subscription = self.create_subscription(String, self.prefix+'/navigation_authority/state',
            self.authority_state, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.messages = {}
        self.wall_received = {}
        self.json_messages = {}
        self.last_warning = None
        self.clock_advanced = time.monotonic()
        self.stationary_since = None
        self.stationary_task = None
        self.arrivals = {}
        self.previous_tick = None
        self.rtf = None
        self.subscriptions_ = []
        self.subscriptions_.append(self.create_subscription(
            OccupancyGrid, '/map', lambda message: self.record('map', message),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
        if self.cooperative:
            self.subscriptions_.append(self.create_subscription(
                TransitionEvent, '/map_server/transition_event', self.map_transition, 10))
        topics = {'global_costmap': ('/global_costmap/costmap', OccupancyGrid),
                  'local_costmap': ('/local_costmap/costmap', OccupancyGrid),
                  'global_footprint': ('/global_costmap/published_footprint', PolygonStamped),
                  'local_footprint': ('/local_costmap/published_footprint', PolygonStamped),
                  'scan': ('/scan', LaserScan), 'odom': ('/odom', Odometry),
                  'global_plan': ('/plan', NavPath), 'trajectory': ('/local_plan', NavPath),
                  'collision_arc': ('/lookahead_arc', NavPath),
                  'depth_cloud': ('/camera/obstacle_points', PointCloud2),
                  'cmd_vel_nav': ('/cmd_vel_nav', Twist), 'cmd_vel_smoothed': ('/cmd_vel_smoothed', Twist),
                  'cmd_vel': ('/cmd_vel', Twist), 'rotating': ('/is_rotating_to_heading', Bool),
                  'goal': ('/navigation_authority/dispatched_goal', PoseStamped)}
        for name, (topic, kind) in topics.items():
            qos = (QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
                   if name == 'goal' else qos_profile_sensor_data)
            self.subscriptions_.append(self.create_subscription(
                kind, self.prefix+topic,
                lambda message, key=name: self.record(key, message), qos))
        json_topics = [('tracking','/navigation/rpp_tracking'), ('depth_filter','/camera/depth_obstacle_status')]
        if not robot:
            json_topics.append(('targets','/targets/estimates'))
        for key, topic in json_topics:
            qos = (qos_profile_sensor_data if key in ('tracking','depth_filter') else
                   QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.subscriptions_.append(self.create_subscription(String, self.prefix+topic,
                lambda message,key=key:self.json_record(key,message), qos))
        if not robot:
            self.subscriptions_.append(self.create_subscription(
                String, '/mission/state', self.mission_state,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
        self.subscriptions_.append(self.create_subscription(Log,'/rosout',self.warning_log,qos_profile_sensor_data))
        self.subscriptions_.append(self.create_subscription(GoalStatusArray,
            self.prefix+'/navigation_backend/navigate_to_pose/_action/status',self.navigation_status,
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)))
        self.initial_map_odom = None
        self.last_status = None
        self.last_log = 0.0
        self.audit = ((Path(directory)/('alignment-'+robot+'.jsonl' if robot else 'alignment.jsonl')).open('a', encoding='utf-8', buffering=1)
                      if directory else None)
        self.stopping_file = Path(directory)/'stopping' if directory else None
        # Continue to report paused simulation clocks and dead sensor publishers.
        # Diagnostics are observability only; 2 Hz is sufficient for the panel
        # and avoids competing with Nav2, Gazebo and perception callbacks.
        self.create_timer(.5, self.update, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def map_transition(self, message):
        if message.goal_state.id in (State.PRIMARY_STATE_ACTIVE, State.PRIMARY_STATE_INACTIVE,
                                     State.PRIMARY_STATE_UNCONFIGURED, State.PRIMARY_STATE_FINALIZED):
            self.static_map_active = message.goal_state.id == State.PRIMARY_STATE_ACTIVE

    def mission_state(self, message):
        try:
            data = json.loads(message.data)
        except ValueError:
            return
        if isinstance(data, dict):
            self.mission_display = mission_description(data) or self.mission_display

    def json_record(self, key, message):
        try:
            value = json.loads(message.data)
            if isinstance(value,dict):
                self.json_messages[key] = value
                self.record(key,message)
        except (ValueError,TypeError):
            pass

    def warning_log(self, message):
        if message.name.startswith('robot2.') != bool(self.robot):
            return
        if message.level >= Log.WARN and any(name in message.name for name in
                ('controller_server','planner_server','bt_navigator','behavior_server','local_costmap')):
            self.last_warning = dict(node=message.name, text=message.msg[:400], received=time.monotonic())

    def navigation_status(self, message):
        if not message.status_list:
            return
        current_id = (self.authority or {}).get('goal_uuid')
        matches = [item for item in message.status_list if bytes(item.goal_info.goal_id.uuid).hex()==current_id]
        latest = max(matches or message.status_list,
                     key=lambda item:(item.goal_info.stamp.sec,item.goal_info.stamp.nanosec))
        self.json_messages['navigation_action'] = dict(status=latest.status,
            goal_id=bytes(latest.goal_info.goal_id.uuid).hex())
        self.wall_received['navigation_action'] = time.monotonic()

    def authority_state(self, message):
        try:
            self.authority = json.loads(message.data)
            self.authority_received = time.monotonic()
        except (ValueError, TypeError):
            pass

    def record(self, key, message):
        self.messages[key] = message
        self.wall_received[key] = time.monotonic()
        stamp = self.get_clock().now().nanoseconds/1e9
        arrivals = self.arrivals.setdefault(key, deque(maxlen=300))
        if arrivals and stamp < arrivals[-1]:
            arrivals.clear()
        arrivals.append(stamp)
        while arrivals and stamp-arrivals[0] > 2:
            arrivals.popleft()

    def rate(self, key, now):
        values = self.arrivals.get(key, [])
        if len(values) < 2 or now-values[-1] > .6 or values[-1] == values[0]:
            return 0.0
        return (len(values)-1)/(values[-1]-values[0])

    def lookup(self, target, source, stamp):
        return planar(self.buffer.lookup_transform(target, source, stamp))

    def marker(self, ident, kind, stamp, color):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = stamp.to_msg()
        marker.ns = 'navigation_diagnostics'
        marker.id, marker.type = ident, kind
        marker.pose.orientation.w = 1.0
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (*color, 1.0)
        marker.lifetime = Duration(seconds=1).to_msg()
        return marker

    def outline(self, key, stamp, base_pose, ident, color, dashed=False):
        message = self.messages.get(key)
        if message is None:
            raise ValueError(f'waiting for {key}')
        if message.header.stamp.sec == 0 and message.header.stamp.nanosec == 0:
            raise ValueError(f'{key}: zero timestamp')
        # Recover each producer's actual configured polygon at its own time,
        # then project BOTH polygons through the same map<-base TF time.
        base_from_message = self.lookup(self.frame_prefix+'base_footprint', message.header.frame_id,
                                        Time.from_msg(message.header.stamp))
        spec = apply_pose(base_from_message, [(p.x, p.y) for p in message.polygon.points])
        points = apply_pose(base_pose, spec)
        if len(points) < 3:
            raise ValueError(f'{key}: empty polygon')
        marker = self.marker(ident, Marker.LINE_LIST, stamp, color)
        marker.scale.x = .022 if dashed else .009
        for a, b in zip(points, points[1:]+points[:1]):
            count = max(2, math.ceil(math.dist(a,b)/.035))
            for i in range(count):
                if dashed and i % 2:
                    continue
                for u in (i/count, (i+1)/count):
                    marker.points.append(Point(x=a[0]+u*(b[0]-a[0]), y=a[1]+u*(b[1]-a[1]), z=.08))
        return marker

    def update(self):
        if self.runtime_directory is not None:
            settings = runtime_settings(self.runtime_directory)
            if settings:
                self.settings = settings
                self.runtime_directory = None
        now = self.get_clock().now()
        if now.nanoseconds <= 0:
            return
        wall = time.monotonic()
        if self.previous_tick is not None:
            previous_sim, previous_wall = self.previous_tick
            delta = now.nanoseconds/1e9-previous_sim
            if delta != 0:
                self.clock_advanced = wall
            factor = max(0.,delta)/max(.001, wall-previous_wall)
            self.rtf = factor if self.rtf is None else .9*self.rtf+.1*factor
        self.previous_tick = (now.nanoseconds/1e9, wall)
        stamp = Time(nanoseconds=max(1, now.nanoseconds-int(self.get_parameter('comparison_delay').value*1e9)),
                     clock_type=now.clock_type)
        a = self.authority or {}
        controller_id = {'heading':'FollowPathHeading', 'rescue':'FollowPathRescue'}.get(
            a.get('arrival_policy'), 'FollowPath')
        follow = self.settings.get('controllers', {}).get(controller_id, {})
        primary = follow.get('primary_controller', follow.get('plugin', '')).lower()
        local_key = 'collision_arc' if 'rpp' in primary or 'pure_pursuit' in primary else 'trajectory'
        ages = {key: age_seconds(now.nanoseconds, message.header.stamp)
                for key, message in self.messages.items() if hasattr(message,'header')}
        rates = {key:self.rate(key,now.nanoseconds/1e9) for key in self.messages}
        wall_ages = {key:wall-value for key,value in self.wall_received.items()}
        for values in (ages, rates, wall_ages):
            if local_key in values:
                values['local_plan'] = values[local_key]
        stale = [key for key in ('map','global_costmap','local_costmap','scan','odom',
                                'global_footprint','local_footprint')
                 if key not in ages or ages[key] is None or ages[key] > (2.5 if key == 'map' else .6)
                 or ages[key] < -.2]
        static_map = self.static_map_active or self.get_parameter('static_map').value
        if static_map and 'map' in self.messages:
            stale=[key for key in stale if key!='map']
        markers, problems = [], []
        correction = None
        try:
            base_frame, odom_frame = (self.frame_prefix+name for name in ('base_footprint','odom'))
            base = self.lookup('map', base_frame, stamp)
            odom_base = self.lookup(odom_frame, base_frame, stamp)
            map_odom = self.lookup('map', odom_frame, stamp)
            if self.initial_map_odom is None:
                self.initial_map_odom = map_odom
            anchor_xy = apply_pose(self.initial_map_odom, [odom_base[:2]])[0]
            correction = math.dist(anchor_xy, base[:2])
            for key, ident, color, dashed in (
                    ('global_footprint', 1, (.30,.56,1.) if self.robot else (0.,1.,1.), True),
                    ('local_footprint', 2, (.45,.82,1.) if self.robot else (1.,.85,0.), False)):
                if key == 'global_footprint' and not self.get_parameter('show_global_outline').value:
                    continue
                try:
                    markers.append(self.outline(key, stamp, base, ident, color, dashed))
                except (ValueError, TransformException) as error:
                    problems.append(str(error))
            arrow = self.marker(3, Marker.ARROW, stamp, (1.0,.2,1.0))
            arrow.scale.x, arrow.scale.y, arrow.scale.z = .025, .06, .08
            if correction > .003:
                arrow.points = [Point(x=anchor_xy[0],y=anchor_xy[1],z=.12), Point(x=base[0],y=base[1],z=.12)]
                markers.append(arrow)
        except TransformException:
            problems.append('TF at comparison time unavailable; no latest fallback')

        stopping = self.stopping_file is not None and self.stopping_file.exists()
        standby = bool(self.robot and self.authority_received is None and not self.settings)
        status = ('STOPPING' if stopping else 'STANDBY' if standby else
                  'WAIT/STALE' if stale or problems else 'TIME CHECK OK')
        odometry = self.messages.get('odom')
        task = (a.get('generation'),a.get('phase'))
        authority_age = wall-self.authority_received if self.authority_received is not None else math.inf
        stationary = (odometry is not None and authority_age <= 2. and a.get('phase')=='ACTIVE'
                      and a.get('following_path') and ages.get('odom') is not None and 0 <= ages['odom'] <= .6
                      and wall-self.wall_received.get('odom',0.) <= 2.
                      and abs(odometry.twist.twist.linear.x)<.01 and abs(odometry.twist.twist.angular.z)<.05)
        if task!=self.stationary_task or not stationary:
            self.stationary_since = None
        self.stationary_task = task
        if stationary and self.stationary_since is None:
            self.stationary_since = now.nanoseconds/1e9
        stationary_s = max(0.,now.nanoseconds/1e9-self.stationary_since) if self.stationary_since is not None else 0.
        velocities = {key:(message.linear.x,message.angular.z) for key,message in self.messages.items()
                      if key in ('cmd_vel_nav','cmd_vel_smoothed','cmd_vel')}
        if odometry is not None:
            velocities['odom'] = (odometry.twist.twist.linear.x,odometry.twist.twist.angular.z)
        goal = self.messages.get('goal')
        if goal is not None:
            p,q = goal.pose.position,goal.pose.orientation
            goal = (goal.header.frame_id,p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        snapshot = dict(authority=self.authority,authority_age=authority_age, settings=self.settings,
            ages=ages,wall_ages=wall_ages,rates=rates,standby=standby,static_map=static_map,
            camera_available=not (self.cooperative and self.robot),
            velocities=velocities,goal=goal,tracking=self.json_messages.get('tracking'),
            tracking_age=wall-self.wall_received.get('tracking',0.),
            rotating=self.messages['rotating'].data if 'rotating' in self.messages else None,
            rotating_age=wall-self.wall_received.get('rotating',0.),
            targets=self.json_messages.get('targets'),
            navigation_action=self.json_messages.get('navigation_action'),
            depth_filter=self.json_messages.get('depth_filter'),depth_filter_age=wall-self.wall_received.get('depth_filter',0.),
            last_warning=self.last_warning, wall_time=wall,
            rtf=self.rtf,correction_m=correction,problems=problems,stopping=stopping,
            stationary_s=stationary_s,clock_paused=wall-self.clock_advanced>2.)
        rendered = self.rendered = render(snapshot)
        if self.status_publisher is not None:
            body, details, legend = rendered['body'], rendered['details'], rendered['legend']
            if self.peer is not None:
                peer = self.peer.rendered
                body = '<h3>机器人 1</h3>'+rendered['summary']+'<hr/><h3>机器人 2</h3>'+(
                    peer['summary'] if peer else '<p>等待机器人 2 诊断数据</p>')
                details = '<h2>机器人 1</h2>'+rendered['body']+details+'<hr/><h2>机器人 2</h2>'+(
                    peer['body']+peer['details'] if peer else '')
                legend += ('<p>机器人 2：橙色全局路径、紫色局部轨迹和前视点、浅蓝色目标与局部轮廓、'
                           '蓝色雷达。两车显示项分别开关；当前仿真仅机器人 1 有 RGB-D。</p>')
            if self.mission_display:
                body = '<p><b>任务调度：</b>'+escape(self.mission_display)+'</p>'+body
            self.status_publisher.publish(String(data=body+'<!--DETAILS-->'+details+'<!--LEGEND-->'+legend))
        self.publisher.publish(MarkerArray(markers=markers))
        state = (status, tuple(stale), bool(problems))
        if state != self.last_status:
            self.get_logger().info(f'{status}; stale={stale}; TF/input={problems}')
            self.last_status = state
        if self.audit and time.monotonic()-self.last_log >= 1:
            self.audit.write(json.dumps(dict(sim_time=now.nanoseconds/1e9, comparison_time=stamp.nanoseconds/1e9,
                ages=ages, rates=rates, map_source='static' if static_map else 'slam',
                real_time_factor=self.rtf, status=status, stale=stale, tf_errors=problems, correction_m=correction,
                operational_status=rendered['heading'], warnings=rendered['warnings'], authority=self.authority,
                goal=goal, velocities=velocities, wall_ages=snapshot['wall_ages'], tracking=snapshot['tracking'],
                navigation_action=snapshot['navigation_action']))+'\n')
            self.last_log = time.monotonic()

    def destroy_node(self):
        if self.audit:
            self.audit.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = NavigationDiagnostics()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    if node.get_parameter('cooperative').value:
        node.peer = NavigationDiagnostics('robot2')
        executor.add_node(node.peer)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        executor.shutdown()
        if node.peer is not None:
            node.peer.destroy_node()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
