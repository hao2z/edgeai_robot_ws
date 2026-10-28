"""Read-only RViz diagnostics using one explicit TF time, never latest fallback."""
import json
import html
import math
import time
from collections import deque
from pathlib import Path

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import Point, PolygonStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


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
    def __init__(self):
        super().__init__('edgeai_navigation_diagnostics')
        self.declare_parameter('comparison_delay', 0.10)
        self.declare_parameter('log_directory', '')
        self.buffer = Buffer(cache_time=Duration(seconds=15))
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(MarkerArray, '/diagnostics/navigation', 5)
        self.status_publisher = self.create_publisher(String, '/diagnostics/navigation_status', 1)
        self.messages = {}
        self.arrivals = {}
        self.previous_tick = None
        self.rtf = None
        self.subscriptions_ = []
        topics = {'map': ('/map', OccupancyGrid),
                  'global_costmap': ('/global_costmap/costmap', OccupancyGrid),
                  'local_costmap': ('/local_costmap/costmap', OccupancyGrid),
                  'global_footprint': ('/global_costmap/published_footprint', PolygonStamped),
                  'local_footprint': ('/local_costmap/published_footprint', PolygonStamped),
                  'scan': ('/scan', LaserScan), 'odom': ('/odom', Odometry)}
        for name, (topic, kind) in topics.items():
            qos = (QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
                   if name == 'map' else qos_profile_sensor_data)
            self.subscriptions_.append(self.create_subscription(
                kind, topic, lambda message, key=name: self.record(key, message), qos))
        self.initial_map_odom = None
        self.last_status = None
        self.last_log = 0.0
        directory = self.get_parameter('log_directory').value
        self.audit = ((Path(directory)/'alignment.jsonl').open('a', encoding='utf-8', buffering=1)
                      if directory else None)
        self.stopping_file = Path(directory)/'stopping' if directory else None
        self.create_timer(.1, self.update)

    def record(self, key, message):
        self.messages[key] = message
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
        base_from_message = self.lookup('base_footprint', message.header.frame_id,
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
        now = self.get_clock().now()
        if now.nanoseconds <= 0:
            return
        wall = time.monotonic()
        if self.previous_tick is not None:
            previous_sim, previous_wall = self.previous_tick
            factor = (now.nanoseconds/1e9-previous_sim)/max(.001, wall-previous_wall)
            self.rtf = factor if self.rtf is None else .9*self.rtf+.1*factor
        self.previous_tick = (now.nanoseconds/1e9, wall)
        stamp = Time(nanoseconds=max(1, now.nanoseconds-int(self.get_parameter('comparison_delay').value*1e9)),
                     clock_type=now.clock_type)
        ages = {key: age_seconds(now.nanoseconds, message.header.stamp)
                for key, message in self.messages.items()}
        stale = [key for key in ('map','global_costmap','local_costmap','scan','odom',
                                'global_footprint','local_footprint')
                 if key not in ages or ages[key] is None or ages[key] > (2.5 if key == 'map' else .6)
                 or ages[key] < -.2]
        markers, problems = [], []
        correction = None
        try:
            base = self.lookup('map', 'base_footprint', stamp)
            odom_base = self.lookup('odom', 'base_footprint', stamp)
            map_odom = self.lookup('map', 'odom', stamp)
            if self.initial_map_odom is None:
                self.initial_map_odom = map_odom
            anchor_xy = apply_pose(self.initial_map_odom, [odom_base[:2]])[0]
            correction = math.dist(anchor_xy, base[:2])
            for key, ident, color, dashed in (
                    ('global_footprint', 1, (0.0,1.0,1.0), True),
                    ('local_footprint', 2, (1.0,.85,0.0), False)):
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

        def age(key):
            value = ages.get(key)
            return '?' if value is None else f'{value:.2f}'

        stopping = self.stopping_file is not None and self.stopping_file.exists()
        status = 'STOPPING' if stopping else ('WAIT/STALE' if stale or problems else 'TIME CHECK OK')
        correction_text = '?' if correction is None else f'{correction*100:.1f} cm'
        label = '测试结束，导航已停止' if stopping else ('等待/过期数据' if stale or problems else '时间检查通过')
        rows = ''.join(f'<tr><td>{name}</td><td>{age(key)} s</td><td>{self.rate(key, now.nanoseconds/1e9):.1f}</td></tr>'
                       for key, name in (('map','SLAM地图'),('global_costmap','全局代价地图'),
                                         ('local_costmap','局部代价地图'),('scan','雷达'),('odom','里程计'),
                                         ('global_footprint','全局轮廓'),('local_footprint','局部轮廓')))
        factor = '?' if self.rtf is None else f'{self.rtf:.2f}x'
        legend = ''.join(f'<tr><td style="color:{color}"><b>{symbol} {name}</b></td></tr><tr><td>{meaning}</td></tr>'
            for color, symbol, name, meaning in (
                ('#008f9a','--','全局轮廓','青色虚线：全局规划使用的车身包络'),
                ('#9a7800','—','局部轮廓','黄色实线：局部控制使用的包络；同一TF时刻投影'),
                ('#198535','━','全局路径','绿色线：导航路线，不是车身范围'),
                ('#198535','■','前沿候选','绿色方块：探索器的候选观察点，不是障碍物'),
                ('#b3316a','━','局部轨迹','粉色线：控制器选中的短时轨迹'),
                ('#bb3030','·','雷达回波','红色点：当前扫描命中点；灰色车模显示实体外观'),
                ('#b020b0','→','SLAM修正','品红箭头：相对初始里程计锚点的修正，不是真值'),
                ('#535d70','■','代价地图','红/紫渐变含障碍及膨胀；可在左侧分别开关两张地图')))
        body = (f'<h3>{label}</h3><p><b>轮廓颜色不代表无碰撞。</b><br>青黄重合表示同一时刻的几何一致。</p>'
                f'<table width="100%" cellspacing="3">{legend}</table><hr>'
                f'<p>仿真倍率：{factor}<br>对比延迟：{self.get_parameter("comparison_delay").value:.2f} s'
                f'<br>SLAM相对锚点修正：{correction_text}</p>'
                f'<table width="100%" cellspacing="3"><tr><th align="left">数据</th><th>年龄</th><th>实收Hz</th></tr>{rows}</table>'
                '<p>频率按模拟时间统计。时间检查通过不等于通行安全。</p>')
        if stale or problems:
            body += '<p>'+html.escape(', '.join(stale)+('; '+ '; '.join(problems) if problems else ''))+'</p>'
        self.status_publisher.publish(String(data=body))
        self.publisher.publish(MarkerArray(markers=markers))
        state = (status, tuple(stale), bool(problems))
        if state != self.last_status:
            self.get_logger().info(f'{status}; stale={stale}; TF/input={problems}')
            self.last_status = state
        if self.audit and time.monotonic()-self.last_log >= 1:
            self.audit.write(json.dumps(dict(sim_time=now.nanoseconds/1e9, comparison_time=stamp.nanoseconds/1e9,
                ages=ages, rates={key:self.rate(key,now.nanoseconds/1e9) for key in self.messages},
                real_time_factor=self.rtf, status=status, stale=stale, tf_errors=problems, correction_m=correction))+'\n')
            self.last_log = time.monotonic()

    def destroy_node(self):
        if self.audit:
            self.audit.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = NavigationDiagnostics()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
