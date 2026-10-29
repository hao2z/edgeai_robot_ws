"""Bounded, passive observation of the upstream exploration demo."""
import collections
import json
import math
from pathlib import Path
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry, OccupancyGrid
from action_msgs.msg import GoalStatusArray

rclpy.init()
node = Node('upstream_demo_observer')
output = Path(sys.argv[1])
state = {'distance_m': 0., 'odom_messages': 0, 'known_cells': 0, 'free_cells': 0,
         'max_known_cells': 0, 'statuses': {}}
last = None
seen = {}
def odom(msg):
    global last
    p = msg.pose.pose.position
    if last is not None:
        state['distance_m'] += math.hypot(p.x-last[0], p.y-last[1])
    last = (p.x,p.y)
    state['pose_xy'] = last
    state['odom_messages'] += 1

def mapping(msg):
    state['known_cells'] = sum(v >= 0 for v in msg.data)
    state['free_cells'] = sum(v == 0 for v in msg.data)
    state['max_known_cells'] = max(state['max_known_cells'],state['known_cells'])
    state['map_resolution'] = msg.info.resolution

def status(kind,msg):
    for item in msg.status_list:
        seen[(kind,bytes(item.goal_info.goal_id.uuid).hex())] = item.status
    state['statuses'] = dict(collections.Counter(f'{k[0]}:{v}' for k,v in seen.items()))

node.create_subscription(Odometry,'/odom',odom,qos_profile_sensor_data)
node.create_subscription(OccupancyGrid,'/map',mapping,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
for kind in ['navigate_to_pose','spin','backup','wait','follow_path']:
    node.create_subscription(GoalStatusArray,f'/{kind}/_action/status',lambda msg,k=kind:status(k,msg),QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
start = time.monotonic()
next_report = 0
while rclpy.ok() and time.monotonic()-start < 210:
    rclpy.spin_once(node,timeout_sec=.2)
    elapsed=time.monotonic()-start
    if elapsed >= next_report:
        state['wall_elapsed_s']=elapsed
        (output/'observation.json').write_text(json.dumps(state,indent=2))
        print(json.dumps(state),flush=True)
        next_report=elapsed+5
    text=(output/'explore.log').read_text(errors='replace')
    if ('Published exploration completion event' in text or 'No more frontiers found' in text) and elapsed>10:
        state['reason']='explorer_completion_log'
        break
else:
    state['reason']='observation_time_limit'
(output/'observation.json').write_text(json.dumps(state,indent=2))
node.destroy_node()
rclpy.shutdown()
