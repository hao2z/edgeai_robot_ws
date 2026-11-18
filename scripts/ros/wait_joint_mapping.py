"""Read-only startup barrier: both scan chains and both navigation servers exist."""
import argparse
import json
from pathlib import Path
import time
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.time import Time


def main():
    p=argparse.ArgumentParser();p.add_argument('directory',type=Path);p.add_argument('--timeout',type=float,default=70.)
    a=p.parse_args();rclpy.init(args=['--ros-args','-p','use_sim_time:=true']);node=rclpy.create_node('joint_mapping_startup')
    data={};buffer=Buffer();listener=TransformListener(buffer,node)
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions=[node.create_subscription(String,topic,lambda m,k=key:data.update({k:json.loads(m.data)}),qos)
                   for key,topic in [('alignment','/joint_mapping/alignment'),('graph','/slam_toolbox/anchor_graph'),('slam','/joint_mapping/status'),('authority2','/robot2/navigation_authority/state')]]
    peers={}
    subscriptions.extend(node.create_subscription(PoseStamped,'/cooperative_rescue/peer_pose/'+robot,
        lambda msg,r=robot:peers.update({r:msg}),1) for robot in ('robot1','robot2'))
    client=node.create_client(GetState,'/robot2/bt_navigator/get_state');request=None;active=False;reason='No joint SLAM graph'
    end=time.monotonic()+a.timeout
    try:
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.05)
            alignment=data.get('alignment',{})
            if str(alignment.get('state','')).lower()=='failed':raise RuntimeError('Relative alignment failed: '+alignment.get('reason','unknown'))
            graph=data.get('graph',{});robots={n.get('robot_id') for n in graph.get('nodes',[])}
            cross_edges=data.get('slam',{}).get('cross_robot_edges',0)
            if client.service_is_ready() and request is None:request=client.call_async(GetState.Request())
            if request is not None and request.done():active=request.result().current_state.id==3;request=None
            peer_ready=all(r in peers and -.2<=(node.get_clock().now()-Time.from_msg(peers[r].header.stamp)).nanoseconds/1e9<=.8
                           for r in ('robot1','robot2'))
            reason=f'alignment={alignment.get("accepted",False)}, scan_chains={sorted(str(r) for r in robots)}, cross_robot_edges={cross_edges}, robot2_nav_active={active}, peer_obstacles_ready={peer_ready}'
            if alignment.get('accepted') and {'robot1','robot2'}<=robots and cross_edges>0 and active and peer_ready:
                for robot in ('robot1','robot2'):
                    try:
                        tf=buffer.lookup_transform('map',robot+'/base_footprint',Time())
                        age=(node.get_clock().now()-Time.from_msg(tf.header.stamp)).nanoseconds/1e9
                        if not -.2<=age<=.8:raise ValueError('stale '+robot+' TF')
                    except (TransformException,ValueError) as error:reason=str(error);break
                else:
                    (a.directory/'joint-mapping-ready.json').write_text(json.dumps(dict(success=True,reason=reason)))
                    print('JOINT_MAPPING_READY: '+reason,flush=True);return 0
        raise RuntimeError('Joint mapping startup timeout: '+reason)
    except RuntimeError as error:
        (a.directory/'startup-failure.json').write_text(json.dumps(dict(success=False,reason=str(error))))
        print(str(error),flush=True);return 1
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':raise SystemExit(main())
