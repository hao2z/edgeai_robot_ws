"""Focused Nav2 check: a reachable goal 0.60 m behind the initial axle pose."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformException, TransformListener


def wrap(angle):
    return math.atan2(math.sin(angle),math.cos(angle))


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    args,ros_args=parser.parse_known_args()
    rclpy.init(args=ros_args)
    node=rclpy.create_node('edgeai_navigation_turn_check',enable_rosout=False)
    buffer=Buffer()
    listener=TransformListener(buffer,node)
    client=ActionClient(node,NavigateToPose,'/navigate_to_pose')
    stop=node.create_publisher(Twist,'/cmd_vel',1)
    command=[0.,0.]
    subscription=node.create_subscription(Twist,'/cmd_vel',
        lambda msg:command.__setitem__(slice(None),(msg.linear.x,msg.angular.z)),qos_profile_sensor_data)
    trace=[]
    handle=None
    result=dict(success=False,mode='isolated_turn_check',trace=trace)

    def pose():
        t=buffer.lookup_transform('map','base_footprint',rclpy.time.Time()).transform
        return t.translation.x,t.translation.y,math.atan2(2*(t.rotation.w*t.rotation.z+t.rotation.x*t.rotation.y),
                                                      1-2*(t.rotation.y**2+t.rotation.z**2))
    try:
        deadline=time.monotonic()+12
        initial=None
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            try: initial=pose();break
            except TransformException: pass
        if initial is None or not client.wait_for_server(timeout_sec=5):
            raise RuntimeError('TF or Nav2 action unavailable')
        x,y,yaw=initial
        goal=NavigateToPose.Goal()
        goal.pose.header.frame_id='map'
        goal.pose.header.stamp=node.get_clock().now().to_msg()
        gx,gy=x-.60*math.cos(yaw),y-.60*math.sin(yaw)
        heading=wrap(yaw+math.pi)
        goal.pose.pose.position.x,goal.pose.pose.position.y=gx,gy
        goal.pose.pose.orientation.z,goal.pose.pose.orientation.w=math.sin(heading/2),math.cos(heading/2)
        result.update(initial=list(initial),goal=[gx,gy,heading])
        print(f'TURN_CHECK: initial={initial}; goal behind axle=({gx:.3f},{gy:.3f},{heading:.3f})',flush=True)
        future=client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node,future,timeout_sec=5)
        handle=future.result()
        if handle is None or not handle.accepted: raise RuntimeError('Nav2 rejected goal')
        finished=handle.get_result_async()
        deadline=time.monotonic()+45
        while time.monotonic()<deadline and not finished.done():
            rclpy.spin_once(node,timeout_sec=.05)
            try:
                current=pose()
                trace.append(dict(time=node.get_clock().now().nanoseconds/1e9,pose=current,command=command.copy()))
            except TransformException: pass
        if finished.done():
            current=pose()
            position_error=math.hypot(current[0]-gx,current[1]-gy)
            yaw_error=abs(wrap(current[2]-heading))
            result.update(action_status=finished.result().status,final=list(current),
                          position_error=position_error,yaw_error=yaw_error,
                          success=finished.result().status==4 and position_error<=.16 and yaw_error<=.26)
        else:
            result['error']='45 second navigation timeout'
        print('TURN_CHECK result: '+json.dumps({k:v for k,v in result.items() if k!='trace'}),flush=True)
    except Exception as error:
        result['error']=str(error)
        print('TURN_CHECK failed: '+str(error),flush=True)
    finally:
        if handle is not None and rclpy.ok():
            cancel=handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node,cancel,timeout_sec=2)
        if rclpy.ok(): stop.publish(Twist())
        pending=args.output.with_suffix('.tmp')
        pending.write_text(json.dumps(result,indent=2))
        pending.replace(args.output)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return 0 if result['success'] else 1


if __name__=='__main__':
    raise SystemExit(main())
