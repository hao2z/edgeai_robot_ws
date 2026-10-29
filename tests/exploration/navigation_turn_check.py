"""Focused Nav2 check: a reachable goal 0.60 m behind the initial axle pose."""
import argparse
import json
import math
import os
from pathlib import Path
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, TransformStamped
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformException, TransformListener, StaticTransformBroadcaster


def wrap(angle):
    return math.atan2(math.sin(angle),math.cos(angle))


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    args,ros_args=parser.parse_known_args()
    scenario=os.environ.get('EDGEAI_MOTION_TEST') or 'turn'
    if scenario not in ('forward','reverse','turn','axes'): raise ValueError(scenario)
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
        if scenario=='axes':
            measurements=[]
            for name,v,w in [('forward',.05,0.),('reverse',-.05,0.),('left',0.,.20),('right',0.,-.20)]:
                start_pose=pose()
                start_sim=node.get_clock().now().nanoseconds/1e9
                deadline=time.monotonic()+8
                while node.get_clock().now().nanoseconds/1e9-start_sim < 1.0:
                    if time.monotonic()>deadline: raise RuntimeError('Simulation clock stalled during axis test')
                    twist=Twist();twist.linear.x=v;twist.angular.z=w
                    stop.publish(twist)
                    rclpy.spin_once(node,timeout_sec=.02)
                stop.publish(Twist())
                until=time.monotonic()+.5
                while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
                end=pose()
                displacement=(end[0]-start_pose[0])*math.cos(start_pose[2])+(end[1]-start_pose[1])*math.sin(start_pose[2])
                rotation=wrap(end[2]-start_pose[2])
                passed=displacement*v>.0005 if v else rotation*w>.005
                measurements.append(dict(name=name,command=[v,w],start=start_pose,end=end,
                                         forward_displacement=displacement,rotation=rotation,success=passed))
                print('AXIS_CHECK: '+json.dumps(measurements[-1]),flush=True)
            result.update(scenario=scenario,measurements=measurements,success=all(v['success'] for v in measurements))
            return 0 if result['success'] else 1
        goal=NavigateToPose.Goal()
        goal.pose.header.frame_id='map'
        goal.pose.header.stamp=node.get_clock().now().to_msg()
        distance=.60 if scenario=='forward' else -.60
        gx,gy=x+distance*math.cos(yaw),y+distance*math.sin(yaw)
        heading=wrap(yaw+math.pi) if scenario=='turn' else yaw
        result['scenario']=scenario
        goal.pose.pose.position.x,goal.pose.pose.position.y=gx,gy
        goal.pose.pose.orientation.z,goal.pose.pose.orientation.w=math.sin(heading/2),math.cos(heading/2)
        if os.environ.get('EDGEAI_MOTION_FRAME')=='rotated':
            broadcaster=StaticTransformBroadcaster(node)
            transform=TransformStamped()
            transform.header.frame_id='map';transform.child_frame_id='motion_test_rotated'
            transform.header.stamp=node.get_clock().now().to_msg()
            transform.transform.rotation.z=1.0
            transform.transform.rotation.w=0.0
            broadcaster.sendTransform(transform)
            until=time.monotonic()+1.0
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
            goal.pose.header.frame_id='motion_test_rotated'
            goal.pose.header.stamp=node.get_clock().now().to_msg()
            goal.pose.pose.position.x,goal.pose.pose.position.y=-gx,-gy
            goal.pose.pose.orientation.z=math.sin((heading-math.pi)/2)
            goal.pose.pose.orientation.w=math.cos((heading-math.pi)/2)
        result['goal_frame']=goal.pose.header.frame_id
        result.update(initial=list(initial),goal=[gx,gy,heading])
        print(f'MOTION_CHECK {scenario}: initial={initial}; goal=({gx:.3f},{gy:.3f},{heading:.3f})',flush=True)
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
                progress=(current[0]-x)*math.cos(yaw)+(current[1]-y)*math.sin(yaw)
                if scenario=='forward' and progress < -.10:
                    result['error']='Direction check failed: moved more than 0.10 m away from a forward goal'
                    break
            except TransformException: pass
        current=pose()
        result.update(final=list(current),position_error=math.hypot(current[0]-gx,current[1]-gy),
                      yaw_error=abs(wrap(current[2]-heading)))
        if finished.done():
            current=pose()
            position_error=math.hypot(current[0]-gx,current[1]-gy)
            yaw_error=abs(wrap(current[2]-heading))
            result.update(action_status=finished.result().status,final=list(current),
                          position_error=position_error,yaw_error=yaw_error,
                          success=finished.result().status==4 and position_error<=.16 and yaw_error<=.26)
        else:
            result.setdefault('error','45 second navigation timeout')
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
