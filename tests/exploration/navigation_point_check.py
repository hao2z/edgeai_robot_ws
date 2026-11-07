"""Manual-mode Gazebo point sequence; invoke through EDGEAI_MOTION_TEST=points."""
import argparse
import json
import math
import os
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist,PointStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry,Path as NavigationPath
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener,TransformException


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    args,ros_args=parser.parse_known_args();rclpy.init(args=ros_args)
    node=Node('navigation_point_check');buffer=Buffer();listener=TransformListener(buffer,node)
    state={};commands={};trace=[];attempts=[];handle=None;pending=None;plan_updates=[]
    node.create_subscription(NavigationPath,'/plan',lambda m:plan_updates.append(
        node.get_clock().now().nanoseconds/1e9),1)
    def received(msg):state.update(json.loads(msg.data))
    node.create_subscription(String,'/navigation_authority/state',received,
        QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    for topic in ('cmd_vel_nav','cmd_vel_smoothed','cmd_vel'):
        node.create_subscription(Twist,'/'+topic,
            lambda m,t=topic:commands.update({t:[m.linear.x,m.angular.z]}),1)
    node.create_subscription(Odometry,'/odom',lambda m:commands.update(measured_odom=[
        math.hypot(m.twist.twist.linear.x,m.twist.twist.linear.y),m.twist.twist.angular.z],
        measured_forward_mps=m.twist.twist.linear.x),qos_profile_sensor_data)
    if os.environ.get('EDGEAI_CONTROLLER')=='rpp':
        node.create_subscription(PointStamped,'/lookahead_point',lambda m:commands.update(
            rpp_lookahead=dict(frame=m.header.frame_id,x=m.point.x,y=m.point.y)),1)
    clients={kind:ActionClient(node,NavigateToPose,endpoint) for kind,endpoint in
             [('position','/navigate_to_pose'),('heading','/navigation/heading_to_pose'),('rescue','/navigation/rescue_to_pose')]}
    def wait(predicate,seconds):
        end=time.monotonic()+seconds
        while not predicate() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.03)
        if not predicate():raise RuntimeError('Point check wait timeout')
    def pose():
        tf=buffer.lookup_transform('map','base_footprint',Time()).transform
        q=tf.rotation
        return [tf.translation.x,tf.translation.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
    def pose_ready():
        try:pose();return True
        except TransformException:return False
    result=dict(success=False,scenario='simple_points',attempts=attempts)
    try:
        wait(lambda:state.get('mode')=='MANUAL' and state.get('phase')=='IDLE' and
             all(c.server_is_ready() for c in clients.values()) and pose_ready(),15.)
        origin=pose();ox,oy,angle=origin
        # Coordinates in the initial robot frame, within the open starting room.
        points=[('approach',1.,-.5,math.pi,'position'),('short_step',1.2,-.5,math.pi,'position'),
                ('turn',.65,-.75,0.,'position'),('return',.25,-.35,math.pi,'position'),
                ('heading',.65,-.5,1.,'heading'),('rescue_stop',.8,-.65,0.,'rescue')]
        if os.environ.get('EDGEAI_POINT_TEST_SCENARIO')=='u_wall':
            result['scenario']='u_wall'
            # First map both sides of W2, then issue one goal across the wall.
            # U legs use no intermediate goal, so their corner tracking is continuous.
            points=[('prepare_west',.3,-2.,0.,'position'),('prepare_north',2.6,-2.25,0.,'position'),
                    ('prepare_east',2.6,-3.1,0.,'position'),('prepare_south',.85,-3.85,0.,'position'),
                    ('u_east_to_west',.3,-2.,0.,'position'),('u_west_to_east',.85,-3.85,0.,'position')]
            if os.environ.get('EDGEAI_NAVIGATION_TEST')=='true':
                # Truth mode has absolute world coordinates and a complete map.
                ox=oy=angle=0.
                points=[('prepare_west',2.85,1.,0.,'position'),
                        ('u_west_to_east',4.5,1.4,0.,'position'),
                        ('u_east_to_west',2.85,1.,0.,'position')]
        if os.environ.get('EDGEAI_POINT_TEST_SCENARIO')=='narrow_u':
            if os.environ.get('EDGEAI_NAVIGATION_TEST')!='true':
                raise RuntimeError('narrow_u requires the complete truth map')
            result['scenario']='narrow_u';ox=oy=angle=0.
            # Corridor between O2 and W1, then around W1b to the other side.
            points=[('prepare_corridor',1.5,2.78,0.,'position'),
                    ('u_narrow_out',.48,3.87,0.,'position'),
                    ('u_narrow_return',1.5,2.78,0.,'position')]
        if os.environ.get('EDGEAI_POINT_TEST_SCENARIO')=='narrow_spaces':
            if os.environ.get('EDGEAI_NAVIGATION_TEST')!='true':
                raise RuntimeError('narrow_spaces requires the complete truth map')
            result['scenario']='narrow_spaces';ox=oy=angle=0.
            points=[('prepare_corridor',1.5,2.78,0.,'position'),
                    ('corridor_short_step',1.75,2.78,0.,'position'),
                    ('corridor_opposite_goal',1.4,2.78,0.,'position'),
                    ('marker_gap',3.78,.9,0.,'position'),
                    ('marker_gap_exit',3.78,1.65,0.,'position')]
        names=os.environ.get('EDGEAI_POINT_TEST_NAMES','').split(',')
        if names!=['']:
            points=[p for p in points if p[0] in names]
            if not points:raise RuntimeError('No matching point test names')
        points=points[:int(os.environ.get('EDGEAI_POINT_TEST_LIMIT','6'))]
        for name,x,y,yaw,kind in points:
            wait(lambda:state.get('phase')=='IDLE',8.)
            gx=ox+math.cos(angle)*x-math.sin(angle)*y
            gy=oy+math.sin(angle)*x+math.cos(angle)*y
            yaw+=angle
            request=NavigateToPose.Goal();request.pose.header.frame_id='map'
            request.pose.header.stamp=node.get_clock().now().to_msg()
            request.pose.pose.position.x=gx;request.pose.pose.position.y=gy
            request.pose.pose.orientation.z=math.sin(yaw/2);request.pose.pose.orientation.w=math.cos(yaw/2)
            start=node.get_clock().now().nanoseconds/1e9
            feedback={}
            sent=clients[kind].send_goal_async(request,feedback_callback=lambda m:feedback.update(
                recoveries=m.feedback.number_of_recoveries));wait(sent.done,5.)
            handle=sent.result()
            if not handle.accepted:raise RuntimeError('Point rejected')
            pending=handle.get_result_async();begin=time.monotonic();last=0.
            record=dict(name=name,kind=kind,goal=[gx,gy,yaw],started_sim_s=start)
            trace_start=len(trace)
            plan_start=len(plan_updates)
            attempts.append(record)
            wall_limit=180. if name.startswith('u_') or result['scenario']=='narrow_spaces' else 65.
            sim_limit=90. if name.startswith('u_') or result['scenario']=='narrow_spaces' else 30. if result['scenario']=='u_wall' else 20.
            while not pending.done() and time.monotonic()-begin<wall_limit:
                rclpy.spin_once(node,timeout_sec=.03)
                now=node.get_clock().now().nanoseconds/1e9
                if now-start>sim_limit:break
                if now-last>=.2:
                    p=pose();last=now
                    odom=buffer.lookup_transform('odom','base_footprint',Time()).transform.translation
                    trace.append(dict(name=name,t=now,pose=p,odom=[odom.x,odom.y],distance=math.hypot(p[0]-gx,p[1]-gy),commands=dict(commands)))
            if not pending.done():
                cancel=handle.cancel_goal_async();wait(cancel.done,3.);wait(pending.done,5.)
                record['timeout']=True
            terminal=pending.result();p=pose()
            record.update(status=terminal.status,error_code=terminal.result.error_code,
                elapsed_sim_s=node.get_clock().now().nanoseconds/1e9-start,
                position_error_m=math.hypot(p[0]-gx,p[1]-gy),
                yaw_error_rad=abs(math.atan2(math.sin(p[2]-yaw),math.cos(p[2]-yaw))))
            samples=trace[trace_start:];stationary=0.;longest=0.;rotate_time=0.
            path_length=0.
            for a,b in zip(samples,samples[1:]):
                dt=b['t']-a['t'];v,w=b['commands'].get('measured_odom',[0.,0.])
                path_length+=math.dist(a['odom'],b['odom'])
                if v<.01 and abs(w)<.05:stationary+=dt;longest=max(longest,stationary)
                else:stationary=0.
                if v<.01 and abs(w)>=.05:rotate_time+=dt
            record.update(path_length_m=path_length,max_stationary_s=longest,
                          in_place_rotation_s=rotate_time,recoveries=feedback.get('recoveries'),
                          global_plan_updates=len(plan_updates)-plan_start,
                          min_forward_command_mps=min((s['commands'].get('cmd_vel_nav',[0.])[0] for s in samples),default=0.),
                          min_forward_measured_mps=min((s['commands'].get('measured_forward_mps',0.) for s in samples),default=0.))
            record['success']=(terminal.status==4 and record['position_error_m']<= (.12 if kind=='rescue' else .08)
                               and (kind=='position' or record['yaw_error_rad']<=.27))
            if name.startswith('u_'):
                record['success']=record['success'] and path_length>=(2.8 if result['scenario']=='narrow_u' else 4.5)
            print('POINT_RESULT: '+json.dumps(record),flush=True)
            handle=None;pending=None
            if not record['success']:break
        result['success']=len(attempts)==len(points) and all(r['success'] for r in attempts)
    except Exception as exc:
        result['error']=str(exc)
    finally:
        if handle is not None and pending is not None and not pending.done():
            cancel=handle.cancel_goal_async()
            try:wait(cancel.done,3.);wait(pending.done,5.)
            except RuntimeError:pass
        args.output.parent.mkdir(parents=True,exist_ok=True)
        (args.output.parent/'point-trace.json').write_text(json.dumps(trace))
        temporary=args.output.with_suffix('.tmp');temporary.write_text(json.dumps(result,indent=2));temporary.replace(args.output)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return 0 if result['success'] else 1


if __name__=='__main__':raise SystemExit(main())
