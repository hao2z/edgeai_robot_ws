"""Bounded repositioning through existing authority/Nav2; no rescue target logic."""
import json
import math
import time
from pathlib import Path


def run(step, c):
    import rclpy
    import yaml
    from normal_rescue import Rescue
    from search_policy import observation_positions
    from nav2_msgs.action import NavigateToPose
    from std_srvs.srv import SetBool
    from tf2_ros import TransformException

    directory=Path(c['directory'])
    settings=yaml.safe_load((directory/'rescue-config.yaml').read_text())
    settings['wall_timeout_s']=c.get('mapping',{}).get('reposition_timeout_s',60.)+20.
    settings['search_frustum']={'enabled':False}
    settings['search_suspects']={'enabled':False}
    rclpy.init(args=['--ros-args','-p','use_sim_time:=true'])
    node=Rescue(settings,directory)
    terminal=sent=None
    try:
        if step=='resume_exploration':
            if c['backend']!='frontier':
                raise RuntimeError('Mapping incomplete: automatic exploration restart requires frontier backend')
            node.wait(lambda:node.state is not None and node.manual.service_is_ready(),10.)
            sequence=node.state['command_sequence']+1
            future=node.manual.call_async(SetBool.Request(data=False));node.wait(future.done,10.)
            if not future.result().success:raise RuntimeError('Authority rejected AUTO resume')
            def resumed():
                if node.state['command_sequence']>sequence:raise RuntimeError('Authority changed during AUTO resume')
                return (node.state['command_sequence']==sequence and node.state['mode']=='AUTO'
                        and not node.state['explorer_paused'])
            node.wait(resumed,15.)
            return
        node.acquire_control()
        pose=None
        def ready():
            nonlocal pose
            if node.search_map is None:return False
            try:
                _,pose=node.fresh_measurement(node.get_clock().now().nanoseconds/1e9,dict(x=0.,y=0.,yaw=0.))
                return True
            except (ValueError,TransformException):return False
        node.wait(ready,10.,check_owner=True)
        grid=node.search_map;q=grid.info.origin.orientation
        if grid.header.frame_id!='map' or abs(q.x)+abs(q.y)+abs(q.z)>1e-6:
            raise RuntimeError('Mapping supplement requires axis-aligned map')
        positions=observation_positions(grid.data,grid.info.width,grid.info.height,grid.info.resolution,
            (grid.info.origin.position.x,grid.info.origin.position.y),(pose['x'],pose['y']),
            settings['search_clearance_m'],.7,32)
        history_path=directory/'mapping-supplement-positions.json'
        history=json.loads(history_path.read_text()) if history_path.exists() else []
        positions=[p for p in positions if math.dist(p,(pose['x'],pose['y']))>=.6
                   and all(math.dist(p,old)>=.5 for old in history)]
        if not positions:raise RuntimeError('Mapping incomplete: no untried clear repositioning point')
        # Spatially separated viewpoints provide a new lidar angle; Nav2 validates the route.
        x,y=max(positions,key=lambda p:math.dist(p,(pose['x'],pose['y'])))
        history.append([x,y]);history_path.write_text(json.dumps(history))
        yaw=math.atan2(y-pose['y'],x-pose['x'])
        request=NavigateToPose.Goal();request.pose.header.frame_id='map'
        request.pose.header.stamp=node.get_clock().now().to_msg()
        request.pose.pose.position.x=x;request.pose.pose.position.y=y
        request.pose.pose.orientation.z=math.sin(yaw/2);request.pose.pose.orientation.w=math.cos(yaw/2)
        node.event('MAPPING_REPOSITION',json.dumps(dict(x=x,y=y,attempt=len(history))))
        sent=node.client.send_goal_async(request);node.wait(sent.done,10.,check_owner=True)
        node.goal_handle=sent.result()
        if not node.goal_handle.accepted:raise RuntimeError('Mapping reposition rejected')
        terminal=node.goal_handle.get_result_async()
        allowance=c.get('mapping',{}).get('reposition_timeout_s',60.)
        expires=time.monotonic()+allowance
        node.wait(lambda:terminal.done() or time.monotonic()>=expires,
                  allowance+2.,check_owner=True)
        intentional_cancel=False
        if not terminal.done():
            intentional_cancel=True
            node.event('MAPPING_REPOSITION_YIELD',json.dumps(dict(reason='reposition_timeout')))
            canceled=node.goal_handle.cancel_goal_async()
            node.wait(lambda:canceled.done() and terminal.done(),8.,check_owner=True)
        result=terminal.result();node.goal_handle=None
        if result.status==5 and not intentional_cancel:raise RuntimeError('Mapping reposition canceled externally')
        if result.status not in (4,5,6):raise RuntimeError('Unknown mapping reposition terminal state')
        # An aborted Nav2 approach still receives one fresh frontier evaluation after recovery.
        node.event('MAPPING_REPOSITION_RESULT',json.dumps(dict(status=result.status,error_code=result.result.error_code)))
        node.wait(lambda:node.state.get('phase')=='IDLE',5.,check_owner=True)
    finally:
        # Await the terminal action result before returning control, including timeout/cancellation.
        node.deadline=time.monotonic()+10.
        node.closing=True
        try:
            if sent is not None and terminal is None:
                node.wait(sent.done,5.)
                node.goal_handle=sent.result()
                if node.goal_handle is not None and node.goal_handle.accepted:
                    terminal=node.goal_handle.get_result_async()
            if terminal is not None and not terminal.done():
                canceled=node.goal_handle.cancel_goal_async()
                node.wait(lambda:canceled.done() and terminal.done(),8.)
                node.goal_handle=None
        finally:
            try:
                node.deadline=time.monotonic()+5.
                node.finish()
            finally:
                node.destroy_node()
                if rclpy.ok():rclpy.shutdown()
