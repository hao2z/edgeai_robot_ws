"""Bounded final rescue corrections through Nav2 behavior actions."""
import math

from action_msgs.msg import GoalStatus
from nav2_msgs.action import Spin, DriveOnHeading
from tf2_ros import TransformException
from edgeai_tasks.rescue_policy import rescue_position, RescueAttemptFailed


def correction(config, goal, measurement):
    """Turn toward the goal before translating; finish with the required yaw."""
    distance=measurement['distance_m']
    tolerance=config['position_tolerance_m']
    task_distance,task_limit=rescue_position(config,goal,measurement)
    if task_distance>task_limit:
        bearing=math.atan2(goal['y']-measurement['y'],goal['x']-measurement['x'])
        angle=math.atan2(math.sin(bearing-measurement['yaw']),math.cos(bearing-measurement['yaw']))
        if abs(angle)>.10:
            return 'spin',max(-.6,min(.6,angle))
        return 'forward',min(config.get('alignment_step_m',.03),distance-.8*tolerance)
    if abs(measurement['yaw_error_rad'])>config['yaw_tolerance_rad']:
        return 'spin',max(-.6,min(.6,-measurement['yaw_error_rad']))
    return None


def adjust(node, goal, measurement):
    """One action, then return to the same independent measurement/dwell loop."""
    step=correction(node.c,goal,measurement)
    if step is None:return False
    kind,amount=step
    node.check_owner()
    if kind=='spin':
        client=node.spin_client
        request=Spin.Goal(target_yaw=amount)
    else:
        client=node.advance_client
        request=DriveOnHeading.Goal()
        request.target.x=amount
        request.speed=node.c.get('alignment_speed_mps',.03)
    request.time_allowance.sec=6
    node.wait(lambda:client.server_is_ready() and node.state.get('phase')=='IDLE',
              timeout=5.,check_owner=True)
    record=dict(kind=kind,amount=amount,before=dict(measurement),
                started_sim_s=node.get_clock().now().nanoseconds/1e9)
    node.alignment_attempts.append(record)
    node.event('ALIGN_ACTION',
        f"位置误差={measurement['distance_m']:.3f} m，朝向误差={math.degrees(measurement['yaw_error_rad']):.1f}°；"
        +(f'单独旋转 {math.degrees(amount):.1f}°' if kind=='spin' else f'短距前进 {amount:.3f} m'))
    sent=client.send_goal_async(request)
    node.wait(sent.done,timeout=5.,check_owner=True)
    node.goal_handle=sent.result()
    if not node.goal_handle.accepted:
        node.goal_handle=None
        record['outcome']='rejected'
        raise RescueAttemptFailed('Rescue final correction rejected','correction')
    handle=node.goal_handle
    result=handle.get_result_async()
    cancel=None
    def ready():
        nonlocal cancel
        if result.done():return True
        now=node.get_clock().now().nanoseconds/1e9
        try:
            sample_time,current=node.fresh_measurement(now,goal)
        except (ValueError, TransformException):
            return False
        if sample_time<record['started_sim_s']:return False
        next_step=correction(node.c,goal,current)
        before_distance,limit=rescue_position(node.c,goal,measurement)
        current_distance,_=rescue_position(node.c,goal,current)
        drifted=before_distance<=limit<current_distance
        if next_step is None or next_step[0]!=kind or drifted:
            record['recheck_measurement']=current
            record['cancel_reason']='conditions_changed'
            cancel=handle.cancel_goal_async()
            return True
        return False
    try:
        node.wait(ready,timeout=12.,check_owner=True)
        if cancel is not None:
            node.wait(lambda:cancel.done() and result.done(),timeout=5.,check_owner=True)
            if not any(bytes(g.goal_id.uuid)==bytes(handle.goal_id.uuid) for g in cancel.result().goals_canceling):
                if result.result().status!=GoalStatus.STATUS_SUCCEEDED:
                    raise RuntimeError('Rescue correction cancellation not acknowledged')
    except Exception:
        cancel=handle.cancel_goal_async()
        node.wait(lambda:cancel.done() and result.done(),timeout=5.)
        node.goal_handle=None
        raise
    terminal=result.result();node.goal_handle=None
    record.update(status=terminal.status,error_code=terminal.result.error_code,
                  finished_sim_s=node.get_clock().now().nanoseconds/1e9)
    own_cancel=cancel is not None and terminal.status==GoalStatus.STATUS_CANCELED
    if not own_cancel and (terminal.status!=GoalStatus.STATUS_SUCCEEDED or terminal.result.error_code!=0):
        message=f'Rescue final correction failed: {kind}, status={terminal.status}, error={terminal.result.error_code}'
        if terminal.status==GoalStatus.STATUS_ABORTED:
            raise RescueAttemptFailed(message,'correction',terminal.result.error_code)
        raise RuntimeError(message)
    node.event('ALIGN_RESULT','修正动作完成，重新检查位姿和停车条件')
    return True


def adjust_joint(units):
    """One bounded S1-style correction, then recheck BOTH robots in the tree."""
    from edgeai_tasks.cooperative_policy import rescue_yaw_error
    from nav2_msgs.action import Spin,DriveOnHeading
    from rclpy.time import Time
    n=units.node;units.check_owner()
    now=n.now()
    if units.alignment_started is None:units.alignment_started=now
    attempts=getattr(units,'joint_corrections',[])
    if now-units.alignment_started>n.c.get('alignment_timeout_s',60.) or len(attempts)>=n.c.get('alignment_max_actions',40):
        raise RescueAttemptFailed('Joint correction budget exhausted','correction')
    for i,ns in enumerate(('','robot2')):
        stamp=Time.from_msg(n.odom[ns].header.stamp)
        if not 0<=now-stamp.nanoseconds/1e9<=n.c['max_data_age_s']:
            return units.wait_joint_data(now,'Correction waiting for fresh odometry')
        try:pose=n.tf_pose('robot2/base_footprint' if ns else 'base_footprint',stamp)
        except TransformException as error:return units.wait_joint_data(now,str(error))
        parking=units.selected[i]
        yaw=math.atan2(n.target['y']-parking[1],n.target['x']-parking[0])
        offset=math.dist(parking[:2],[n.target['x'],n.target['y']])
        goal=dict(x=parking[0],y=parking[1],yaw=yaw,approach_distance_m=offset)
        measurement=dict(x=pose[0],y=pose[1],yaw=pose[2],distance_m=math.dist(pose[:2],parking[:2]),
                         yaw_error_rad=rescue_yaw_error(pose,n.target))
        config=dict(n.c,goal_source='detected_target',target_radius_m=n.c['radius_m'],
                    normal_offset_m=offset,position_tolerance_m=n.c.get('alignment_position_tolerance_m',.10))
        step=correction(config,goal,measurement)
        if step is None:continue
        kind,amount=step
        units.reset_dwell()
        n.wait(lambda:n.states.get(ns,{}).get('phase')=='IDLE',5.,True)
        if kind=='spin':client=n.spin[ns];request=Spin.Goal(target_yaw=amount)
        else:
            client=n.advance[ns];request=DriveOnHeading.Goal()
            request.target.x=amount;request.speed=n.c.get('alignment_speed_mps',.03)
        request.time_allowance.sec=6
        record=dict(robot=ns or 'robot1',kind=kind,amount=amount,started_sim_s=now)
        attempts.append(record);units.joint_corrections=attempts
        n.event('ALIGN_ACTION',**record)
        terminal=n.action(client,request,12.)
        record.update(status=terminal.status,error_code=terminal.result.error_code)
        if terminal.status!=4 or terminal.result.error_code:
            raise RescueAttemptFailed('Joint correction action failed: '+str(record),'correction')
        units.navigation_after=n.now()
        n.event('ALIGN_RESULT',**record)
        return dict(corrected=True,robot=record['robot'],kind=kind)
    return dict(corrected=False)
