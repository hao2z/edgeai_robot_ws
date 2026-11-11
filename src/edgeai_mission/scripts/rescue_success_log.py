"""Success evidence for display only; never changes the rescue decision."""
import json
import math
import time


def pose_text(pose):
    return f'({pose[0]:.3f}, {pose[1]:.3f}, {math.degrees(pose[2]):.1f}°)'


def truth_snapshot(directory, robot, stamp, target_id, radius, yaw_limit=.25, dwell=10.,
                   stopped_linear=.02, stopped_angular=.05):
    """Use only a fresh, synchronized truth record from this session."""
    path = directory/('truth.jsonl' if robot == 'robot1' else 'robot2-truth.jsonl')
    try:
        with path.open('rb') as stream:
            stream.seek(0, 2)
            position=stream.tell();data=b'';rows=[]
            # Usually one tail block; also supports replaying an older success.
            while position>0:
                begin=max(0,position-65536);stream.seek(begin)
                data=stream.read(position-begin)+data;position=begin
                rows=[]
                for line in data.splitlines():
                    try:rows.append(json.loads(line))
                    except (ValueError,UnicodeDecodeError):continue
                if any(r.get('sample_time',stamp+10)<=stamp-dwell-2 for r in rows):break
        target_snapshot=directory/'truth-targets.json'
        fallback_targets=json.loads(target_snapshot.read_text())['targets_world'] if target_snapshot.exists() else []
        for r in rows:
            if 'targets_world' not in r:r['targets_world']=fallback_targets
        # Keep the current clock epoch only after any simulator time reset.
        history=[]
        for r in rows:
            t=r.get('sample_time')
            if t is None:continue
            if history and t<history[-1]['sample_time']:history=[]
            if history and t==history[-1]['sample_time']:history[-1]=r
            else:history.append(r)
        row=next((r for r in reversed(history) if r['sample_time']<=stamp),{})
        valid=lambda r:r.get('truth_valid',r.get('status')=='ok')
        if not valid(row) or not 0 <= stamp-row['sample_time'] <= 1.0:
            raise ValueError('无新鲜且同步的真值样本')
        # Use the TASK interval, never shift it back to the last audit timestamp.
        begin=stamp-dwell
        def at(t):
            lower=next((r for r in reversed(history) if r['sample_time']<=t),None)
            upper=next((r for r in history if r['sample_time']>=t),None)
            if lower is None or upper is None or not valid(lower) or not valid(upper):return None
            dt=upper['sample_time']-lower['sample_time']
            if dt>1.5:return None
            p,q=lower['robot_world'],upper['robot_world']
            f=(t-lower['sample_time'])/dt if dt else 0.
            angle=math.atan2(math.sin(q[2]-p[2]),math.cos(q[2]-p[2]))
            return dict(lower,sample_time=t,robot_world=[p[0]+f*(q[0]-p[0]),p[1]+f*(q[1]-p[1]),p[2]+f*angle])
        first,last=at(begin),at(stamp)
        window=[r for r in history if begin<r['sample_time']<stamp]
        if first:window.insert(0,first)
        if last:window.append(last)
        complete=first is not None and last is not None and all(valid(r) for r in window)
        raw_stamp=row['sample_time']
        if last:row=last
        target = next(t for t in row['targets_world'] if t['id'] == target_id)
        pose = row['robot_world']
        distance = math.hypot(pose[0]-target['pose'][0], pose[1]-target['pose'][1])
        estimate = next((t for t in row.get('target_estimates', []) if t['id'] == target_id), {})
        yaw_error=math.atan2(math.sin(pose[2]-target['pose'][2]-math.pi),
                             math.cos(pose[2]-target['pose'][2]-math.pi))
        position_ok=distance<=radius;heading_ok=abs(yaw_error)<=yaw_limit
        max_distance=distance;max_yaw=abs(yaw_error);max_linear=max_angular=0.
        for previous,current in zip(window,window[1:]):
            dt=current['sample_time']-previous['sample_time']
            if not 0<dt<=1.5:complete=False;continue
            p=current['robot_world'];q=previous['robot_world']
            actual=next(t['pose'] for t in current['targets_world'] if t['id']==target_id)
            max_distance=max(max_distance,math.dist(p[:2],actual[:2]))
            max_yaw=max(max_yaw,abs(math.atan2(math.sin(p[2]-actual[2]-math.pi),math.cos(p[2]-actual[2]-math.pi))))
            max_linear=max(max_linear,math.dist(p[:2],q[:2])/dt)
            max_angular=max(max_angular,abs(math.atan2(math.sin(p[2]-q[2]),math.cos(p[2]-q[2])))/dt)
        if window:
            first=window[0];actual=next(t['pose'] for t in first['targets_world'] if t['id']==target_id)
            p=first['robot_world']
            max_distance=max(max_distance,math.dist(p[:2],actual[:2]))
            max_yaw=max(max_yaw,abs(math.atan2(math.sin(p[2]-actual[2]-math.pi),math.cos(p[2]-actual[2]-math.pi))))
        passed=position_ok and heading_ok and max_distance<=radius and max_yaw<=yaw_limit and max_linear<stopped_linear and max_angular<stopped_angular
        status='failed' if not passed else 'passed' if complete else 'unavailable'
        return dict(available=True, sample_time_s=raw_stamp, pose_time_s=stamp if last else raw_stamp,
                    coverage_pending=history[-1]['sample_time']<stamp, robot_world=pose,
                    target_world=target['pose'], distance_m=distance, radius_m=radius,
                    position_pass=position_ok,yaw_error_rad=yaw_error,heading_pass=heading_ok,
                    status=status,window_start_s=begin,window_end_s=stamp,
                    dwell_s=dwell,samples=len(window),
                    max_distance_m=max_distance,max_yaw_error_rad=max_yaw,
                    max_linear_mps=max_linear,max_angular_rps=max_angular,
                    target_error_m=estimate.get('error_m'))
    except (OSError, ValueError, KeyError, StopIteration, TypeError) as error:
        return dict(available=False, reason='真值记录读取失败：'+str(error))


def success_evidence(directory, task, target_id, target_pose, poses, distances, errors,
                     radius, yaw_limit, dwell, stamp, stopped_linear=.02, stopped_angular=.05,wait_s=0.):
    rows = []
    heading = f'{task.upper()} 救援成功｜目标 {target_id}｜map 目标位姿={pose_text(target_pose)}｜连续满足 {dwell:.1f} s'
    messages = [heading]
    deadline=time.monotonic()+wait_s
    while True:
        snapshots=[truth_snapshot(directory,f'robot{i+1}',stamp,target_id,radius,yaw_limit,dwell,
                                  stopped_linear,stopped_angular) for i in range(len(poses))]
        if not any(t.get('coverage_pending',False) for t in snapshots) or time.monotonic()>=deadline:break
        # Passive audit completion only: no navigation or sensor commands.
        time.sleep(.1)
    for i, pose in enumerate(poses):
        robot = f'robot{i+1}'
        truth = snapshots[i]
        rows.append(dict(robot=robot, frame_id='map', pose=pose, distance_m=distances[i],
                         yaw_error_rad=errors[i], truth=truth))
        messages.append(f'机器人{i+1}｜map 位姿={pose_text(pose)}｜距离 {distances[i]:.3f} ≤ {radius:.3f} m'
                        f'｜朝向误差 {abs(math.degrees(errors[i])):.1f}° ≤ {math.degrees(yaw_limit):.1f}°｜停车及停留检查通过')
        if truth['available']:
            truth['distance_error_m']=distances[i]-truth['distance_m']
            truth['yaw_estimation_error_rad']=math.atan2(math.sin(errors[i]-truth['yaw_error_rad']),math.cos(errors[i]-truth['yaw_error_rad']))
            text = (f'机器人{i+1}｜world 真值位姿={pose_text(truth["robot_world"])}'
                    f'｜目标真值位姿={pose_text(truth["target_world"])}'
                    f'｜真值距离 {truth["distance_m"]:.3f} / {radius:.3f} m'
                    f'｜真值朝向误差 {abs(math.degrees(truth["yaw_error_rad"])):.1f}° / {math.degrees(yaw_limit):.1f}°'
                    f'｜距离估计误差 {truth["distance_error_m"]:+.3f} m'
                    f'｜朝向估计误差 {math.degrees(truth["yaw_estimation_error_rad"]):+.1f}°'
                    f'｜样本滞后 {stamp-truth["sample_time_s"]:.2f} s')
            if truth['target_error_m'] is not None:
                text += f'｜目标定位误差 {truth["target_error_m"]:.3f} m'
            messages.append(text)
            label={'passed':'通过','failed':'未通过','unavailable':'无法确认'}[truth['status']]
            messages.append(f'真值核验：{label}｜机器人{i+1}｜{truth["samples"]}个历史样本，要求停留{dwell:.1f}s'
                            f'｜期间最大距离 {truth["max_distance_m"]:.3f} m'
                            f'｜最大朝向误差 {math.degrees(truth["max_yaw_error_rad"]):.1f}°'
                            f'｜采样估算最大速度 {truth["max_linear_mps"]:.3f} m/s、{truth["max_angular_rps"]:.3f} rad/s')
        else:
            messages.append(f'真值核验：无法确认｜机器人{i+1}｜{truth["reason"]}')
    statuses=[r['truth'].get('status','unavailable') for r in rows]
    status='failed' if 'failed' in statuses else 'passed' if all(s=='passed' for s in statuses) else 'unavailable'
    common_duration=None
    if status=='passed':
        common_duration=min(r['truth']['window_end_s'] for r in rows)-max(r['truth']['window_start_s'] for r in rows)
        if common_duration+1e-6<dwell:status='unavailable'
    messages.append(f'{task.upper()} 真值核验：'+{'passed':'通过','failed':'未通过','unavailable':'无法确认'}[status])
    return dict(task=task, target_id=target_id, frame_id='map', target_pose=target_pose,
                sample_time_s=stamp, radius_m=radius, yaw_tolerance_rad=yaw_limit,
                dwell_s=dwell, robots=rows,truth_verification=dict(status=status,common_duration_s=common_duration,
                    judge='Gazebo world pose history; sampled dwell check')), messages
