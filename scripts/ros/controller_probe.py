"""Bounded, read-only controller sampling. Duration and freshness use wall time."""
import argparse
from collections import Counter,deque
from datetime import datetime
import json
import math
import os
from pathlib import Path
import time

VELOCITIES=('cmd_vel_nav','cmd_vel_smoothed','cmd_vel','odom')
ACTION_STATUS={0:'UNKNOWN',1:'ACCEPTED',2:'EXECUTING',3:'CANCELING',4:'SUCCEEDED',5:'CANCELED',6:'ABORTED'}


def controller_view(row,controllers):
    values=row['values'];ages=row['wall_age_s'];authority=values.get('authority',{})
    policy=authority.get('arrival_policy')
    name={'position':'FollowPath','heading':'FollowPathHeading','rescue':'FollowPathRescue'}.get(policy)
    config=controllers.get(name,{}) if name else {}
    status=values.get('follow_path_status',{})
    feedback=values.get('follow_path_feedback',{})
    if feedback.get('goal_id')!=status.get('goal_id'):feedback={}
    return {'controller_id':name,'plugin':config.get('plugin'),'primary':config.get('primary_controller'),
            'authority_fresh':ages.get('authority',math.inf)<=.5,
            'following_path':authority.get('following_path'),
            'follow_path':status,'status_wall_age_s':ages.get('follow_path_status'),
            'feedback':feedback,'feedback_wall_age_s':ages.get('follow_path_feedback') if feedback else None,
            'rpp_rotating':values.get('rpp_rotating'),
            'rpp_rotating_wall_age_s':ages.get('rpp_rotating'),
            'lookahead':values.get('lookahead'),'lookahead_wall_age_s':ages.get('lookahead'),
            'tracking_plan':values.get('tracking_plan'),'tracking_plan_wall_age_s':ages.get('tracking_plan'),
            'rpp_tracking':values.get('rpp_tracking'),'rpp_tracking_wall_age_s':ages.get('rpp_tracking')}


def sample_flags(row, freshness=.5):
    values=row['values'];ages=row['wall_age_s'];flags=[]
    def fresh(*names):
        return all(name in values and ages.get(name,math.inf)<=freshness for name in names)
    for before,after,label in (('cmd_vel_nav','cmd_vel_smoothed','smoother_difference'),
                               ('cmd_vel_smoothed','cmd_vel','monitor_difference')):
        if fresh(before,after) and (abs(values[before][0]-values[after][0])>.03 or
                                   abs(values[before][1]-values[after][1])>.08):
            flags.append(label)
    if fresh('cmd_vel','odom'):
        v,w=values['cmd_vel'];mv,mw=values['odom']
        if (abs(v)>.03 or abs(w)>.1) and abs(mv)<.01 and abs(mw)<.05:
            flags.append('command_without_measured_motion')
    return flags


def summarize(rows,counts,elapsed):
    report={'wall_duration_s':round(elapsed,3),'samples':len(rows),'channels':{},
            'difference_samples':dict(Counter(flag for row in rows for flag in row['flags']))}
    for name in VELOCITIES:
        values=[row['values'][name] for row in rows
                if name in row['values'] and row['wall_age_s'].get(name,math.inf)<=.5]
        report['channels'][name]={'messages':counts.get(name,0),
                                  'received_wall_hz':round(counts.get(name,0)/max(elapsed,.001),2),
                                  'fresh_samples':len(values)}
        if values:
            report['channels'][name].update(min_v=round(min(v[0] for v in values),4),
                max_v=round(max(v[0] for v in values),4),max_abs_w=round(max(abs(v[1]) for v in values),4))
    longest=current=0.;previous=None
    for row in rows:
        if 'command_without_measured_motion' in row['flags']:
            if previous is not None:current+=row['wall_elapsed_s']-previous
            longest=max(longest,current)
        else:current=0.
        previous=row['wall_elapsed_s']
    report['longest_command_without_motion_wall_s']=round(longest,3)
    report['local_controller']={
        'status_samples':dict(Counter(r.get('controller',{}).get('follow_path',{}).get('name','NO_DATA') for r in rows)),
        'latest_observation':rows[-1].get('controller') if rows else None}
    report['interpretation']='差异采样包含正常加减速和异步到达，不代表已确定故障原因；Hz和持续时间为墙钟时间。'
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration',type=float,default=20.)
    parser.add_argument('--hz',type=float,default=10.)
    parser.add_argument('--session',type=Path)
    args=parser.parse_args()
    if not (1<=args.duration<=120 and 1<=args.hz<=20):parser.error('duration 1..120, hz 1..20')
    root=Path(__file__).resolve().parents[2]
    session=args.session or Path((root/'runtime/latest-session.txt').read_text().strip())
    meta=json.loads((session/'session.json').read_text())
    os.environ['ROS_DOMAIN_ID']=str(meta['domain'])
    # Import ROS after selecting the session's domain. No publishers/action clients.
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
    from geometry_msgs.msg import Twist,PointStamped
    from nav_msgs.msg import Odometry,Path as NavPath
    from nav2_msgs.msg import CollisionMonitorState
    from nav2_msgs.action import FollowPath
    from action_msgs.msg import GoalStatusArray
    from rcl_interfaces.msg import Log
    from std_msgs.msg import String,Bool
    import yaml
    config_path=session/'nav2-runtime.yaml'
    startup=yaml.safe_load(config_path.read_text()) if config_path.is_file() else {}
    controllers=startup.get('controller_server',{}).get('ros__parameters',{})

    output=session/'controller-diagnostics'/f'{datetime.now():%Y%m%d-%H%M%S}-{os.getpid()}'
    output.mkdir(parents=True)
    rclpy.init()
    node=Node('controller_short_probe',parameter_overrides=[Parameter('use_sim_time',value=True)])
    values={};received={};counts=Counter();warnings=Counter();node_warnings=Counter()
    recent=deque(maxlen=20);examples={};rows=[]
    def record(name,value):
        values[name]=value;received[name]=time.monotonic();counts[name]+=1
    def json_record(name,data):
        try:record(name,json.loads(data))
        except (ValueError,TypeError):warnings['malformed_'+name]+=1
    subscriptions=[]
    for name in VELOCITIES[:-1]:
        subscriptions.append(node.create_subscription(Twist,'/'+name,
            lambda m,key=name:record(key,[m.linear.x,m.angular.z]),qos_profile_sensor_data))
    def odometry(m):
        record('odom',[m.twist.twist.linear.x,m.twist.twist.angular.z])
        record('odom_stamp_s',m.header.stamp.sec+m.header.stamp.nanosec/1e9)
    subscriptions.append(node.create_subscription(Odometry,'/odom',odometry,qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(String,'/navigation_authority/state',
        lambda m:json_record('authority',m.data),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    subscriptions.append(node.create_subscription(String,'/navigation/rpp_tracking',
        lambda m:json_record('rpp_tracking',m.data),qos_profile_sensor_data))
    def follow_status(m):
        if not m.status_list:return
        latest=max(m.status_list,key=lambda s:(s.goal_info.stamp.sec,s.goal_info.stamp.nanosec))
        record('follow_path_status',{'status':latest.status,'name':ACTION_STATUS.get(latest.status,'?'),
                                   'goal_id':bytes(latest.goal_info.goal_id.uuid).hex()})
    subscriptions.append(node.create_subscription(GoalStatusArray,'/navigation_backend/follow_path/_action/status',
        follow_status,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    subscriptions.append(node.create_subscription(FollowPath.Impl.FeedbackMessage,
        '/navigation_backend/follow_path/_action/feedback',lambda m:record('follow_path_feedback',
            {'goal_id':bytes(m.goal_id.uuid).hex(),'distance_to_goal_m':m.feedback.distance_to_goal,
             'speed_mps':m.feedback.speed}),qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(PointStamped,'/lookahead_point',
        lambda m:record('lookahead',{'frame':m.header.frame_id,'x':m.point.x,'y':m.point.y}),qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(Bool,'/is_rotating_to_heading',
        lambda m:record('rpp_rotating',m.data),qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(CollisionMonitorState,'/collision_monitor_state',
        lambda m:record('collision_monitor',{'action':m.action_type,'polygon':m.polygon_name}),qos_profile_sensor_data))
    for name,topic in (('global_plan','/plan'),('tracking_plan','/received_global_plan')):
        def plan(m,key=name):
            item={'frame':m.header.frame_id,'poses':len(m.poses)}
            if m.poses:
                p=m.poses[-1].pose.position;item['end']=[p.x,p.y]
            record(key,item)
        subscriptions.append(node.create_subscription(NavPath,topic,plan,qos_profile_sensor_data))
    def log(m):
        if m.level<30 or not (m.name in ('controller_server','planner_server','behavior_server',
                                        'collision_monitor','navigation_authority') or m.name.startswith('bt_navigator')):return
        lower=m.msg.lower()
        kind=next((key for key,terms in (('collision',('collision',)),('progress',('progress','patience')),
                  ('tf',('transform','extrapolation','tf ')),('rate',('loop','rate')))
                  if any(term in lower for term in terms)),'other')
        warnings[kind]+=1
        node_warnings[m.name]+=1
        event={'node':m.name,'level':m.level,'message':m.msg[:400]}
        recent.append(event);examples[kind]=event
    subscriptions.append(node.create_subscription(Log,'/rosout',log,qos_profile_sensor_data))
    # Copy only startup inputs for reproducibility; this is not a live parameter dump.
    for name in ('motion-limits.json','nav2-runtime.yaml'):
        if (session/name).is_file():(output/name).write_bytes((session/name).read_bytes())
    print(f'控制器短时诊断：域 {meta["domain"]}，{args.duration:g} 墙钟秒，{args.hz:g} Hz\n记录：{output}',flush=True)
    print('每项显示 v(m/s)/w(rad/s)；差异仅作为排查线索。',flush=True)
    begin=time.monotonic();next_sample=begin;next_print=begin
    interrupted=False
    try:
        with (output/'samples.jsonl').open('w',encoding='utf-8') as stream:
            while rclpy.ok() and time.monotonic()-begin<args.duration:
                rclpy.spin_once(node,timeout_sec=min(.03,max(0.,next_sample-time.monotonic())))
                now=time.monotonic()
                if now<next_sample:continue
                sim=node.get_clock().now().nanoseconds/1e9
                row={'wall_elapsed_s':round(now-begin,3),'sim_time_s':sim,
                     'wall_age_s':{k:round(now-v,3) for k,v in received.items()},'values':dict(values)}
                if values.get('odom_stamp_s',0)>0:row['odom_sim_age_s']=sim-values['odom_stamp_s']
                row['flags']=sample_flags(row);row['controller']=controller_view(row,controllers);rows.append(row)
                stream.write(json.dumps(row,ensure_ascii=False)+'\n')
                next_sample=now+1/args.hz
                if now>=next_print:
                    def speed(key):
                        if key not in values:return '无数据'
                        if now-received[key]>.5:return '过期'
                        return f'{values[key][0]:+.2f}/{values[key][1]:+.2f}'
                    a=values.get('authority',{});cm=values.get('collision_monitor',{})
                    cv=row['controller'];feedback=cv['feedback']
                    goal_distance=feedback.get('distance_to_goal_m')
                    distance_text=f'{goal_distance:.2f}m' if goal_distance is not None else '?'
                    feedback_age=cv['feedback_wall_age_s']
                    age_text=f'{feedback_age:.2f}s' if feedback_age is not None else '?'
                    idle=a.get('phase')=='IDLE' and not a.get('following_path')
                    print(f'局部控制器 {cv["controller_id"] or ("空闲" if idle else "未确认")} | '
                          f'最新FollowPath={cv["follow_path"].get("name","无数据")} '
                          f'反馈年龄={age_text} 距离={distance_text} '
                          f'RPP转向={cv["rpp_rotating"]} '
                          f'跟踪报告={(cv["rpp_tracking"] or {}).get("status","?")} 前视={cv["lookahead"]}',flush=True)
                    print(f'{now-begin:5.1f}s 导航 {speed("cmd_vel_nav")} → 平滑 {speed("cmd_vel_smoothed")} '
                          f'→ 最终 {speed("cmd_vel")} | 实测 {speed("odom")} '
                          f'| {a.get("phase","?")}/{a.get("arrival_policy") or "-"} '
                          f'恢复记录={bool(a.get("recovery"))} 碰撞动作={cm.get("action","?")}',flush=True)
                    next_print=now+2.
    except KeyboardInterrupt:interrupted=True
    finally:
        elapsed=time.monotonic()-begin
        summary=summarize(rows,counts,elapsed)
        summary.update(session=str(session),domain=meta['domain'],interrupted=interrupted,
                       warning_counts=dict(warnings),recent_warnings=list(recent),
                       warning_counts_by_node=dict(node_warnings),warning_examples=examples,
                       configured_controllers={name:controllers[name] for name in controllers.get('controller_plugins',[])},
                       startup_config_only=True)
        (output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
        lines=['控制器诊断摘要',f'墙钟 {elapsed:.1f}s，采样 {len(rows)} 次']
        lines.append('局部控制器状态采样：'+str(summary['local_controller']['status_samples']))
        latest=summary['local_controller']['latest_observation'] or {}
        lines.append(f'最后实例：{latest.get("controller_id")}，primary={latest.get("primary")}；'
                     f'反馈={latest.get("feedback")}，年龄={latest.get("feedback_wall_age_s")}s')
        for key,item in summary['channels'].items():
            lines.append(f'{key}: {item["received_wall_hz"]:.1f} Hz，鲜活样本 {item["fresh_samples"]}/{len(rows)}'
                +(f'，v=[{item["min_v"]:.2f},{item["max_v"]:.2f}]，|w|≤{item["max_abs_w"]:.2f}' if 'min_v' in item else '，未收到数据'))
        lines.extend([f'差异样本：{summary["difference_samples"]}',
                      f'告警计数：{dict(warnings)}',summary['interpretation'],f'记录：{output}'])
        for kind in ('collision','progress','tf','other','rate'):
            if kind in examples:lines.append(f'{kind}: [{examples[kind]["node"]}] {examples[kind]["message"]}')
        (output/'summary.txt').write_text('\n'.join(lines)+'\n',encoding='utf-8')
        print('\n'+'\n'.join(lines),flush=True)
        node.destroy_node();rclpy.shutdown()
    return 0 if counts['odom'] else 2


if __name__=='__main__':raise SystemExit(main())
