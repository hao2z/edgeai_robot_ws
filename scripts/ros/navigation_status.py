"""Pure, read-only status rendering. Observations are never navigation decisions."""
import html
import math
from pathlib import Path
import yaml


def runtime_settings(directory):
    path = Path(directory)/'nav2-runtime.yaml' if directory else None
    if path is None or not path.is_file():
        return {}
    config = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    local = config.get('local_costmap', {}).get('local_costmap', {}).get('ros__parameters', {})
    layer = local.get('depth_layer', {})
    return dict(controllers=config.get('controller_server', {}).get('ros__parameters', {}),
                depth_enabled='depth_layer' in local.get('plugins', []) and layer.get('enabled', True),
                depth_hz=config.get('depth_obstacle_filter', {}).get('ros__parameters', {}).get('processing_hz'),
                local_frame=local.get('global_frame'),
                planners=config.get('planner_server', {}).get('ros__parameters', {}))


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def number(value, unit='', digits=2):
    return f'{value:.{digits}f}{unit}' if finite(value) else '—'


def escape(value):
    return html.escape(str(value))


def render(snapshot):
    """Return compact body, collapsible details, legend and auditable observations."""
    a = snapshot.get('authority') or {}
    settings = snapshot.get('settings', {})
    ages = snapshot.get('ages', {})
    wall_ages = snapshot.get('wall_ages', {})
    authority_fresh = bool(a) and snapshot.get('authority_age', math.inf) <= 2.
    active = authority_fresh and a.get('phase') == 'ACTIVE'
    recovery = a.get('recovery') or {}
    warnings = []
    heading, color = '等待任务状态', '#786000'
    if snapshot.get('stopping'):
        heading, color = '任务会话正在结束', '#555555'
    elif snapshot.get('clock_paused'):
        heading, color = '仿真时钟暂停或未推进', '#a45a00'
    elif snapshot.get('standby'):
        heading, color = '待命：导航尚未启动', '#555555'
    elif not authority_fresh:
        warnings.append('任务状态缺失或超过2秒未更新，不能确认当前执行阶段')
    elif a.get('phase') == 'UNKNOWN':
        heading, color = '任务执行状态未知', '#b3261e'
    elif active and recovery.get('outcome') == 'RUNNING':
        heading, color = '正在执行恢复动作', '#a45a00'
    elif active and a.get('following_path'):
        heading, color = '正在跟踪路径', '#17713c'
    elif active:
        heading, color = '任务活动：规划、等待或行为执行中', '#786000'
    elif a.get('phase') == 'IDLE':
        heading, color = '当前没有活动导航目标', '#555555'
    else:
        heading = {'SENDING':'等待Nav2接单','CANCELING':'等待任务取消完成'}.get(a.get('phase'), '等待任务状态')

    def fresh(key, limit=.6):
        age = ages.get(key)
        return finite(age) and -.2 <= age <= limit and wall_ages.get(key, math.inf) <= 2.

    def velocity(key):
        values = snapshot.get('velocities', {}).get(key)
        limit = .6 if key == 'odom' else 2.
        age_ok = fresh(key) if key == 'odom' else wall_ages.get(key, math.inf) <= limit
        if values is None or not age_ok:
            return '无新数据'
        return number(values[0], ' m/s')+' / '+number(values[1], ' rad/s')

    if active and not snapshot.get('clock_paused') and not snapshot.get('stopping'):
        if snapshot.get('stationary_s', 0.) >= 5.:
            warnings.append('路径跟踪中已静止 '+number(snapshot['stationary_s'], ' s')+'：需核对控制输出与碰撞日志')
            heading, color = '路径跟踪停滞，原因待核查', '#a45a00'
        for key, label in [('scan','雷达'), ('odom','里程计'), ('local_costmap','局部代价图')]:
            if not fresh(key):
                warnings.append(label+'数据缺失或过期')
        if a.get('fault'):
            warnings.append(str(a['fault']))
        if snapshot.get('problems'):
            warnings.append('同刻TF或轮廓投影不可用，查看数据时序细节')
        for key, label in [('global_plan','全局路径'), ('local_plan','局部轨迹')]:
            if finite(ages.get(key)) and ages[key] > 2.:
                warnings.append(label+'已 '+number(ages[key], ' s')+' 未更新，图上可能是旧路线')
    tracking = snapshot.get('tracking') or {}
    tracking_fresh = snapshot.get('tracking_age', math.inf) <= 1.
    if active and a.get('following_path') and tracking_fresh and tracking.get('status') == 'BLOCKED':
        warnings.append('RPP报告BLOCKED；这不是物理碰撞的证明')
        heading, color = 'RPP控制计算受阻', '#a45a00'

    def sensor(key):
        if key not in ages:
            return '等待数据'
        return ('新鲜' if fresh(key) else '过期/时间异常')+' · '+number(ages[key], ' s')

    depth = settings.get('depth_enabled')
    depth_text = ('已关闭，单雷达' if depth is False else '配置未知')
    if snapshot.get('camera_available') is False:
        depth_text = '本仿真未启用 RGB-D'
        if depth is True:
            depth_text += ' · 代价层仍配置为开启'
    elif depth is True:
        depth_text = '已配置开启 · '+sensor('depth_cloud')
        if snapshot.get('rates', {}).get('depth_cloud') is not None:
            depth_text += ' · '+number(snapshot['rates']['depth_cloud'], ' Hz', 1)
        if not fresh('depth_cloud') and not snapshot.get('clock_paused') and not snapshot.get('stopping'):
            warnings.append('深度输入未就绪或过期；不能据此假定旧深度障碍已清除')
        pipeline = snapshot.get('depth_filter') or {}
        if pipeline and snapshot.get('depth_filter_age',math.inf)<=3.:
            states = dict(WAITING_INPUT='处理节点未收到原始点云',INPUT_STALE='原始点云已断流',
                          NO_OUTPUT='已收到原始点云，但尚未输出',OUTPUT_STALE='过滤输出已断流',OUTPUT_OK='处理节点正在输出')
            depth_text += ' · '+states.get(pipeline.get('state'),'处理状态未知')
            depth_text += (f' · 收{pipeline.get("input_messages",0)}/发{pipeline.get("published_messages",0)}'
                           f' · 过期丢弃{pipeline.get("expired_messages",0)}')
        elif not fresh('depth_cloud'):
            depth_text += ' · 处理节点状态未收到，检查nav.log/节点存活'

    policy = a.get('arrival_policy')
    controller_id = {'position':'FollowPath','heading':'FollowPathHeading','rescue':'FollowPathRescue'}.get(policy,'FollowPath')
    controller = settings.get('controllers', {}).get(controller_id, {})
    primary = controller.get('primary_controller', controller.get('plugin','未知'))
    plugin = str(primary).split('::')[-1]
    planner_id = 'GridBasedHeading' if policy in ('heading','rescue') else 'GridBased'
    planner = settings.get('planners', {}).get(planner_id, {}).get('plugin','未知').split('::')[-1]
    rows = []
    def row(name, value):
        rows.append('<tr><td><b>'+escape(name)+'</b></td><td>'+escape(value)+'</td></tr>')
    mode = {'AUTO':'自动探索','MANUAL':'手动/任务接管','STOPPED':'停止'}.get(a.get('mode'),'未知')
    row('任务', f'{mode} · #{a.get("generation","—")} · '+
        {'position':'位置到点','heading':'朝向到点','rescue':'救援到点','frontier':'前沿位置到点'}.get(policy,'无当前到点策略'))
    action = snapshot.get('navigation_action') or {}
    if action:
        name = {1:'已接受',2:'执行中',3:'取消中',4:'成功',5:'已取消',6:'已中止'}.get(action.get('status'),'未知')
        matched = authority_fresh and bool(a.get('goal_uuid')) and action.get('goal_id')==a.get('goal_uuid')
        label = '当前导航动作' if matched and wall_ages.get('navigation_action',math.inf)<=2. else '最近导航动作记录'
        row(label,name+' · ID '+str(action.get('goal_id',''))[:8])
    row('剩余路径', number(a.get('distance_remaining'), ' m') if active else '—（无新鲜活动任务）')
    row('累计到点', str(a.get('completed_goals','—'))+' · 执行者 '+str(a.get('owner') or '无'))
    row('局部控制', controller_id+' · '+plugin+('（按当前到点策略配置）' if active else '（默认配置，非活动证明）'))
    row('全局规划', planner_id+' · '+planner)
    rotating = snapshot.get('rotating')
    if active and snapshot.get('rotating_age',math.inf) <= 1.:
        row('朝向对齐', '控制器报告旋转中' if rotating else '未报告旋转中')
    goal = snapshot.get('goal')
    goal_text = f'{goal[0]} · x={goal[1]:.2f}, y={goal[2]:.2f}, yaw={goal[3]:.2f} rad' if goal else '暂无'
    row('最近派发目标', goal_text)
    outcomes = {'RUNNING':'动作执行中', 'ACTION_FAILED':'动作失败',
                'UNVERIFIED_NO_ODOMETRY':'无足够里程计证据', 'UNVERIFIED_NO_MOTION':'未验证有效位移',
                'AWAITING_NAVIGATION':'已验证位移，尚未证明导航恢复', 'NAVIGATION_RESTORED':'已验证导航恢复'}
    kinds = {'spin':'旋转', 'backup':'后退', 'drive_on_heading':'前进', 'escape_occupied':'重叠脱困'}
    recovery_text = '暂无'
    if recovery:
        recovery_text = (kinds.get(recovery.get('kind'),str(recovery.get('kind','—')))+' · '+
                         outcomes.get(recovery.get('outcome'),str(recovery.get('outcome','未知'))))
    row('最近恢复证据', recovery_text)
    row('指令 → 实测', '线速度 / 角速度')
    for key, label in [('cmd_vel_nav','控制器输出'),('cmd_vel_smoothed','平滑后'),('cmd_vel','最终底盘指令'),('odom','里程计实测')]:
        row(label, velocity(key))
    row('雷达',sensor('scan'))
    row('深度融合',depth_text)
    row('局部代价',sensor('local_costmap')+' · '+str(settings.get('local_frame') or '未知坐标系'))
    targets = snapshot.get('targets') or {}
    if isinstance(targets.get('targets'),list):
        items = targets['targets']
        row('目标定位',str(len(items))+' 个已定位 · '+str(sum(bool(t.get('confirmed')) for t in items))+' 个多帧确认')
    warning = snapshot.get('last_warning')
    if warning:
        row('最近导航警告',str(warning['node'])+' · '+number(snapshot.get('wall_time',0.)-warning['received'],'墙钟秒前')+' · '+warning['text'][:160])
    row('仿真倍率',number(snapshot.get('rtf'),'x')+' · 数值不是导航成功判据')
    warnings_html = ''.join('<p style="color:#a45a00">'+escape(warning)+'</p>' for warning in warnings)
    body = f'<h3 style="color:{color}">{escape(heading)}</h3>'+warnings_html+'<table width="100%" cellspacing="4">'+''.join(rows)+'</table>'
    remaining = number(a.get('distance_remaining'), ' m') if active else '—'
    summary = (f'<p><b style="color:{color}">{escape(heading)}</b> · 剩余 {remaining} · '
               f'到点 {escape(a.get("completed_goals","—"))}<br/>'
               f'目标：{escape(goal_text)}<br/>恢复：{escape(recovery_text)}</p>')
    if warnings:
        summary += '<p style="color:#a45a00">'+escape(warnings[0])+(
            f'（另有 {len(warnings)-1} 条，见详情）' if len(warnings) > 1 else '')+'</p>'
    details = '<p>年龄和Hz按仿真时间；断流另用墙钟检查。路径存在不代表仍有效；没有警报不代表无碰撞。</p>'
    details += '<table width="100%"><tr><th>数据</th><th>年龄(s)</th><th>实收Hz</th></tr>'
    labels = dict(map='地图',global_costmap='全局代价',local_costmap='局部代价',scan='雷达',depth_cloud='深度输入',
                  odom='里程计',global_plan='全局路径',local_plan='局部轨迹',global_footprint='全局轮廓',local_footprint='局部轮廓')
    for key, label in labels.items():
        details += '<tr><td>'+label+'</td><td>'+number(ages.get(key))+'</td><td>'+number(snapshot.get('rates',{}).get(key),digits=1)+'</td></tr>'
    details += '</table><p>同刻TF问题：'+escape('; '.join(snapshot.get('problems',[])) or '本次查询未报错')+'</p>'
    map_text = '静态地图（不要求持续更新）' if snapshot.get('static_map') else '在线 SLAM 地图'
    details += '<p>'+map_text+' · 相对初始锚点修正：'+number(snapshot.get('correction_m'), ' m')+'；这是坐标修正量，不是定位误差。</p>'
    legend_rows = [('#50ff50','全局路径','绿色线：全局路线；可能保留上次结果'),
        ('#ff50aa','局部轨迹 / RPP碰撞弧','粉色线：DWB/MPPI短时轨迹；RPP为碰撞检查弧'),
        ('#ffaa00','派发目标 / RPP前视点','橙色箭头是最近派发目标；黄色圆点是RPP跟踪点'),
        ('#ff5050','雷达','红色点：扫描平面上的回波'),
        ('#00dcff','深度输入','青色点：含地面清障回波，不全是障碍'),
        ('#ffdc00','局部车身包络','黄色实线：同刻TF投影的局部碰撞轮廓'),
        ('#00ffff','全局车身包络（默认隐藏）','青色虚线：全局规划包络，可独立开启'),
        ('#999999','障碍与软代价','Local live obstacle costs是融合结果；膨胀色带不是实体墙'),
        ('#50aa50','前沿与目标','探索候选点不是障碍；ArUco文字ID标注已定位目标'),
        ('#b020b0','SLAM修正箭头','相对初始里程计锚点的坐标修正，不是真值轨迹')]
    legend = '<table cellspacing="3">'+''.join('<tr><td style="color:'+color+'"><b>'+escape(name)+
        '</b></td></tr><tr><td>'+escape(meaning)+'</td></tr>' for color,name,meaning in legend_rows)+'</table>'
    return dict(body=body,summary=summary,details=details,legend=legend,heading=heading,warnings=warnings)
