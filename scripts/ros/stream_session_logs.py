"""Follow native logs without buffering; preserve raw output and readable events."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import time
import sys
import errno
import os
from collections import Counter, defaultdict
from mission_display import mission_description
from language import language, localize

COOPERATIVE = dict(
    TEST_SETUP='准备独立地图与双车粗略起点', AMCL_ACTIVE='AMCL已激活',
    INITIAL_POSE_SENT='已下发粗略初始位姿', AUTHORITY_ACQUIRED='已取得导航控制权',
    LOCALIZATION_FAILED='双车定位未收敛',
    WAITING_S1='等待 S1 完成', HANDOVER_REQUESTED='准备共享地图交接', HANDOVER='开始共享地图交接',
    NAVIGATION_WAITING='等待导航组件激活', NAVIGATION_READY='双机导航已就绪',
    LOCALIZING='正在校准双机定位', LOCALIZED='双机定位已通过',
    PARKING_SELECTED='已选定双机停车点', PLAN_REJECTED='候选停车点不可达',
    PLAN_BLOCKED_BY_PARKED_PEER='候选路径被对方车身阻挡',
    JOINT_DWELL='开始检查双机救援停留', RESCUE_REPAIR='重新调整双机救援位姿',
    TARGET_UNAVAILABLE='C1 位置暂不可用', TARGET_UPDATED='已更新 C1 位置',
    NAVIGATING='前往救援停车点', NAV_RESULT='救援导航结果', RESULT='C1 救援结果')

ROS_LINE = re.compile(r'\[(?:DEBUG|INFO|WARN|ERROR|FATAL)\]\s+\[[^\]]+\]\s+\[([^\]]+)\]:\s*(.*)')
RECOVERY_KINDS = dict(spin='旋转', backup='后退', drive_on_heading='前进', escape_occupied='重叠脱困')


def truth_level(message):
    if '真值核验：未通过' in message:return 'ERROR'
    if '真值核验：无法确认' in message:return 'WARN'
    if '真值核验：通过' in message:return 'OK'
    return 'INFO'


def robot_for(source, node):
    if node.startswith(('robot2.', '/robot2/')) or node == 'edgeai_navigation_diagnostics_robot2':
        return 'robot2'
    if source in ('nav', 'authority', 'explore') or node == 'edgeai_navigation_diagnostics':
        return 'robot1'
    if source == 'cooperative-nav' and node.startswith(('amcl', 'fleet_scan_filter')):
        return 'robot1'
    return None

RULES = [
    (r'indexed_8bit_image.vert', 'RViz地图着色器链接异常，详见 rviz.log'),
    (r'Startup /map timing samples were unstable', '地图启动采样时序不稳定，探索器使用配置的处理频率，详见 explore.log'),
    (r'Dock database filepath nor dock parameters set', '充电桩未配置（本轮探索不使用）'),
    (r'No (?:goal|progress) checker was specified', '使用已配置的默认到点/进展检查器'),
    (r'TIME CHECK OK', '时间对齐诊断就绪：消息时效及同一时刻TF检查通过'),
    (r'WAIT/STALE', '时间对齐诊断等待数据或发现过期消息，详见 alignment.log'),
    (r'Failed to get result for follow_path', '取消/切换路径时未取得控制器结果，详见 nav.log'),
    (r'Goal succeeded|Reached.*goal|Goal reached', '已到达导航目标'),
    (r'Start occupied', '规划失败：起点被代价地图判为占用；这不等于已发生物理碰撞'),
    (r'Failed to make progress', '导航长时间没有取得进展'),
    (r'Aborting|Goal failed|Failed to create plan', '导航或规划失败，检查后续恢复动作'),
    (r'Attempting.*spin|Running.*spin|Spin behavior', '正在执行旋转动作（观察扫描或导航恢复，结合任务日志判断）'),
    (r'[Aa]ll .*suppressed|[Nn]o .*unsuppressed', '前沿候选均被抑制，探索可能暂时停滞'),
    (r'[Ee]xploration (?:is )?(?:complete|finished)|[Nn]o frontiers found', '探索器报告完成；仍需结合覆盖率检查遗漏区域'),
    (r'[Cc]ollision.*(stop|halt)|[Ss]top.*collision', '碰撞监测器触发停车；需要检查实际间距'),
]


class EventReducer:
    """Keep evidence and counts while bounding repeated live messages."""
    def __init__(self, emit, clock=time.monotonic, selected=None):
        self.writer, self.clock = emit, clock
        self.selected_language = language(selected)
        self.groups = {}
        self.counts = Counter()
        self.level_counts = Counter()
        self.goal = {}
        self.sends = self.changes = 0
        self.managed_sends = self.recovery_attempts = self.recoveries_verified = 0
        self.frontier = False
        self.last_success = {}
        self.alignment_state = {}
        self.robots = defaultdict(Counter)
        self.mission = None
        self.mission_seen = {}
        self.last_feedback = None
        self.last_progress = self.clock()
        self.joint_status = None

    def emit(self, source, level, description, evidence, **fields):
        match = ROS_LINE.search(evidence)
        robot = fields.pop('robot', None) or robot_for(source, match[1] if match else '')
        self.writer(source, level, localize(description, self.selected_language), evidence,
                    **fields, **({'robot': robot} if robot else {}))

    def task_event(self, source, line, message):
        prefix = 'MISSION_' if source == 'mission' else 'COOPERATIVE_'
        if not message.startswith(prefix):
            return False
        name, _, raw = message.partition(': ')
        try:
            data = json.loads(raw)
        except ValueError:
            return False
        if not isinstance(data, dict):
            return False
        phase = name[len(prefix):]
        data.setdefault('phase', phase)
        failure = data.get('success') is False
        warning = phase in ('MAPPING_AREA_INSUFFICIENT', 'TASK_STOP_UNCONFIRMED',
                            'PLAN_REJECTED', 'PLAN_BLOCKED_BY_PARKED_PEER', 'TARGET_UNAVAILABLE')
        level = 'ERROR' if failure else ('WARN' if warning else ('OK' if data.get('success') is True else 'INFO'))
        if source == 'mission':
            description = mission_description(data, self.selected_language)
            if not description:
                return True
            self.mission = None if phase == 'RESULT' else (description, line, data)
            if phase in ('TASK_REQUESTED', 'TASK_FINISHED'):
                self.joint_status = None
                self.last_feedback = None
                self.mission_seen.clear()
            if phase == 'TASK_FEEDBACK':
                key = (data.get('task'), data.get('unit'))
                previous, self.last_feedback = self.last_feedback, key
                if key == previous or self.clock()-self.mission_seen.get(key, -float('inf')) < 15:
                    return True
                self.mission_seen[key] = self.clock()
        else:
            if phase == 'SUCCESS_POSE':
                text=data.get('summary', '救援成功位姿记录')
                self.emit(source, truth_level(text), text, line, event=name, data=data)
                return True
            if phase == 'JOINT_STATUS':
                self.joint_status = (self.clock(), data)
                return True
            description = COOPERATIVE.get(phase)
            if not description:
                return True
            if phase == 'NAV_RESULT':
                status = data.get('status')
                description += '：'+{4:'导航成功', 5:'已取消', 6:'已中止'}.get(status, '状态未知')
                level = 'OK' if status == 4 else 'WARN' if status == 6 else 'INFO'
            if phase == 'RESULT':
                description += '：'+('失败' if failure else '成功')
            if warning:
                self.grouped((source, phase, data.get('robot')), source, level, description, line,
                             event=name, data=data, robot=data.get('robot'))
                return True
        self.emit(source, level, description, line, event=name, data=data, robot=data.get('robot'))
        self.level_counts[level] += 1
        if source == 'mission':
            self.last_progress = self.clock()
        return True

    def grouped(self, key, source, level, description, evidence, **fields):
        self.counts[key] += 1
        self.level_counts[level] += 1
        if key not in self.groups:
            self.groups[key] = [self.clock(), 0, source, level, description, evidence, fields]
            self.emit(source, level, description, evidence, count=1, **fields)
        else:
            self.groups[key][1] += 1
            self.groups[key][5] = evidence
            self.groups[key][6] = fields

    def flush(self, final=False):
        for key, group in self.groups.items():
            when, pending, source, level, description, evidence, fields = group
            if pending and (final or self.clock()-when >= 15):
                self.emit(source, level, f'{description}（新增 {pending} 次，累计 {self.counts[key]} 次）',
                          evidence, count=self.counts[key], added=pending, **fields)
                group[0], group[1] = self.clock(), 0
        if not final and self.mission and self.clock()-self.last_progress >= 20:
            description, line, data = self.mission
            joint = self.joint_status[1] if self.joint_status and self.clock()-self.joint_status[0] < 5 else {}
            # Joint measurements are ordered [robot1, robot2] and valid only during the check.
            if data.get('unit') == 'check_rescue' and data.get('task') == 'c1' and joint:
                distances = joint.get('distances_m', [])
                if len(distances) == 2 and joint.get('valid'):
                    description += f' · 机器人1 {distances[0]:.2f} m / 机器人2 {distances[1]:.2f} m'
                description += f' · 连续满足 {joint.get("dwell_s", 0):.1f} s'
            self.emit('mission', 'INFO', '进度：'+description, line, event='MISSION_PROGRESS', data=data)
            self.last_progress = self.clock()

    def consume(self, source, line, stopping=False):
        match = ROS_LINE.search(line)
        node, message = match.groups() if match else ('', line)
        robot = robot_for(source, node)
        if source in ('mission', 'cooperative') and self.task_event(source, line, message):
            return
        if source=='suspects' and ('SUSPECT_PROPOSED:' in line or 'SUSPECT_RETAINED:' in line):
            self.emit(source,'INFO',line.split(']: ',1)[-1],line)
            return
        if source in ('rescue', 'target-search') and 'RESCUE_' in line:
            self.emit(source, 'ERROR' if ('RESCUE_FAILED:' in line or 'RESCUE_RESULT: 失败' in line) else truth_level(line), line.split(']: ', 1)[-1], line)
            return
        if source == 'observation' and 'OBSERVATION_' in line:
            self.emit(source, 'ERROR' if 'ERROR' in line or 'UNKNOWN' in line else 'INFO',
                      line.split(']: ', 1)[-1], line)
            return
        if source == 'scan360' and 'SCAN360_' in line:
            self.emit(source, 'WARN' if 'INCOMPLETE' in line or 'ERROR' in line else 'INFO',
                      line.split(']: ', 1)[-1], line)
            return
        if source == 'perception' and ('TARGET_DECODED:' in line or 'TARGET_FOUND:' in line or 'TARGET_CONFIRMED:' in line or 'TARGET_SEARCH_READY:' in line or 'TARGET_LOCALIZATION_WAIT:' in line):
            self.emit(source, 'INFO', line.split(']: ', 1)[-1], line)
            return
        if source in ('authority', 'cooperative-nav') and node.endswith('navigation_authority'):
            event = re.match(r'([A-Z_]+): (.*?); task=(\d+)', message)
            if event:
                stats = self.robots[robot]
                if event[1] == 'GOAL_RESERVED':
                    self.managed_sends += 1
                    stats['managed_goal_sends'] += 1
                elif event[1] == 'RECOVERY_STARTED':
                    self.recovery_attempts += 1
                    stats['recovery_attempts'] += 1
                elif event[1] == 'NAVIGATION_RESTORED':
                    self.recoveries_verified += 1
                    stats['recoveries_verified'] += 1
                level = ('OK' if event[1] == 'NAVIGATION_RESTORED' else
                         'WARN' if event[1] in ('STATE_UNKNOWN', 'GOAL_REJECTED', 'MANUAL_GOAL_REJECTED') else 'INFO')
                if event[1] == 'RECOVERY_STARTED':
                    kind = re.search(r"'kind': '([^']+)'", message)
                    label = RECOVERY_KINDS.get(kind[1], kind[1]) if kind else ''
                    self.emit(source, level, '开始恢复'+('：'+label if label else ''), line,
                              event=event[1], task=int(event[3]), recovery_kind=kind[1] if kind else None)
                    return
                if event[1] == 'GOAL_TERMINAL':
                    result = re.search(r"'status': (\d+).*?'error_code': (\d+)", message)
                    if result:
                        status, code = map(int, result.groups())
                        if status == 4:
                            self.arrived(source, robot, line)
                            return
                        level = 'INFO' if status == 5 or stopping else 'WARN'
                        description = '导航已取消' if status == 5 else f'导航未到位（状态 {status}，错误码 {code}）'
                        self.emit(source, level, description, line, event=event[1], task=int(event[3]))
                        return
                self.emit(source, level, event[2], line, event=event[1], task=int(event[3]))
                return
        if source == 'rosout':
            return
        if stopping and re.search(r'Stopping the thread per request|Requested to deactivate server but goal is still executing', line):
            return
        if source == 'alignment' and message.split(';', 1)[0] in ('TIME CHECK OK', 'WAIT/STALE', 'STOPPING', 'STANDBY'):
            state = ('STOPPING' if stopping or message.startswith('STOPPING') else 'STANDBY' if
                     message.startswith('STANDBY') else 'OK' if message.startswith('TIME CHECK OK') else 'WARN')
            if state != self.alignment_state.get(robot) and state not in ('STOPPING', 'STANDBY'):
                self.grouped((source, robot, state), source, state,
                             '时间检查通过' if state == 'OK' else '等待数据或数据过期，详见 alignment.log', line)
            self.alignment_state[robot] = state
            return
        # Candidate rejection logs also contain dispatch=(x,y); they do not
        # send a navigation action and must not inflate goal/retry counts.
        sending = source == 'explore' and re.search(r'Sending (?:updated )?frontier goal', line)
        goal = re.search(r'dispatch=\(([^)]+)\)', line) if sending else None
        if goal:
            self.frontier = True
        if not goal and (source == 'cooperative-nav' and robot == 'robot2' or
                         source == 'nav' and (not self.frontier or self.mission and
                                             self.mission[2].get('task'))):
            goal = re.search(r'Begin navigating .*? to \(([^)]+)\)', line)
        if goal:
            coords = tuple(round(float(v.strip()), 2) for v in goal[1].split(',')[:2])
            self.sends += 1
            self.robots[robot]['goal_sends'] += 1
            self.last_success.pop(robot, None)
            if coords != self.goal.get(robot):
                self.goal[robot], self.changes = coords, self.changes+1
                self.robots[robot]['goal_changes'] += 1
                self.emit('goal', 'INFO', f'目标变更 → ({coords[0]:.2f}, {coords[1]:.2f}) m', line, robot=robot)
            else:
                self.grouped(('repeat_goal', robot, coords), source, 'WARN',
                             f'同一目标重复下发 ({coords[0]:.2f}, {coords[1]:.2f}) m', line)
            return
        for pattern, description in RULES:
            if re.search(pattern, line):
                level = ('OK' if 'TIME CHECK OK' in line or '已到达' in description else
                         'ERROR' if 'Failed to get result' in line or '[ERROR]' in line else
                         'WARN' if '[WARN]' in line and '充电桩' not in description else
                         'WARN' if re.search(r'WAIT/STALE|occupied|Failed|Aborting|suppressed|collision', line, re.I) else 'INFO')
                if stopping and re.search(r'Aborting|Goal failed|Failed to get result', line):
                    level, description = 'INFO', '测试停止阶段：当前导航任务被取消'
                if '默认到点/进展检查器' in description:
                    # Expected Nav2 fallback to the sole configured plugin.
                    return
                if '已到达导航目标' == description:
                    if source in ('nav', 'cooperative-nav') and robot and (not node or 'bt_navigator' in node):
                        self.arrived(source, robot, line)
                    return
                if source == 'cooperative-nav' and '详见 nav.log' in description:
                    description = description.replace('nav.log', 'cooperative-nav.log')
                self.grouped((source, robot, description, stopping), source, level, description, line)
                return
        match = re.search(r'\[(ERROR|FATAL|WARN)\]\s*(.*)', line)
        if match:
            level = 'ERROR' if match[1] == 'FATAL' else match[1]
            text = message if node else match[2]
            key = re.sub(r'[-+]?\d+(?:\.\d+)?', '#', text)
            self.grouped((source, robot, level, key), source, level, f'{text[:180]}（详见 {source}.log）', line)
        elif re.search(r'\[Err\]', line):
            self.grouped((source, 'native_error', line), source, 'ERROR', f'{line[:180]}（详见 {source}.log）', line)
        elif 'Traceback (most recent call last)' in line or 'process has died' in line:
            self.grouped((source, 'crash', stopping), source, 'WARN' if stopping else 'ERROR',
                         f'{"关闭阶段异常" if stopping else "进程异常"}，详见 {source}.log', line)

    def arrived(self, source, robot, line):
        if self.clock()-self.last_success.get(robot, -float('inf')) > 1:
            self.emit(source, 'OK', '已到达导航目标', line, robot=robot)
            self.robots[robot]['arrivals'] += 1
            self.last_success[robot] = self.clock()


def follow(directory):
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    directory = Path(directory)
    names = ('sim', 'nav', 'explore', 'rviz', 'rosout', 'watch', 'alignment', 'authority', 'perception', 'suspects',
             'scan360', 'observation', 'target-search', 'rescue', 'cooperative', 'cooperative-nav', 'mission')
    for name in names:
        (directory / f'{name}.log').touch(exist_ok=True)
    handles = {name: (directory / f'{name}.log').open(encoding='utf-8', errors='replace')
               for name in names}
    pending = dict.fromkeys(names, '')
    captured = set()
    terminal_output = True
    color = not os.environ.get('NO_COLOR') and (sys.stdout.isatty() or os.environ.get('FORCE_COLOR'))
    colors = {'WARN': '\033[33m', 'ERROR': '\033[31m'}
    with (directory / 'console.log').open('a', encoding='utf-8', buffering=1) as console, \
         (directory / 'events.log').open('a', encoding='utf-8', buffering=1) as events, \
         (directory / 'events.jsonl').open('a', encoding='utf-8', buffering=1) as structured:
        def emit(source, level, description, evidence='', **fields):
            nonlocal terminal_output
            description = localize(description)
            stamp = datetime.now().astimezone().isoformat(timespec='milliseconds')
            label = localize({'robot1':'机器人1', 'robot2':'机器人2'}.get(fields.get('robot'), ''))
            scope = f' [{label}]' if label else ''
            text = f'{stamp} [{level}]{scope} [{source}] {description}'
            events.write(text + '\n')
            structured.write(json.dumps(dict(time=stamp, source=source, level=level,
                description=description, evidence=evidence, language=language(), **fields), ensure_ascii=False) + '\n')
            if terminal_output:
                try:
                    # Keep timestamps and message text neutral; highlight severity only.
                    display = f'{stamp[11:19]} [{level}]{scope} {description}'
                    if color and level in colors:
                        display = display.replace(f'[{level}]', f'{colors[level]}[{level}]\033[0m', 1)
                    print(display, flush=True)
                except (BrokenPipeError, OSError):
                    terminal_output = False
        reducer = EventReducer(emit, selected=language())
        reducer.frontier = (directory/'frontier-backend').exists()
        while True:
            for path in (directory / 'screenshots').glob('*.event'):
                if path.name not in captured:
                    try:
                        text = path.read_text(encoding='utf-8')
                    except OSError:
                        # Windows may briefly hold an event while moving it.
                        continue
                    console.write(text + '\n')
                    if '截图失败' in text:
                        emit('capture', 'WARN', '截图失败，详见 screenshots/capture.log', text)
                    captured.add(path.name)
            for name, handle in handles.items():
                try:
                    pending[name] += handle.read()
                except OSError as error:
                    # WSL's Windows drive can temporarily return ENODATA while
                    # another process appends. Retry without losing offsets.
                    if error.errno in (errno.ENODATA, errno.EAGAIN, errno.EINTR):
                        continue
                    raise
                lines = pending[name].split('\n')
                pending[name] = lines.pop()
                for line in lines:
                    stamp = datetime.now().astimezone().isoformat(timespec='milliseconds')
                    raw = f'{stamp} [{name}] {line}'
                    # /rosout remains available separately, without duplicating console.log.
                    if name != 'rosout':
                        console.write(raw + '\n')
                    reducer.consume(name, line, (directory/'stopping').exists())
            reducer.flush()
            if (directory / 'stream.stop').exists():
                reducer.flush(final=True)
                summary = dict(goal_sends=reducer.sends, goal_changes=reducer.changes,
                    managed_goal_sends=reducer.managed_sends,
                    recovery_attempts=reducer.recovery_attempts,
                    recoveries_verified=reducer.recoveries_verified,
                    repeated_goal_sends=reducer.sends-reducer.changes,
                    severity_counts=dict(reducer.level_counts),
                    robots={robot:dict(counts) for robot,counts in reducer.robots.items()},
                    event_counts={str(k):v for k,v in reducer.counts.items()})
                (directory/'log-summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
                emit('summary', 'INFO', f'目标下发 {reducer.managed_sends or reducer.sends} 次，变更 {reducer.changes} 次，'
                     f'同目标重发 {reducer.sends-reducer.changes} 次，错误 {reducer.level_counts["ERROR"]} 次；详见 log-summary.json')
                for handle in handles.values():
                    handle.close()
                return
            time.sleep(.15)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('directory')
    follow(parser.parse_args().directory)
