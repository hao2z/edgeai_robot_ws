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
from collections import Counter

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
    def __init__(self, emit, clock=time.monotonic):
        self.emit, self.clock = emit, clock
        self.groups = {}
        self.counts = Counter()
        self.level_counts = Counter()
        self.goal = None
        self.sends = self.changes = 0
        self.managed_sends = self.recovery_attempts = self.recoveries_verified = 0
        self.frontier = False
        self.last_success = -100.0
        self.alignment_state = None

    def grouped(self, key, source, level, description, evidence):
        self.counts[key] += 1
        self.level_counts[level] += 1
        if key not in self.groups:
            self.groups[key] = [self.clock(), 0, source, level, description, evidence]
            self.emit(source, level, description, evidence, count=1)
        else:
            self.groups[key][1] += 1
            self.groups[key][5] = evidence

    def flush(self, final=False):
        for key, group in self.groups.items():
            when, pending, source, level, description, evidence = group
            if pending and (final or self.clock()-when >= 15):
                self.emit(source, level, f'{description}（新增 {pending} 次，累计 {self.counts[key]} 次）',
                          evidence, count=self.counts[key], added=pending)
                group[0], group[1] = self.clock(), 0

    def consume(self, source, line, stopping=False):
        if source == 'observation' and 'OBSERVATION_' in line:
            self.emit(source, 'ERROR' if 'ERROR' in line or 'UNKNOWN' in line else 'INFO',
                      line.split(']: ', 1)[-1], line)
            return
        if source == 'scan360' and 'SCAN360_' in line:
            self.emit(source, 'WARN' if 'INCOMPLETE' in line or 'ERROR' in line else 'INFO',
                      line.split(']: ', 1)[-1], line)
            return
        if source == 'perception' and ('TARGET_FOUND:' in line or 'TARGET_CONFIRMED:' in line or 'TARGET_SEARCH_READY:' in line):
            self.emit(source, 'INFO', line.split(']: ', 1)[-1], line)
            return
        if source == 'authority':
            event = re.search(r'\[navigation_authority\]: ([A-Z_]+): (.*?); task=', line)
            if event:
                if event[1] == 'GOAL_RESERVED':
                    self.managed_sends += 1
                elif event[1] == 'RECOVERY_STARTED':
                    self.recovery_attempts += 1
                elif event[1] == 'NAVIGATION_RESTORED':
                    self.recoveries_verified += 1
                level = ('OK' if event[1] == 'NAVIGATION_RESTORED' else
                         'WARN' if event[1] in ('STATE_UNKNOWN', 'GOAL_REJECTED', 'MANUAL_GOAL_REJECTED') else 'INFO')
                self.emit(source, level, event[1]+': '+event[2], line)
                return
        if source == 'rosout':
            return
        if stopping and re.search(r'Stopping the thread per request|Requested to deactivate server but goal is still executing', line):
            return
        if source == 'alignment' and ('TIME CHECK OK' in line or 'WAIT/STALE' in line or 'STOPPING' in line):
            state = 'STOPPING' if stopping or 'STOPPING' in line else ('OK' if 'TIME CHECK OK' in line else 'WARN')
            if state != self.alignment_state and state != 'STOPPING':
                self.emit(source, state, '时间检查通过' if state == 'OK' else '等待数据或数据过期，详见 alignment.log', line)
            self.alignment_state = state
            return
        # Candidate rejection logs also contain dispatch=(x,y); they do not
        # send a navigation action and must not inflate goal/retry counts.
        sending = source == 'explore' and re.search(r'Sending (?:updated )?frontier goal', line)
        goal = re.search(r'dispatch=\(([^)]+)\)', line) if sending else None
        if goal:
            self.frontier = True
        if not goal and source == 'nav' and not self.frontier:
            goal = re.search(r'Begin navigating .*? to \(([^)]+)\)', line)
        if goal:
            coords = tuple(round(float(v.strip()), 2) for v in goal[1].split(',')[:2])
            self.sends += 1
            if coords != self.goal:
                self.flush(final=True)
                self.goal, self.changes = coords, self.changes+1
                self.emit('goal', 'INFO', f'目标变更 → ({coords[0]:.2f}, {coords[1]:.2f}) m', line)
            else:
                self.grouped(('repeat_goal', coords), 'goal', 'WARN',
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
                    if source == 'nav' and self.clock()-self.last_success > 1:
                        self.emit(source, level, description, line)
                        self.last_success = self.clock()
                    return
                self.grouped((source, description, stopping), source, level, description, line)
                return
        match = re.search(r'\[(ERROR|FATAL|WARN)\].*?\]\s*(.*)', line)
        if match:
            level = 'ERROR' if match[1] == 'FATAL' else match[1]
            text = match[2]
            key = re.sub(r'[-+]?\d+(?:\.\d+)?', '#', text)
            self.grouped((source, level, key), source, level, f'{text[:180]}（详见 {source}.log）', line)
        elif re.search(r'\[Err\]', line):
            self.grouped((source, 'native_error', line), source, 'ERROR', f'{line[:180]}（详见 {source}.log）', line)
        elif 'Traceback (most recent call last)' in line or 'process has died' in line:
            self.grouped((source, 'crash', stopping), source, 'WARN' if stopping else 'ERROR',
                         f'{"关闭阶段异常" if stopping else "进程异常"}，详见 {source}.log', line)


def follow(directory):
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    directory = Path(directory)
    names = ('sim', 'nav', 'explore', 'rviz', 'rosout', 'watch', 'alignment', 'authority', 'perception', 'scan360', 'observation')
    for name in names:
        (directory / f'{name}.log').touch(exist_ok=True)
    handles = {name: (directory / f'{name}.log').open(encoding='utf-8', errors='replace')
               for name in names}
    pending = dict.fromkeys(names, '')
    captured = set()
    terminal_output = True
    color = not os.environ.get('NO_COLOR') and (sys.stdout.isatty() or os.environ.get('FORCE_COLOR'))
    colors = {'INFO': '\033[36m', 'OK': '\033[32m', 'WARN': '\033[33m', 'ERROR': '\033[31m'}
    with (directory / 'console.log').open('a', encoding='utf-8', buffering=1) as console, \
         (directory / 'events.log').open('a', encoding='utf-8', buffering=1) as events, \
         (directory / 'events.jsonl').open('a', encoding='utf-8', buffering=1) as structured:
        def emit(source, level, description, evidence='', **fields):
            nonlocal terminal_output
            stamp = datetime.now().astimezone().isoformat(timespec='milliseconds')
            text = f'{stamp} [{level}] [{source}] {description}'
            events.write(text + '\n')
            structured.write(json.dumps(dict(time=stamp, source=source, level=level,
                description=description, evidence=evidence, **fields), ensure_ascii=False) + '\n')
            if terminal_output:
                try:
                    print(colors[level]+text+'\033[0m' if color else text, flush=True)
                except (BrokenPipeError, OSError):
                    terminal_output = False
        reducer = EventReducer(emit)
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
