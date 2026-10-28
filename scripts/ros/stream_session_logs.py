"""Follow native logs without buffering; preserve raw output and readable events."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import time
import sys

RULES = [
    (r'Begin navigating|Navigating to|Sending.*goal', '导航开始前往新的目标'),
    (r'Goal succeeded|Reached.*goal|Goal reached', '已到达导航目标'),
    (r'Start occupied', '规划失败：起点被代价地图判为占用；这不等于已发生物理碰撞'),
    (r'Failed to make progress', '导航长时间没有取得进展'),
    (r'Aborting|Goal failed|Failed to create plan', '导航或规划失败，检查后续恢复动作'),
    (r'Attempting.*spin|Running.*spin|Spin behavior', '正在执行旋转恢复动作'),
    (r'[Aa]ll .*suppressed|[Nn]o .*unsuppressed', '前沿候选均被抑制，探索可能暂时停滞'),
    (r'[Ee]xploration (?:is )?(?:complete|finished)|[Nn]o frontiers found', '探索器报告完成；仍需结合覆盖率检查遗漏区域'),
    (r'[Cc]ollision.*(stop|halt)|[Ss]top.*collision', '碰撞监测器触发停车；需要检查实际间距'),
]


def follow(directory):
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    directory = Path(directory)
    names = ('sim', 'nav', 'explore', 'rviz', 'rosout', 'watch')
    handles = {name: (directory / f'{name}.log').open(encoding='utf-8', errors='replace')
               for name in names}
    pending = dict.fromkeys(names, '')
    last = {}
    captured = set()
    terminal_output = True
    with (directory / 'console.log').open('a', encoding='utf-8', buffering=1) as console, \
         (directory / 'events.log').open('a', encoding='utf-8', buffering=1) as events, \
         (directory / 'events.jsonl').open('a', encoding='utf-8', buffering=1) as structured:
        while True:
            for path in (directory / 'screenshots').glob('*.event'):
                if path.name not in captured:
                    try:
                        text = path.read_text(encoding='utf-8')
                    except OSError:
                        # Windows may briefly hold an event while moving it.
                        continue
                    events.write(text + '\n')
                    captured.add(path.name)
            for name, handle in handles.items():
                pending[name] += handle.read()
                lines = pending[name].split('\n')
                pending[name] = lines.pop()
                for line in lines:
                    stamp = datetime.now().astimezone().isoformat(timespec='milliseconds')
                    raw = f'{stamp} [{name}] {line}'
                    console.write(raw + '\n')
                    # rosout duplicates node output, but is retained in console.log.
                    if name != 'rosout' and terminal_output:
                        try:
                            print(raw, flush=True)
                        except (BrokenPipeError, OSError):
                            terminal_output = False
                    for pattern, description in RULES if name != 'rosout' else []:
                        if re.search(pattern, line) and time.monotonic() - last.get((name, pattern), -100) > 5:
                            last[name, pattern] = time.monotonic()
                            if (directory / 'stopping').exists() and re.search(r'Aborting|Goal failed', line):
                                description = '测试停止阶段：当前导航任务被取消'
                            events.write(f'{stamp} [{name}] {description}\n')
                            structured.write(json.dumps(dict(time=stamp, source=name,
                                description=description, evidence=line), ensure_ascii=False) + '\n')
            if (directory / 'stream.stop').exists():
                return
            time.sleep(.15)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('directory')
    follow(parser.parse_args().directory)
