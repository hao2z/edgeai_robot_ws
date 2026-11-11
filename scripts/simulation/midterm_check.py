#!/usr/bin/env python3
"""Run the two independent midterm parts using the existing mission/task trees."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]


def read_json(path, default=None):
    try:
        return json.loads(path.read_text())
    except (FileNotFoundError, json.JSONDecodeError):
        return default


def events(directory):
    path = directory/'mission-events.jsonl'
    if not path.exists():
        return []
    rows = []
    for line in path.read_text().splitlines():
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            pass  # The writer may still be appending the final line.
    return rows


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--duration', type=int, default=780, help='Shared wall budget, including Part2 startup/switching')
    p.add_argument('--verification-seconds', type=int, default=120)
    p.add_argument('--domain', type=int, default=86)
    p.add_argument('--controller', choices=('rpp', 'dwb', 'mppi'), default='rpp')
    p.add_argument('--planner', choices=('lattice', 'navfn'), default='lattice')
    p.add_argument('--depth-obstacles', choices=('config', 'true', 'false'), default='config')
    p.add_argument('--part', choices=('all', 'part2'), default='all')
    p.add_argument('--check', action='store_true')
    a = p.parse_args()
    if not 30 <= a.duration <= 3600 or not 0 <= a.domain <= 232 or not 0 <= a.verification_seconds <= 120:
        p.error('Budget 30..3600, domain 0..232, verification 0..120 seconds required')
    # Check the actual worker too: dependency discovery alone does not catch
    # damaged shell syntax or CRLF line endings in the Part1 runner.
    subprocess.run(['bash', '-n', str(ROOT/'scripts/simulation/run-open-source-exploration.sh')], check=True)
    command = ['bash', str(ROOT/'scripts/simulation/start-autonomous-exploration.sh'),
               '--duration', str(a.duration), '--domain', str(a.domain),
               '--controller', a.controller, '--planner', a.planner, '--arena', 'midterm',
               '--depth-obstacles', a.depth_obstacles]
    subprocess.run(command+['--check'], check=True, cwd=ROOT)
    if a.check:
        print(f'{a.part}: Part2 uses AMCL and S1 → C1 task trees. No simulation started.', flush=True)
        return 0
    subprocess.run(['python3', str(ROOT/'scripts/simulation/stop-test-sessions.py')], check=True)
    subprocess.run(['bash', str(ROOT/'scripts/simulation/stop-simulation.sh')], check=True)
    directory = Path(os.environ.get('EDGEAI_MIDTERM_SESSION_DIR') or
        ROOT/'tests/results/runs'/datetime.now().strftime(f'%Y%m%d-%H%M%S-midterm-domain{a.domain}-%f'))
    directory.mkdir(parents=True,exist_ok=True)
    (ROOT/'runtime/latest-midterm-session.txt').write_text(str(directory)+'\n')
    registration = ROOT/f'runtime/midterm-{a.domain}.json'
    registration.write_text(json.dumps(dict(pid=os.getpid(), domain=a.domain, directory=str(directory),
        start_ticks=Path('/proc/self/stat').read_text().split()[21])))
    child = None
    started = None
    completed = None
    stopped = False
    result = dict(success=False, parts=a.part, budget_wall_s=a.duration, directory=str(directory),
                  collision_status='not instrumented; inspect simulator/contact evidence')

    def stop(*_):
        nonlocal stopped
        stopped = True  # Never advance to Part2 after Stop-Simulation/Ctrl+C.

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)

    def run(command, part, env):
        nonlocal child, started, completed
        if stopped:
            raise RuntimeError('Demonstration stopped')
        print(f'[{part}] 日志：{directory/part}', flush=True)
        child = subprocess.Popen(command, cwd=ROOT, env=env, start_new_session=True)
        startup_end = time.monotonic()+180.
        seen = 0
        while True:
            rows = events(directory/part)
            for row in rows[seen:]:
                if part == 'part1' and row.get('phase') == 'EXPLORING' and started is None:
                    started = row['wall_monotonic_s']
                    (directory/'timer.json').write_text(json.dumps(dict(start_wall_monotonic_s=started,
                        budget_wall_s=a.duration, includes='Part1, switching, Part2 initialization and rescue'), indent=2))
                    print(f'共享计时开始：{a.duration} 秒', flush=True)
                if part == 'part2':
                    if row.get('phase') == 'FINISHED' and row.get('success'):
                        completed = row['completed_wall_monotonic_s']
            seen = len(rows)
            now = time.monotonic()
            if stopped:
                raise RuntimeError('Demonstration stopped')
            if started is None and now > startup_end:
                raise RuntimeError('Part1 startup exceeded 180 seconds')
            if started is not None and (completed if completed is not None else now)-started > a.duration:
                raise RuntimeError('Shared wall budget exhausted')
            code = child.poll()
            if code is not None:
                if code != 0:
                    failure=read_json(directory/part/'startup-failure.json',{})
                    if not failure:failure=read_json(directory/part/'mission-result.json',{})
                    raise RuntimeError(f'{part} exited with code {code}: '+failure.get('reason','inspect session logs'))
                saved = read_json(directory/part/'mission-result.json', {})
                if not saved.get('success'):
                    raise RuntimeError(f'{part}: no confirmed successful mission result')
                child = None
                return saved
            time.sleep(.2)

    try:
        # Prepared independently BEFORE Part1; never loaded into its SLAM session.
        sys.path[:0] = [str(ROOT/'src/edgeai_simulation/launch'), str(ROOT/'src/edgeai_simulation/tools')]
        from simulation_support import make_world
        from generate_navigation_truth_map import generate
        reference = directory/'reference'; reference.mkdir()
        share = ROOT/'src/edgeai_simulation'
        world = reference/'arena.sdf'
        make_world(share, share/'config/targets_midterm.yaml', world, share/'worlds/midterm.sdf')
        generate(world, reference/'map')
        env = dict(os.environ, EDGEAI_COOPERATIVE_RESCUE='true', EDGEAI_MIDTERM_PART='part1',
            EDGEAI_SESSION_DIR=str(directory/'part1'), EDGEAI_MANAGED_SESSION='false',
            EDGEAI_CAPTURE_ACK=os.environ.get('EDGEAI_CAPTURE_ACK','false'), EDGEAI_NAVIGATION_TEST='false', EDGEAI_TURN_TEST='false',
            EDGEAI_RECOVERY_TEST='false', EDGEAI_SCAN360='false', EDGEAI_EXPLORER='frontier',
            EDGEAI_RESCUE='true', EDGEAI_RESCUE_CONFIG=str(ROOT/'src/edgeai_navigation/config/midterm_single_rescue.yaml'))
        if a.part == 'all':
            print(f'中期检查：Part1 建图及寻找两目标 → 重启 → Part2 S1/C1。结果：{directory}', flush=True)
            run(command+['--map', str(directory/'part1/map')], 'part1', env)
            registry = read_json(directory/'part1/targets.json', {})
            ids = {t['id'] for t in registry.get('targets', []) if t.get('yaw') is not None}
            if not {1, 2} <= ids or not (directory/'part1/map.yaml').exists() or not (directory/'part1/map.pgm').exists():
                raise RuntimeError('Part1 missing saved map or localized S1/C1')
            if started is None:
                raise RuntimeError('Part1 missing shared timer start; rebuild edgeai_mission')
            params = directory/'part1/nav2-runtime.yaml'
        else:
            params = reference/'nav2-runtime.yaml'
            config_env = dict(env, EDGEAI_CONTROLLER=a.controller, EDGEAI_NAV_PROFILE='rectangle',
                EDGEAI_GLOBAL_PLANNER=a.planner, EDGEAI_DEPTH_OBSTACLES=a.depth_obstacles)
            subprocess.run([sys.executable, str(ROOT/'scripts/ros/prepare_navigation_runtime.py'),
                str(ROOT/'src/edgeai_navigation/config/nav2_exploration_rectangle.yaml'), str(params)],
                env=config_env, check=True)
            started = time.monotonic()
            (directory/'timer.json').write_text(json.dumps(dict(start_wall_monotonic_s=started,
                budget_wall_s=a.duration, includes='Part2 initialization and rescue only'), indent=2))
            print(f'仅运行 Part2：AMCL 定位后执行 S1 → C1。结果：{directory}', flush=True)
        remaining = a.duration-(time.monotonic()-started)
        if remaining <= 0:
            raise RuntimeError('No wall budget left for Part2')
        env.pop('EDGEAI_SESSION_DIR', None)
        env['EDGEAI_MIDTERM_PART'] = 'part2'
        print(f'启动双车起始区 Part2，剩余 {remaining:.1f} 秒。', flush=True)
        run(['bash', str(ROOT/'scripts/simulation/run-cooperative-truth-test.sh'),
             '--midterm-part2', '--directory', str(directory/'part2'),
             '--params', str(params),
             '--prepared-map', str(reference/'map.yaml'), '--domain', str(a.domain),
             '--duration', str(remaining), '--verification-seconds', str(a.verification_seconds)], 'part2', env)
        if completed is None:
            raise RuntimeError('Part2 missing completion timestamp')
        result.update(success=True, reason='Part2 completed' if a.part == 'part2' else 'Both parts completed', elapsed_wall_s=completed-started)
    except (RuntimeError, subprocess.SubprocessError, OSError) as error:
        result['reason'] = str(error)
        print(f'中期检查未完成：{error}', flush=True)
    finally:
        if child is not None and child.poll() is None:
            child.terminate()
            try:
                child.wait(timeout=90.)
            except subprocess.TimeoutExpired:
                result.update(success=False, reason='Child cleanup timed out; use Stop-Simulation')
        registration.unlink(missing_ok=True)
        if started is not None:
            result.setdefault('elapsed_wall_s', (completed or time.monotonic())-started)
        for name, file in (('s1', 'rescue-result.json'), ('c1', 'cooperative-result.json')):
            result[name] = read_json(directory/'part2'/file)
        (directory/'midterm-result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
        evidence = ['# 中期检查核验', '', f'- 完成：{result["success"]}',
            f'- 原因：{result.get("reason", "interrupted")}',
            f'- 共享墙钟：{result.get("elapsed_wall_s", 0):.1f} / {a.duration} 秒',
            '- Part1 地图：part1/map.yaml、map.pgm' if a.part == 'all' else '- 本次仅运行 Part2，未执行 Part1',
            '- 视觉证据：part1/target-observations.jsonl、target-list.csv、targets.json' if a.part == 'all' else '- Part2 使用规定目标坐标，无需重复识别',
            '- Part2 使用独立预建地图 reference/map.yaml 及规定目标坐标；双车重新从起始区出发',
            '- S1/C1 检查及真值距离窗口：part2/rescue-result.json、cooperative-result.json',
            '- Windows分阶段截图：part1/screenshots/、part2/screenshots/；原始日志及双车诊断同普通任务入口',
            '- 物理碰撞：未配置接触记录，需人工核验，不能由任务成功推断无碰撞',
            '- 当前标记高度按项目配置为 0.15 m，演示前确认现场要求', '']
        (directory/'verification.md').write_text('\n'.join(evidence))
        (directory/'exit-code.txt').write_text('0' if result['success'] else '1')
        print(f'核验记录：{directory/"verification.md"}', flush=True)
    return 0 if result['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
