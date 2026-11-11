"""Stop only registered exploration runners after checking Linux PID identity."""
import json
import os
from pathlib import Path
import re
import signal
import time

root = Path(__file__).resolve().parents[2]


def stop_orphan_lock_holders():
    """Recover children left behind when a runner was killed before cleanup."""
    groups = {}
    for process in Path('/proc').glob('[0-9]*'):
        try:
            pid = int(process.name)
            env = dict(item.split(b'=', 1) for item in
                       (process/'environ').read_bytes().split(b'\0') if b'=' in item)
            if env.get(b'PROJECT_ROOT') != os.fsencode(root):
                continue
            domain = env.get(b'ROS_DOMAIN_ID', b'').decode()
            if not domain.isdigit():
                continue
            lock = str(Path.home()/f'.cache/edgeai-autonomous-{domain}.lock')
            if not any(os.readlink(fd) == lock for fd in (process/'fd').iterdir()):
                continue
            group = os.getpgid(pid)
            # Kill only the isolated project launch groups, never a terminal group.
            leader = Path('/proc')/str(group)
            command = (leader/'cmdline').read_bytes()
            if b'ros2\0launch\0edgeai_' not in command:
                continue
            groups[group] = (leader/'stat').read_text().split()[21]
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
    for group, identity in groups.items():
        print(f'Stopping orphan project launch group: {group}', flush=True)
        for sig, delay in ((signal.SIGINT, 2), (signal.SIGTERM, 2), (signal.SIGKILL, 0)):
            try:
                if (Path('/proc')/str(group)/'stat').read_text().split()[21] != identity:
                    break
                os.killpg(group, sig)
            except (FileNotFoundError, ProcessLookupError):
                break
            deadline = time.monotonic() + delay
            while time.monotonic() < deadline:
                time.sleep(.1)

# Stop the parent first: stopping only Part1 could otherwise launch Part2.
registrations = list((root/'runtime').glob('midterm-*.json'))
registrations += list((root/'runtime').glob('exploration-*.json'))
for registration in registrations:
    # Other runtime reports (e.g. midterm-release-checks.json) are not PID records.
    if not re.fullmatch(r'(midterm|exploration)-\d+\.json', registration.name):continue
    if not registration.exists():continue
    try:
        record = json.loads(registration.read_text())
    except (OSError, ValueError) as error:
        raise SystemExit(f'Cannot read session registration {registration.name}: {error}')
    if (not isinstance(record,dict) or type(record.get('pid')) is not int
            or record['pid']<=1 or not str(record.get('start_ticks','')).isdigit()
            or 'domain' not in record):
        raise SystemExit(f'Invalid session registration: {registration.name}; expected pid/start_ticks/domain')
    pid = record['pid']

    def matches():
        try:
            return (Path(f'/proc/{pid}/stat').read_text().split()[21] == record['start_ticks']
                    and any(name in Path(f'/proc/{pid}/cmdline').read_bytes() for name in
                            (b'run-open-source-exploration.sh',b'cooperative_truth_test.py',b'midterm_check.py')))
        except FileNotFoundError:
            return False

    if matches():
        print(f'Stopping previous test: PID={pid}, domain={record["domain"]}', flush=True)
        os.kill(pid, signal.SIGTERM)
        for _ in range(1000 if registration.name.startswith('midterm-') else 350):
            if not matches():
                break
            time.sleep(.1)
        if matches():
            raise SystemExit('Previous test did not shut down; refusing to start a competing test.')
    registration.unlink(missing_ok=True)

stop_orphan_lock_holders()
