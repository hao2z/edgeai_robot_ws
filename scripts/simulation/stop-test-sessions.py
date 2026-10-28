"""Stop only registered exploration runners after checking Linux PID identity."""
import json
import os
from pathlib import Path
import signal
import time

root = Path(__file__).resolve().parents[2]
for registration in (root / 'runtime').glob('exploration-*.json'):
    record = json.loads(registration.read_text())
    pid = record['pid']

    def matches():
        try:
            return (Path(f'/proc/{pid}/stat').read_text().split()[21] == record['start_ticks']
                    and b'run-open-source-exploration.sh' in Path(f'/proc/{pid}/cmdline').read_bytes())
        except FileNotFoundError:
            return False

    if matches():
        print(f'Stopping previous test: PID={pid}, domain={record["domain"]}', flush=True)
        os.kill(pid, signal.SIGTERM)
        for _ in range(350):
            if not matches():
                break
            time.sleep(.1)
        if matches():
            raise SystemExit('Previous test did not shut down; refusing to start a competing test.')
    registration.unlink(missing_ok=True)
