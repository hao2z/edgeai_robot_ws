"""Bounded /proc and existing-log sampling; no ROS subscriptions or process control."""
import argparse
import json
import os
from pathlib import Path
import time


def counters(path):
    """Parse /proc stat, whose parenthesized name can contain spaces."""
    text = path.read_text()
    fields = text[text.rindex(')')+2:].split()
    return dict(ticks=int(fields[11])+int(fields[12]), started=int(fields[19]),
                rss_bytes=int(fields[21])*os.sysconf('SC_PAGE_SIZE'))


def snapshot(domain):
    result = {}
    for process in Path('/proc').glob('[0-9]*'):
        try:
            if int(process.name) == os.getpid():
                continue
            env = (process/'environ').read_bytes().split(b'\0')
            if f'ROS_DOMAIN_ID={domain}'.encode() not in env:
                continue
            row = counters(process/'stat')
            row['pid'] = int(process.name)
            row['command'] = (process/'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')[:500]
            row['threads'] = {}
            for thread in (process/'task').iterdir():
                try:
                    item = counters(thread/'stat')
                    item['name'] = (thread/'comm').read_text().strip()
                    row['threads'][thread.name] = item
                except (OSError, ValueError, IndexError):
                    continue
            result[process.name] = row
        except (OSError, ValueError, IndexError):
            continue
    return result


def latest_alignment(directory):
    try:
        with (directory/'alignment.jsonl').open('rb') as stream:
            stream.seek(0, 2)
            stream.seek(max(0, stream.tell()-65536))
            lines = stream.read().splitlines()
        for line in reversed(lines):
            try:
                return json.loads(line)
            except ValueError:
                continue
    except OSError:
        pass
    return {}


def compare(first, second, elapsed):
    frequency = os.sysconf('SC_CLK_TCK')
    rows = []
    for pid, after in second.items():
        before = first.get(pid)
        if not before or before['started'] != after['started']:
            continue
        row = dict(pid=after['pid'], command=after['command'],
                   cpu_percent=100*(after['ticks']-before['ticks'])/frequency/elapsed,
                   rss_mib=after['rss_bytes']/2**20)
        threads = []
        for tid, item in after['threads'].items():
            previous = before['threads'].get(tid)
            if previous and previous['started'] == item['started']:
                threads.append(dict(tid=int(tid), name=item['name'],
                    cpu_percent=100*(item['ticks']-previous['ticks'])/frequency/elapsed))
        row['top_threads'] = sorted(threads, key=lambda t:t['cpu_percent'], reverse=True)[:5]
        rows.append(row)
    return sorted(rows, key=lambda row:row['cpu_percent'], reverse=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=10.)
    parser.add_argument('--session', type=Path)
    parser.add_argument('--domain', type=int)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if not 1 <= args.duration <= 60:
        parser.error('duration must be 1..60 wall seconds')
    root = Path(__file__).resolve().parents[2]
    directory = args.session or Path((root/'runtime/latest-session.txt').read_text().strip())
    session = json.loads((directory/'session.json').read_text())
    domain = args.domain if args.domain is not None else int(session['domain'])
    first = snapshot(domain)
    alignment = latest_alignment(directory)
    started = time.monotonic()
    time.sleep(args.duration)
    second = snapshot(domain)
    elapsed = time.monotonic()-started
    latest = latest_alignment(directory)
    rows = compare(first, second, elapsed)
    sim_start, sim_end = alignment.get('sim_time'), latest.get('sim_time')
    rtf = ((sim_end-sim_start)/elapsed if latest != alignment and sim_start is not None
           and sim_end is not None and sim_end>=sim_start else None)
    result = dict(domain=domain, session=str(directory), sample_wall_s=elapsed,
                  rtf_from_log_window=rtf, latest_diagnostics=latest, processes=rows,
                  note='100% CPU = one core; RTF uses ~1 Hz log endpoints; restarted/new/exited processes excluded')
    output = args.output or directory/'performance.json'
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(f'采样 {elapsed:.1f}s · 域 {domain} · 日志窗口倍率 {rtf:.2f}x' if rtf is not None else '没有足够的倍率记录')
    for row in rows[:10]:
        print(f'{row["pid"]:6}  CPU {row["cpu_percent"]:6.1f}%  RSS {row["rss_mib"]:6.1f} MiB  {row["command"][:115]}')
    print(f'已保存：{output}')


if __name__ == '__main__':
    main()
