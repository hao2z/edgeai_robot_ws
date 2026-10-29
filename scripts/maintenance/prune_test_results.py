"""Bound completed test history without deleting active sessions or following links."""
import argparse
import json
import os
from pathlib import Path
import shutil
import stat
import time


def is_link(path):
    info = path.lstat()
    return stat.S_ISLNK(info.st_mode) or bool(
        getattr(info, 'st_file_attributes', 0) & getattr(stat, 'FILE_ATTRIBUTE_REPARSE_POINT', 0))


def directory_size(path):
    total = 0
    for base, directories, files in os.walk(path, followlinks=False):
        directories[:] = [name for name in directories
                          if not is_link(Path(base, name))]
        for name in files:
            item = Path(base, name)
            if not is_link(item):
                total += item.lstat().st_size
    return total


def prune(root, *, max_bytes=2 * 1024**3, max_age_days=14, keep_latest=3,
          active_names=(), apply=False, now=None):
    root = Path(root).resolve()
    now = time.time() if now is None else now
    if not root.exists():
        return dict(removed=[], reclaimed_bytes=0, remaining_bytes=0, over_budget=False)
    entries = []
    for path in root.iterdir():
        if is_link(path) or not path.is_dir():
            continue
        # Completion marker is required: interrupted/unregistered runs need review.
        marker = path / 'exit-code.txt'
        entries.append(dict(path=path, size=directory_size(path),
                            completed=marker.exists(),
                            ended=marker.stat().st_mtime if marker.exists() else now))
    entries.sort(key=lambda item: item['ended'], reverse=True)
    protected = set(active_names)
    protected.update(item['path'].name for item in
                     [entry for entry in entries if entry['completed']][:keep_latest])
    remaining = sum(item['size'] for item in entries)
    removed, reclaimed = [], 0
    for item in reversed(entries):
        path = item['path']
        if not item['completed'] or path.name in protected or (path / '.keep').exists():
            continue
        expired = now - item['ended'] > max_age_days * 86400
        if not expired and remaining <= max_bytes:
            continue
        # Validate the final absolute deletion target immediately before mutation.
        if path.resolve().parent != root or is_link(path):
            raise ValueError(f'Unsafe retention target: {path}')
        if apply:
            shutil.rmtree(path)
        removed.append(path.name)
        remaining -= item['size']
        reclaimed += item['size']
    return dict(removed=removed, reclaimed_bytes=reclaimed, remaining_bytes=remaining,
                over_budget=remaining > max_bytes, applied=apply)


def main():
    project = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    active = []
    for registration in (project / 'runtime').glob('exploration-*.json'):
        try:
            active.append(Path(json.loads(registration.read_text())['directory']).name)
        except (ValueError, KeyError, OSError):
            # An unreadable registration makes automatic deletion unsafe.
            print('Storage retention skipped: unreadable active-session registration')
            return
    print(json.dumps(prune(project / 'tests/results/runs', active_names=active,
                           apply=args.apply)))


if __name__ == '__main__':
    main()
