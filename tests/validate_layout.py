"""Read-only checks for directory migration and active script/map references."""
import ast
from pathlib import Path
import subprocess
import yaml

root = Path(__file__).resolve().parents[1]
for relative in ('src/edgeai_robot_description', 'scripts', 'algorithms/exploration',
                 'simulation/tools', 'tests/exploration', 'tests/simulation'):
    assert (root/relative).is_dir(), f'Missing workspace directory: {relative}'
for name in ('Start-Simulation.ps1', 'Start-Autonomous-Exploration.ps1', 'start_sim.sh'):
    assert (root/name).is_file(), f'Missing entry point: {name}'
files = list(root.glob('*.ps1'))
for folder in ('scripts','algorithms','simulation','tests/simulation','tests/exploration'):
    files.extend(p for p in (root/folder).rglob('*') if p.is_file())
python_count = shell_count = 0
for p in files:
    if p.suffix not in ('.py','.sh','.ps1','.rc'):
        continue
    text = p.read_text(encoding='utf-8-sig')
    assert 'environment-' + 'check/' not in text, f'Stale path in {p}'
    if p.suffix == '.py':
        ast.parse(text, filename=str(p))
        python_count += 1
    elif p.suffix == '.sh':
        subprocess.run(['bash','-n',str(p)],check=True)
        shell_count += 1
maps = list((root/'tests/results/exploration').glob('*.yaml'))
maps.extend((root/'tests/results/runs').glob('*/map.yaml'))
for p in maps:
    config = yaml.safe_load(p.read_text())
    assert (p.parent/config['image']).is_file(), f'Missing map image: {p}'
print(f'PASS: unified workspace references; {python_count} Python files, {shell_count} shell scripts, {len(maps)} map/image pairs')
