"""Plot saved native samples only. No simulator or learned-model execution."""
from pathlib import Path
import csv
import hashlib
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent
source = ROOT / 'merge-speed-traces.csv'
rows = list(csv.DictReader(source.open(newline='', encoding='utf-8')))
styles = {'frozen-v05': ('#b35c00', 'Frozen MAPPO', '-'),
          'v4': ('#136c9e', 'Scripted reference v4', '-'),
          'native': ('#777777', 'Native SUMO (context)', '--')}
plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 11})
fig, axs = plt.subplots(2, 2, figsize=(13, 7.6), constrained_layout=True)
for r, agent in enumerate(('agent_0', 'agent_4')):
    for controller, (color, label, line) in styles.items():
        samples = [x for x in rows if x['controller'] == controller and x['agent_id'] == agent]
        t = [float(x['time_s']) for x in samples]
        axs[r, 0].plot(t, [float(x['speed_m_s']) * 2.2369362921 for x in samples], color=color, label=label, linestyle=line, linewidth=2)
        axs[r, 1].plot(t, [float(x['distance_m']) for x in samples], color=color, label=label, linestyle=line, linewidth=2)
    for c in range(2):
        ax = axs[r, c]
        ax.axvspan(.1, 30., color='#555555', alpha=.045)
        ax.axvline(30, color='#777777', linewidth=.8, linestyle=':')
        ax.grid(alpha=.16)
        ax.spines[['top', 'right']].set_visible(False)
        ax.set_xlim(0, 55)
        ax.set_ylim(bottom=0)
        ax.set_xlabel('Time in saved episode (seconds)')
    axs[r, 0].set_ylabel('Measured speed (mph)')
    axs[r, 1].set_ylabel('Distance traveled (m)')
axs[0, 0].set_title('Main ramp car: slower during the displayed merge')
axs[0, 1].set_title('Main car: v4 has traveled 55 m less at 30 s')
axs[1, 0].set_title('Another controlled car: v4 slows to 3.3 mph')
axs[1, 1].set_title('This car arrives 7.7 s later with v4')
axs[0, 0].legend(frameon=False, fontsize=9, loc='lower right')
fig.suptitle('Why fewer overrides did not mean better traffic\nSame I-110 seed; actual saved samples; shading marks the 30-second replay', fontsize=15)
fig.savefig(ROOT / 'saved-speed-progress-comparison.png', dpi=150)
manifest = dict(training=False, new_simulator_runs=False,
                source=str(source.relative_to(ROOT.parent.parent.parent)),
                source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                figure_sha256=hashlib.sha256((ROOT/'saved-speed-progress-comparison.png').read_bytes()).hexdigest(),
                interpretation='Original native samples; no interpolation or invented arrivals. Native SUMO has automatic lane changes and is context, not a pure actor-only ablation. Episode ceiling 18 m/s, ramp 13.41 m/s. Past visualization covers first30s only.')
(ROOT/'saved-speed-progress-comparison.json').write_text(json.dumps(manifest, indent=2)+'\n', encoding='utf-8')
print(json.dumps(manifest))
