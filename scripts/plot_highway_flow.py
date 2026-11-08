"""Publish plots from saved motion only; uses the separate plotting Python."""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--comparison', type=Path, required=True)
    parser.add_argument('--prior-recording', type=Path)
    args = parser.parse_args()
    folder = args.comparison.resolve()
    if not folder.is_relative_to(ROOT/'outputs'):
        raise ValueError('Plots belong under project outputs')
    rows = list(csv.DictReader((folder/'traces.csv').open(newline='', encoding='utf-8')))
    labels = list(dict.fromkeys(r['label'] for r in rows))
    provenance = {}
    if args.prior_recording:
        prior = args.prior_recording.resolve()
        if not prior.is_relative_to(ROOT/'outputs'):
            raise ValueError('Use recorded project evidence')
        data = json.loads(prior.read_text(encoding='utf-8'))
        current = json.loads((folder/'left-5fps/recording.json').read_text())
        if data['network_sha256'] != current['network_sha256']:
            raise ValueError('Historical comparison network differs')
        label = 'Previous reference v4'
        labels.append(label)
        provenance[label] = dict(path=str(prior), sha256=hashlib.sha256(prior.read_bytes()).hexdigest(),
            scope='Historical recorded controller on matching network; configuration remains separately identified')
        for frame in data['frames']:
            for car in frame['vehicles']:
                if car['id'] in ('agent_0', 'agent_4'):
                    rows.append(dict(label=label, agent=car['id'], time_s=frame['time'], speed_m_s=car['speed'],
                        native_distance_m=car['distance_m'], acceleration_m_s2=car['acceleration']))
    plt.rcParams.update({'font.size': 10, 'font.family': 'DejaVu Sans'})
    fig, axes = plt.subplots(2, 3, figsize=(14, 8), constrained_layout=True)
    colors = ('#B45B13', '#186EA6', '#6F7783')
    for i, agent in enumerate(('agent_0', 'agent_4')):
        for label, color in zip(labels, colors):
            series = [r for r in rows if r['agent'] == agent and r['label'] == label]
            for j, (field, title, unit) in enumerate((('speed_m_s','Speed','m/s'),
                ('native_distance_m','Route progress','m'), ('acceleration_m_s2','Measured acceleration','m/s²'))):
                axes[i,j].plot([float(r['time_s']) for r in series], [float(r[field]) for r in series],
                    label=label, color=color, linewidth=1.6, linestyle='--' if label.startswith('Previous') else '-')
                axes[i,j].set(title=f'{agent}: {title}', xlabel='Simulation time (s)', ylabel=unit, xlim=(0,60))
                axes[i,j].grid(alpha=.18)
        axes[i,0].axhline(9, color='#B44444', linewidth=.8, linestyle=':', label='Half the 18 m/s ceiling')
        axes[i,0].set_ylim(bottom=0)
    handles, names = axes[0,0].get_legend_handles_labels()
    fig.legend(handles, names, loc='outside lower center', ncols=4, frameon=False)
    fig.suptitle('I-110: main-car flow still fails; the ending-lane detour is removed', fontsize=15)
    target = folder/'full-route-comparison.png'
    if target.exists():
        raise ValueError('Preserve prior plot output')
    fig.savefig(target, dpi=150)
    (folder/'plot-provenance.json').write_text(json.dumps(dict(training=False, simulation_run=False,
        source_csv_sha256=hashlib.sha256((folder/'traces.csv').read_bytes()).hexdigest(), prior=provenance,
        figure_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),
        unobserved_after_arrival='No invented zero speeds or extrapolated distance'), indent=2)+'\n')
    print(target)


if __name__ == '__main__':
    main()
