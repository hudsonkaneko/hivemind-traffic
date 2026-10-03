"""Generate capacity tables and arithmetic projections from immutable probe results."""
import argparse
import ctypes
import json
import platform
import statistics
from pathlib import Path

from experiments.probe_support import ROOT, write_json
from traffic.capacity_model import workload


def hardware():
    result={'cpu':platform.processor()}
    if platform.system()=='Windows':
        import winreg
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE,r'HARDWARE\DESCRIPTION\System\CentralProcessor\0') as key:
            result['cpu']=winreg.QueryValueEx(key,'ProcessorNameString')[0].strip()
        class Memory(ctypes.Structure):
            _fields_=[('length',ctypes.c_ulong),('load',ctypes.c_ulong)]+[(name,ctypes.c_ulonglong) for name in ('total','available','total_page','available_page','total_virtual','available_virtual','extended')]
        memory=Memory();memory.length=ctypes.sizeof(memory)
        if ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(memory)):
            result['os_physical_memory_GiB']=memory.total/2**30
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--traffic',type=Path,required=True)
    parser.add_argument('--rtx',type=Path,required=True)
    parser.add_argument('--refresh',type=Path)
    parser.add_argument('--rtx-smoke',type=Path)
    args=parser.parse_args()
    for name in ('traffic', 'rtx', 'refresh', 'rtx_smoke'):
        value = getattr(args, name)
        if value is not None:
            setattr(args, name, value.resolve())
    traffic=json.loads((args.traffic/'cases.json').read_text())
    rtx=json.loads((args.rtx/'cases.json').read_text())
    refresh=json.loads((args.refresh/'cases.json').read_text()) if args.refresh else []
    smoke=json.loads((args.rtx_smoke/'cases.json').read_text()) if args.rtx_smoke else []
    groups=[]
    for n in sorted({r['vehicles'] for r in traffic}):
        for mode,hz in [('sumo_native',2),('scripted_speed',2),('scripted_speed',10)]:
            rows=[r for r in traffic if (r['vehicles'],r['mode'],r['control_hz'])==(n,mode,hz)]
            if not rows:continue
            group=dict(vehicles=n,mode=mode,control_hz=hz,n=len(rows),passed=sum(r['passed'] for r in rows),
                mean_p95_step_ms=statistics.mean(r['p95_step_ms'] for r in rows),
                min_p95_step_ms=min(r['p95_step_ms'] for r in rows),max_p95_step_ms=max(r['p95_step_ms'] for r in rows),
                mean_step_ms=statistics.mean(r['wall_seconds']/r['steps']*1000 for r in rows),
                mean_real_time_factor=statistics.mean(r['real_time_factor'] for r in rows),
                mean_cpu_cores_at_real_time=statistics.mean(r['cpu_seconds']/r['sim_seconds'] for r in rows),
                max_sampled_process_RSS_MiB=max(sum(v['rss_bytes'] for v in r['resources_after'].values())/2**20 for r in rows))
            groups.append(group)
    model={str(n):{'all_lidar':workload(vehicles=n,lidar_vehicles=n,av_vehicles=n),
                  'two_lidar':workload(vehicles=n,lidar_vehicles=min(n,2),av_vehicles=n),
                  'eight_neighbors':workload(vehicles=n,lidar_vehicles=min(n,2),av_vehicles=n,max_neighbors=8)}
           for n in (2,20,50,100)}
    first_gpu=json.loads((ROOT/rtx[0]['path']/'manifest.json').read_text())['gpu_before']
    result=dict(hardware={**hardware(),'gpu':first_gpu},traffic_groups=groups,rtx_cases=rtx,rtx_smoke_cases=smoke,refresh_cases=refresh,
        arithmetic_projections=model,traffic_suite=str(args.traffic.relative_to(ROOT)),rtx_suite=str(args.rtx.relative_to(ROOT)),
        traffic_repeat=json.loads((args.traffic/'summary.json').read_text()))
    write_json(ROOT/'documentation/capacity-results.json',result)
    lines=['# Workstation capacity and roadmap foundations', '', 'Generated from recorded probes, October 3, 2026.', '',
        f"Hardware: {result['hardware']['cpu']}; OS-reported {result['hardware'].get('os_physical_memory_GiB',0):.1f} GiB RAM; {first_gpu['name']}, {first_gpu['total_mib']:.0f} MiB VRAM, driver {first_gpu['driver']}.", '',
        '## Measured SUMO workload', '',
        'Three predeclared seeds per row. These are sparse straight-road workloads, not proof of coordination benefits. AV tags select deterministic roles; only native driving and independent scripted speed commands are implemented. No LiDAR, learning or V2V runs in this table.', '',
        '| Cars | Control | Hz | Runs passed | Mean step ms | Mean p95 ms (seed range) | CPU cores needed at real time* | Sampled combined RAM MiB |',
        '| ---: | --- | ---: | --- | ---: | --- | ---: | ---: |']
    for r in groups:
        lines.append(f"| {r['vehicles']} | {r['mode']} | {r['control_hz']} | {r['passed']}/{r['n']} | {r['mean_step_ms']:.3f} | {r['mean_p95_step_ms']:.3f} ({r['min_p95_step_ms']:.3f}–{r['max_p95_step_ms']:.3f}) | {r['mean_cpu_cores_at_real_time']:.3f} | {r['max_sampled_process_RSS_MiB']:.1f} |")
    lines += ['', '*CPU-seconds (Python + SUMO) divided by simulated seconds: a real-time arithmetic estimate, not an observed whole-PC utilization percentage. Windows CPU counters are coarse for short runs. Timings include subscriptions, state validation, serialization and trajectory hashing. Mean p95 is the average of per-run percentiles, not a pooled percentile or confidence interval.', '',
        f"All {len(traffic)} cases retained; same-seed repeat matched: {result['traffic_repeat']['repeat_matches']}. No failed runs were excluded. Startup/network generation/warmup and final artifact writes are outside steady-loop timing.", '',
        '## Measured RTX workload', '',
        'Unpaced headless simple boxes, 30 simulated render steps/s, 32-emitter LiDARs at 7200 firing patterns/s. Viewport updates disabled. CPU readback copies included; perception, SUMO, GUI, dynamics and raw-scan disk recording excluded. One run per case plus a repeated 2-car/2-sensor case: exploratory, not a robust capacity limit.', '',
        '| Cars | LiDARs | Sensor gate | Median frame ms | p95 frame ms | Worst frame ms | Sampled whole-GPU peak MiB |',
        '| ---: | ---: | --- | ---: | ---: | ---: | ---: |']
    for r in rtx:
        if 'p95_frame_ms' in r:
            lines.append(f"| {r['cars']} | {r['sensors']} | {r['passed']} | {r['p50_frame_ms']:.2f} | {r['p95_frame_ms']:.2f} | {r['max_frame_ms']:.2f} | {r['gpu_peak_mib']:.0f} |")
        else:lines.append(f"| failed | — | {r.get('reason',r.get('error'))} | — | — | — | — |")
    lines += ['', 'The zero-sensor row is a scene-update baseline with no active camera viewport, NOT a visual-render benchmark. GPU telemetry is sampled about once per second across startup and measurement and includes desktop applications; peaks may be missed. Sensor callback counts and errors are retained per run. A passing sensor gate does not mean every frame met 33.3 ms.', '',
        f"The separate exploratory smoke peak was {max((r['gpu_peak_mib'] for r in smoke),default=0):,.0f} MiB global GPU memory; suite peaks ranged {min(r['gpu_peak_mib'] for r in rtx):,.0f}–{max(r['gpu_peak_mib'] for r in rtx):,.0f} MiB. Cache/desktop/startup effects prevent treating these differences as exact per-car memory. Reserve headroom; do not extrapolate a maximum fleet from these short samples.", '',
        '## Arithmetic: what additional cars actually multiply', '',
        'Nominal single-return sample opportunities per LiDAR-second = 32 × 7200 = 230,400. This is configured workload, not measured valid hits. Full-return recording projection assumes 20 bytes/sample, no compression, headers or duplicate scans. Actual archive size depends on hit rate, fields and capture behavior.', '',
        '| Cars | All cars sensed: nominal samples/s | Two sensed cars: nominal samples/s | All-sensed payload GB/hour | All-to-all receiver deliveries/s | Eight-neighbor deliveries/s |',
        '| ---: | ---: | ---: | ---: | ---: | ---: |']
    for n,r in model.items():
        a,b,c=r['all_lidar'],r['two_lidar'],r['eight_neighbors']
        lines.append(f"| {n} | {a['nominal_lidar_sample_opportunities_per_second']:,.0f} | {b['nominal_lidar_sample_opportunities_per_second']:,.0f} | {a['full_return_payload_GB_per_hour']:.2f} | {a['logical_receiver_deliveries_per_second']:,.0f} | {c['logical_receiver_deliveries_per_second']:,.0f} |")
    lines += ['', 'Communication calculation assumes every car is an AV publishing at 10 Hz: A(A−1)f logical deliveries, or A·min(8,A−1)f with eight neighbors. This is receiver-processing work, NOT physical radio broadcast bandwidth. Neighbor limiting is a recommendation, not implemented in the existing all-to-all V2V bus. Its 4096-pending-delivery bound is another reason not to attach a large fleet unchanged.', '',
        '## Recommended operating tiers', '',
        '1. **Sensor-critical AVs:** initially two cars retain full LiDAR, 10 Hz control, tracking and explicit V2V. Preserve safety/freshness gates.',
        '2. **Background cars:** SUMO native car-following/lane-changing at the same simulation timestep; no individual RTX sensor or Python controller required. They remain visible geometry and obstacles to nearby AV sensors.',
        '3. **Optional lightweight AVs:** lower-frequency scripted or policy decisions with SUMO maintaining motion between decisions. The probe implements 2 Hz scripted speed commands, not a new autonomous-driving policy. Declare this fidelity difference in results.', '',
        'First integrated target: two fully sensed cars plus 18 native background cars; then test 50 total. These are recommended next validation targets, not a claim that the existing two-car controller already controls an arbitrary fleet. Four sensors already approach the 30 Hz frame budget before controller/recording overhead. Benchmark 4+ sensors explicitly before promoting them.', '',
        'Further mitigations: instance repeated assets; bound nearby communication; keep one displayed camera; disable point-cloud drawing for statistics runs; use documented lower-detail geometry without hiding sensor-relevant obstacles; log scalar metrics routinely and retain raw scans only for bounded validation or explicit sampling. Do not lower sensor fidelity silently or let the camera position change the underlying driving experiment.', '',
        'At 100 AVs, reducing decision rate from 10 Hz to 2 Hz cuts speed-command count from 1000/s to 200/s (80%). It does not reduce SUMO physics cadence, LiDAR work, or the size of individual command batches; p95 may improve much less than average cost. Scene complexity and perception algorithms may become the next bottlenecks.', '',
        '## Startup refresh validation', '']
    for r in refresh:
        s=r.get('summary',{});a=r.get('audit',{})
        lines.append(f"- {r['seconds']} s: passed={r['passed']}; integrity={a.get('integrity_passed')}; first freshness={r.get('first_fresh')}; late >100 ms={s.get('deadline_misses_over_100ms')}; behavior={s.get('behavior_passed')}; `{r.get('path')}`.")
    lines += ['', 'The implementation refreshes stationary scans AFTER expensive GC preparation, then anchors sensor time. It does not advance SUMO during refresh or relax stale-sensor limits. It is not proof of exact RTX repeatability or a fix for GUI memory growth.', '',
        '## Reproduce and evidence', '', 'Run from the canonical project root with the traffic environment:', '', '```text',
        'python -m pytest -q', 'python -m experiments.benchmark_traffic --suite', 'python -m experiments.benchmark_rtx',
        'python -m experiments.verify_cooperative_refresh --long', '```', '',
        f"Traffic suite: `{result['traffic_suite']}`. RTX suite: `{result['rtx_suite']}`. Full generated values and run references: [capacity-results.json](capacity-results.json). Each run preserves resolved configuration, source copies/hashes, Git state, metrics, and failures.", '',
        'Original failed CPU smoke (incomplete insertion at 12 m spacing) is retained under `outputs/traffic_scaling/20261003T082342Z-33b8ef3e`. The final sparse fixture uses 25 m spacing and checks that the requested population actually departed. It is not a congestion study.', '',
        'Guidance sources: [SUMO subscriptions/libsumo](https://sumo.dlr.de/docs/TraCI/Interfacing_TraCI_from_Python.html) and [NVIDIA performance handbook](https://docs.isaacsim.omniverse.nvidia.com/latest/reference_material/sim_performance_optimization_handbook.html). Skill guidance informed provenance, matched seeds, sensor/traffic separation, and shared state conventions.']
    (ROOT/'documentation/capacity-results.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')


if __name__=='__main__':main()
