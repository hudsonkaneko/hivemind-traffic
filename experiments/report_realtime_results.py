"""Generate the local learning report from immutable realtime run packages."""
import hashlib
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from traffic.live_runtime import ROOT


def main():
    records=[]
    for manifest_path in sorted((ROOT/'outputs/live_lidar').glob('*/manifest.json')):
        manifest=json.loads(manifest_path.read_text())
        if not manifest.get('config',{}).get('realtime'): continue
        summary_path=manifest_path.parent/'summary.json'
        summary=json.loads(summary_path.read_text()) if summary_path.exists() else {}
        records.append(dict(run=manifest_path.parent.name,status=manifest['status'],config=manifest['config'],
            error=manifest.get('error'),summary=summary,
            manifest_sha256=hashlib.sha256(manifest_path.read_bytes()).hexdigest()))
    suites=sorted((ROOT/'outputs/live_lidar').glob('realtime-suite-*-4.json'))
    suite=json.loads(suites[-1].read_text()) if suites else None
    if suite and all(r.get('integrity_passed') for r in suite['runs'][:2]):
        traces=[json.loads((ROOT/'outputs/live_lidar'/r['run']/'telemetry.json').read_text()) for r in suite['runs'][:2]]
        differences={k:max(abs(a['ego_after'][k]-b['ego_after'][k]) for a,b in zip(*traces)) for k in ('x','y','speed')}
        suite['repeat_max_differences']=differences
        suite['repeat_tolerance']=.01
        suite['repeat_passed']=len(traces[0])==len(traces[1]) and all(d<=.01 for d in differences.values()) and all(a['phase']==b['phase'] and a['lane_request']==b['lane_request'] for a,b in zip(*traces))
        suite['passed']=suite['passed'] and suite['repeat_passed']
    report=dict(runs=records,validation_suite=suite,
        scope='One car, stationary obstacles, known straight two-lane map; no PPO, communication, or physics steering',
        failed_launches={'20260928T235802Z-avoid-1e3b7a':'Native GUI startup access violation; log realtime-60-gui-01.log; no completed simulation',
            '20260929T000019Z-avoid-151ce8':'Startup stalled; only the matching child PID 9024 was terminated; original manifest intentionally unchanged'},
        caveat='Earlier exploratory runs overlapped a separate user-started baseline viewer. The fixed suite began after it exited. Exploratory failures are retained, not replaced.')
    (ROOT/'documentation/realtime-lidar-results.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    lines=['# Real-time lidar results','',report['scope'],'',
           'Generated from saved run summaries and raw-evidence audits. See [implementation guide](realtime-lidar.md).','',
           '## Fixed validation suite','', '| Case | Behavior | Wall / simulated seconds | RTF | >100 ms late cycles | Alignment p95 max | Full gate |',
           '|---|---|---|---|---|---|---|']
    if suite:
        for r in suite['runs']:
            lines.append(f"| {r['case']} | {r.get('behavior_passed','N/A')} | {r.get('loop_wall_seconds',0):.3f} / {r.get('traffic_seconds',0):.1f} | {r.get('loop_real_time_factor',0):.4f} | {r.get('deadline_misses_over_100ms','N/A')} | {r.get('max_alignment_p95_m',0):.3f} m | {r['passed']} |")
        lines+=['',f"All suite gates passed: **{suite['passed']}**. This is not a guarantee of hard-real-time behavior or crash-free future runs.",
            '',f"Repeat check (1 cm / 0.01 m/s tolerance and identical phases/lane requests): {suite.get('repeat_passed','not available')}; max differences: {suite.get('repeat_max_differences',{})}.",'']
        for r in suite['runs']: lines.append(f"- {r['case']}: `{r.get('run','no completed run')}`; log `{r['log']}`; audit error: {r.get('audit_error','none')}.")
        lines+=['','### Process memory during the bounded loop','', '| Case | RSS at start | RSS at end |','|---|---|---|']
        by_run={r['run']:r['summary'] for r in records}
        for r in suite['runs']:
            s=by_run.get(r.get('run'),{})
            if s.get('rss_start_bytes') is not None and s.get('rss_end_bytes') is not None:
                lines.append(f"| {r['case']} | {s['rss_start_bytes']/2**20:.1f} MiB | {s['rss_end_bytes']/2**20:.1f} MiB |")
        lines+=['','RSS includes the whole Isaac process, not just controller allocations; these bounded measurements do not prove leak-free indefinite operation. The GUI run grew by about 480 MiB, so continuous-service memory behavior remains an explicit follow-up, not a completed claim.']
    lines+=['','## Retained exploratory runs','', '| Run | Status | Error / result |','|---|---|---|']
    suite_ids={r.get('run') for r in suite['runs']} if suite else set()
    for r in records:
        if r['run'] in suite_ids: continue
        s=r['summary'];detail=r['error'] or report['failed_launches'].get(r['run']) or f"behavior={s.get('behavior_passed','short probe')}; RTF={s.get('loop_real_time_factor','N/A')}; deadline misses={s.get('deadline_misses_over_100ms','N/A')}; alignment={s.get('max_alignment_p95_m','N/A')}"
        lines.append(f"| {r['run']} | {r['status']} | {detail} |")
    lines+=['','A retained manifest marked `running` belongs to an interrupted native process that could not finalize it; it is not a claim that the process is still active.','', '## Interpretation','',report['caveat'],
        '', 'The initial suite showed a repeatable full Python garbage-collection pause. A measured generation-2 collection lasted 176 ms and overlapped the late cycle. Deferred GC is scoped to the bounded demo and restored in a finally block; process memory measurements are saved in each run summary. Long-running service memory management is not validated.',
        '', 'Contract v2 corrects the dropout alignment audit: only fresh newly admitted packets are scored against obstacle geometry; rejected scans remain saved. Frozen old ego-body returns must not be treated as newly perceived obstacles when evaluating a stopped sensor.',
        '', 'The native COMPENSATED world-output probe completed avoidance at real-time average speed but failed alignment and deadline gates. NONCOMPENSATED world output and the lower-density demo profile were evaluated explicitly, not treated as identical sensors. Exact output transformation internals are not proven by this fixture.',
        '', 'The GPU runtime had an exploratory startup access violation and a separate startup stall. These are retained limitations, not claimed fixed. No external runtime binaries or driver settings were patched.',
        '', 'Warm-up, application initialization, final evidence hashing, and shutdown are excluded from loop real-time factor. GUI performance is evaluated separately from headless performance. Whole scans are still roughly 100–200 ms old at control time; real-time speed does not mean zero sensor latency.',
        '', 'Reproduce the fixed suite: `python experiments/run_realtime_suite.py`. JSON includes every recorded run configuration and manifest hash. Raw scans remain in ignored `outputs/`, runtime logs in `logs/`; large evidence is not pushed to GitHub.']
    (ROOT/'documentation/realtime-lidar-results.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')


if __name__=='__main__': main()
