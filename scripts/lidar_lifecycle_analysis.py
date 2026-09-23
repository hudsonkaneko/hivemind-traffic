"""Independent interior-face checks for a stationary two-car visibility fixture."""
import numpy as np
try:
    from .lidar_geometry import directions, ray_box_ranges
except ImportError:
    from lidar_geometry import directions, ray_box_ranges


def phases(variant, duration):
    if variant == 'always-visible':
        return [('visible', 0., duration, True)]
    return [('hidden_initial', 0., 1., False), ('visible_first', 1., 2., True),
            ('hidden_again', 2., 3., False), ('visible_again', 3., duration, True)]


def analyze(scan, config, variant):
    start = float(scan['timestamp_ns'])*1e-9-config['sensor_to_usd_offset_s']
    end = start + config['scan_period_s']
    phase = next((p for p in phases(variant,config['frames']/config['fps'])
                  if start >= p[1]+config['transition_guard_s']-1e-8
                  and end <= p[2]-config['transition_guard_s']+1e-8), None)
    valid = np.isfinite(scan['range_m']) & (scan['range_m']>0) & ((scan['flags']&64)!=0)
    roi = (np.abs(scan['azimuth_deg'])<config['roi_half_angle_deg']) & (np.abs(scan['elevation_deg'])<config['roi_half_angle_deg'])
    selected = valid & roi
    rays = directions(scan['azimuth_deg'][selected],scan['elevation_deg'][selected])
    expected = ray_box_ranges(np.tile(config['sensor_mount_m'],(len(rays),1)),rays,
                              np.tile(config['target_center_m'],(len(rays),1)),config['target_size_m'])
    errors = np.abs(scan['range_m'][selected]-expected)
    echoes,counts = np.unique(scan['echo_id'][selected],return_counts=True)
    return dict(phase=phase[0] if phase else 'transition_excluded',
        target_visible=phase[3] if phase else None,timestamp_ns=int(scan['timestamp_ns']),
        hits=int(selected.sum()),identity_matches=int(scan['target_label'][selected].sum()),
        p95_m=float(np.percentile(errors,95)) if errors.size else None,
        max_m=float(errors.max()) if errors.size else None,
        echo_counts={str(int(k)):int(v) for k,v in zip(echoes,counts)})


def summarize(rows, config, variant, errors=()):
    reports = {}
    for name,_,_,visible in phases(variant,config['frames']/config['fps']):
        selected = [r for r in rows if r['phase']==name]
        hits=sum(r['hits'] for r in selected)
        agreement=sum(r['identity_matches'] for r in selected)/max(1,hits)
        coverage=len(selected)>=config['min_scans_per_phase']
        ranges=[r for r in selected if r['hits']]
        geometry=(hits>=config['min_hits_per_visible_phase'] and all(
            r['p95_m']<=config['range_p95_tolerance_m'] and r['max_m']<=config['range_max_tolerance_m'] for r in ranges)) if visible else hits==0
        identity=agreement>=config['min_identity_agreement'] if visible else hits==0
        reports[name]=dict(scans=len(selected),hits=hits,identity_agreement=agreement if visible else None,
            geometry_passed=bool(geometry),identity_passed=bool(identity),coverage_passed=coverage,
            max_range_error_m=max((r['max_m'] for r in ranges),default=None))
    timestamps=np.array([r['timestamp_ns'] for r in rows],dtype=float)*1e-9
    cadence=float(np.max(np.abs(np.diff(timestamps)-config['scan_period_s']))) if len(rows)>1 else None
    timing=cadence is not None and cadence<=1e-7
    return dict(passed=bool(not errors and timing and all(p['geometry_passed'] and p['identity_passed'] and p['coverage_passed'] for p in reports.values())),
        variant=variant,scans=len(rows),phases=reports,timing_passed=timing,max_cadence_error_s=cadence,
        callback_errors=list(errors),limitation='Interior ROI and guarded visibility phases only; not full traffic or sensor validation.')
