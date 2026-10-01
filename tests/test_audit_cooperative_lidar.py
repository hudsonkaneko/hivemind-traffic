"""Synthetic CPU evidence checks, with rehashed tampering to exercise replay."""
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import shutil
import tempfile

import numpy as np
import pytest

from scripts.audit_cooperative_lidar import ROOT, audit
from traffic.cooperative_control import CooperativeController
from traffic.lidar_avoidance import body_bounds, rectangle_gap
from traffic.lidar_tracking import LidarTracker
from traffic.realtime_lidar import packet_points
from traffic.v2v import IntentPayload, ObservedObstacle, V2VBus


def write_json(path, data):
    path.write_text(json.dumps(data), encoding="utf-8")


def rehash(root):
    manifest = json.loads((root / "manifest.json").read_text())
    manifest["hashes"] = {str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest()
                          for path in root.rglob("*") if path.is_file() and path.name != "manifest.json"}
    write_json(root / "manifest.json", manifest)


@pytest.fixture
def evidence():
    with tempfile.TemporaryDirectory(prefix="audit-test-", dir=ROOT / "outputs") as temporary:
        root = Path(temporary)
        ids = ("ego", "peer")
        source_names = ("traffic/cooperative_control.py", "traffic/lidar_tracking.py", "traffic/v2v.py",
                        "traffic/realtime_lidar.py", "traffic/lidar_avoidance.py", "traffic/lidar_control.py")
        source_hashes = {}
        for name in source_names:
            destination = root / "source" / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(ROOT / name, destination)
            source_hashes[name] = hashlib.sha256(destination.read_bytes()).hexdigest()
        config = dict(seconds=.4, comm="ideal", seed=42, dropout_step=-1, dropout_vehicle="ego")
        transport = dict(mode="ideal", delay_s=.15, jitter_s=.15, drop_probability=.2, ttl_s=.8)
        manifest = dict(run="synthetic-episode", config=config, transport=transport,
                        source_hashes=source_hashes, evaluation_obstacles=[[100, 104, -5.8, -3.8]])
        contract = dict(sensor_origin=10., traffic_origin=.1, control_hz=10, render_hz=30, sensor_height_m=1.,
                        sensor_ids={vid: "/World/Vehicles/" + vid + "/Lidar" for vid in ids})
        write_json(root / "manifest.json", manifest)
        write_json(root / "timing-contract.json", contract)
        bus = V2VBus(manifest["run"], ids, seed=42, **transport)
        trackers = {vid: LidarTracker() for vid in ids}
        controllers = {vid: CooperativeController(vid, index) for index, vid in enumerate(ids)}
        inboxes = {vid: {} for vid in ids}
        poses = {"ego": dict(x=40., y=-4.8, speed=6., angle=90.),
                 "peer": dict(x=34., y=-1.6, speed=6., angle=90.)}
        rows = []
        for step in range(4):
            now = .1 + step * .1
            sensor_now = 10. + step * .1
            observations, decisions = {}, {}
            for vid in ids:
                for message in bus.receive(vid, now):
                    inboxes[vid][message.sender_id] = message
                packet = dict(vehicle_id=vid, sensor_path=contract["sensor_ids"][vid],
                              scan_complete=1,
                              timestamp=round((sensor_now - .1) * 1e9),
                              xyz=np.tile([60., -4.8, .8], (1000, 1)), flags=np.full(1000, 64, dtype=np.uint32),
                              offset=np.linspace(0, 100000000, 1000).astype(np.int64),
                              frame_start=round((sensor_now - 1 / 30) * 1e9), frame_end=round(sensor_now * 1e9),
                              start_position=np.array([poses[vid]["x"], poses[vid]["y"], 1.]),
                              end_position=np.array([poses[vid]["x"], poses[vid]["y"], 1.]))
                np.savez(root / f"{vid}_{step:04d}.npz", **packet)
                points, healthy, age = packet_points(packet, poses[vid], sensor_now, require_scan_complete=True)
                observation_time = .1 + (packet["timestamp"] + int(np.max(packet["offset"]))) * 1e-9 - 10.
                tracks = trackers[vid].update(points, observation_time)
                messages = list(inboxes[vid].values())
                decisions[vid] = controllers[vid].step(poses[vid], tracks, messages, now, fresh=True)
                observations[vid] = dict(scan_id=f"{vid}_{step:04d}", packet_timestamp=packet["timestamp"],
                                         sensor_age=age, receipt_age=.01, healthy=healthy, fresh=True,
                                         is_new=True, pose_error_m=0., pose_valid=True,
                                         tracks=[asdict(track) for track in tracks], messages=[asdict(message) for message in messages])
            for vid in ids:
                decision = decisions[vid]
                obstacle = ObservedObstacle(*decision.observed_obstacle) if decision.observed_obstacle else None
                bus.publish(vid, now, IntentPayload(poses[vid]["x"], poses[vid]["y"], poses[vid]["speed"],
                                                   decision.intended_lane, decision.phase, obstacle))
            after = {vid: {**pose, "x": pose["x"] + .6} for vid, pose in poses.items()}
            swept, offroad = {}, {}
            for vid in ids:
                a, b = body_bounds(poses[vid]), body_bounds(after[vid])
                swept[vid] = (min(a[0], b[0]) - .02, max(a[1], b[1]) + .02,
                              min(a[2], b[2]) - .02, max(a[3], b[3]) + .02)
                offroad[vid] = b[2] < -6.4 or b[3] > 0
            rows.append(dict(step=step, traffic_time=now, sensor_now=sensor_now, ego=poses, ego_after=after,
                             observations=observations, decisions={vid: asdict(decision) for vid, decision in decisions.items()},
                             obstacle_gaps={vid: rectangle_gap(swept[vid], manifest["evaluation_obstacles"][0]) for vid in ids},
                             vehicle_gap=rectangle_gap(swept["ego"], swept["peer"]), offroad=offroad,
                             collisions=[], peer_returns={vid: 0 for vid in ids},
                             speed_interventions={vid: abs(after[vid]["speed"] - decisions[vid].speed) > .15 for vid in ids},
                             deadline_lateness=0.))
            poses = after
        summary = dict(passed=False, behavior_passed=False, safe=True, steps=4, traffic_seconds=.4,
                       loop_wall_seconds=.4, loop_real_time_factor=1., deadline_misses_over_100ms=0,
                       min_vehicle_gap_m=min(row["vehicle_gap"] for row in rows),
                       min_obstacle_gap_m=min(min(row["obstacle_gaps"].values()) for row in rows),
                       speed_interventions=sum(sum(row["speed_interventions"].values()) for row in rows),
                       final_phases={vid: controllers[vid].phase for vid in ids}, communication=bus.telemetry,
                       sensor_counts={vid: 4 for vid in ids}, peer_visibility_passed=False,
                       initial_peer_returns={vid: 0 for vid in ids})
        write_json(root / "telemetry.json", rows)
        write_json(root / "summary.json", summary)
        rehash(root)
        yield root


def mutate_rows(root, change):
    rows = json.loads((root / "telemetry.json").read_text())
    change(rows)
    write_json(root / "telemetry.json", rows)
    rehash(root)


def test_valid_evidence_replays_but_short_run_does_not_pass_behavior(evidence):
    result = audit(evidence)
    assert result.get("integrity_passed"), result
    assert result["passed"] is False


def test_missing_evidence_is_failed_report(evidence):
    (evidence / "summary.json").unlink()
    assert not audit(evidence)["integrity_passed"]


def test_hash_tampering_fails(evidence):
    (evidence / "ego_0000.npz").write_bytes(b"tampered")
    assert "Hash mismatch" in audit(evidence)["audit_error"]


@pytest.mark.parametrize("change,expected", [
    (lambda rows: rows[1].update(step=5), "Noncontiguous"),
    (lambda rows: rows[1].update(traffic_time=2.), "Traffic clock"),
    (lambda rows: rows[1]["decisions"]["ego"].update(speed=99.), "replayed decision"),
    (lambda rows: rows[1]["observations"]["ego"].update(fresh=False), "observation fresh"),
    (lambda rows: rows[1]["observations"]["peer"]["messages"][0]["payload"].update(x=999.), "replayed bus inbox"),
    (lambda rows: rows[1]["obstacle_gaps"].update(ego=999.), "obstacle truth gap"),
    (lambda rows: rows[1]["peer_returns"].update(ego=99), "peer visibility returns"),
])
def test_rehashed_invalid_evidence_still_fails(evidence, change, expected):
    mutate_rows(evidence, change)
    result = audit(evidence)
    assert not result["integrity_passed"]
    assert expected in result["audit_error"], result


def test_rehashed_raw_vehicle_spoof_fails(evidence):
    path = evidence / "ego_0000.npz"
    with np.load(path) as raw:
        packet = {key: raw[key] for key in raw.files}
    packet["vehicle_id"] = "peer"
    np.savez(path, **packet)
    rehash(evidence)
    assert "vehicle identity" in audit(evidence)["audit_error"]


def test_rehashed_incomplete_scan_cannot_claim_healthy(evidence):
    path = evidence / "ego_0000.npz"
    with np.load(path) as raw:
        packet = {key: raw[key] for key in raw.files}
    packet["scan_complete"] = 0
    np.savez(path, **packet)
    rehash(evidence)
    result = audit(evidence)
    assert not result["integrity_passed"]
    assert "observation healthy" in result["audit_error"]
