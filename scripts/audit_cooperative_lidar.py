"""Independently replay hashed fleet sensor, tracker, controller and V2V evidence."""
import argparse
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from traffic.cooperative_control import CooperativeController
from traffic.lidar_avoidance import body_bounds, rectangle_gap
from traffic.lidar_tracking import LidarTracker
from traffic.realtime_lidar import packet_points
from traffic.v2v import IntentPayload, ObservedObstacle, V2VBus, V2VMessage


def require(condition, message):
    if not condition:
        raise ValueError(message)


def close(actual, expected, context, tolerance=1e-7):
    """Compare nested serialized structures, preserving boolean/type semantics."""
    if isinstance(expected, dict):
        require(isinstance(actual, dict) and set(actual) == set(expected), context + ": keys differ")
        for key in expected:
            close(actual[key], expected[key], context + "." + key, tolerance)
    elif isinstance(expected, (tuple, list)):
        require(isinstance(actual, (tuple, list)) and len(actual) == len(expected), context + ": length differs")
        for index, (a, e) in enumerate(zip(actual, expected)):
            close(a, e, context + f"[{index}]", tolerance)
    elif isinstance(expected, bool) or expected is None or isinstance(expected, str):
        require(type(actual) is type(expected) and actual == expected, context + ": value differs")
    else:
        require(not isinstance(actual, bool) and isinstance(actual, (int, float)) and
                math.isfinite(actual) and math.isfinite(expected) and abs(actual - expected) <= tolerance,
                context + ": numeric value differs")


def decode_message(data):
    payload = dict(data["payload"])
    observed = payload.get("observed_obstacle")
    if observed is not None:
        payload["observed_obstacle"] = ObservedObstacle(**observed)
    return V2VMessage(**{**data, "payload": IntentPayload(**payload)})


def checked_path(root, name):
    require(isinstance(name, str) and not Path(name).is_absolute(), "Evidence path must be relative")
    path = (root / name).resolve()
    require(path.is_relative_to(root.resolve()), "Evidence path escapes run directory")
    return path


def load_json(path):
    def reject(value):
        raise ValueError("Nonfinite JSON value " + value)
    return json.loads(path.read_text(encoding="utf-8"), parse_constant=reject)


def _audit(directory):
    root = Path(directory).resolve()
    manifest = load_json(root / "manifest.json")
    hashes = manifest.get("hashes")
    require(isinstance(hashes, dict) and hashes, "Missing evidence hashes")
    files = {p.relative_to(root).as_posix() for p in root.rglob("*") if p.is_file() and p.name != "manifest.json"}
    require(files == {Path(name).as_posix() for name in hashes}, "Evidence hash inventory differs from recorded files")
    for name, digest in hashes.items():
        require(hashlib.sha256(checked_path(root, name).read_bytes()).hexdigest() == digest,
                "Hash mismatch: " + name)
    sources = manifest.get("source_hashes")
    require(isinstance(sources, dict) and sources, "Missing source hashes")
    for name, digest in sources.items():
        require(hashlib.sha256(checked_path(root, "source/" + name).read_bytes()).hexdigest() == digest,
                "Archived source hash mismatch: " + name)
    for name in ("traffic/cooperative_control.py", "traffic/lidar_tracking.py", "traffic/v2v.py",
                 "traffic/realtime_lidar.py", "traffic/lidar_avoidance.py", "traffic/lidar_control.py"):
        require(name in sources and hashlib.sha256((ROOT / name).read_bytes()).hexdigest() == sources[name],
                "Replay implementation differs from archived source: " + name)
    config = manifest["config"]
    rows = load_json(root / "telemetry.json")
    summary = load_json(root / "summary.json")
    contract = load_json(root / "timing-contract.json")
    require(isinstance(rows, list) and rows and len(rows) == round(config["seconds"] * 10), "Incomplete run")
    if (root / "telemetry.jsonl").exists():
        streamed = [json.loads(line) for line in (root / "telemetry.jsonl").read_text().splitlines()]
        close(streamed, rows, "Telemetry stream")
    ids = ("ego", "peer")
    require(set(contract["sensor_ids"]) == set(ids) and len(set(contract["sensor_ids"].values())) == 2,
            "Independent sensor paths required")
    require(contract["control_hz"] == 10 and contract["render_hz"] == 30, "Unexpected clock rates")
    require(math.isfinite(contract["sensor_height_m"]) and contract["sensor_height_m"] > 0, "Invalid sensor height")
    obstacles = manifest.get("evaluation_obstacles")
    require(isinstance(obstacles, list) and obstacles, "Missing evaluation obstacle bounds")
    for obstacle in obstacles:
        require(len(obstacle) == 4, "Malformed evaluation bounds")
        ObservedObstacle(*obstacle)
    transport = manifest["transport"]
    require(transport["mode"] == config["comm"], "Transport configuration mismatch")
    bus = V2VBus(manifest["run"], ids, seed=config["seed"], **transport)
    trackers = {vid: LidarTracker() for vid in ids}
    controllers = {vid: CooperativeController(vid, index) for index, vid in enumerate(ids)}
    tracks = {vid: [] for vid in ids}
    previous = {vid: -1 for vid in ids}
    inboxes = {vid: {} for vid in ids}
    frozen = {}
    sensor_counts = {vid: 0 for vid in ids}
    min_obstacle, min_vehicle, interventions = math.inf, math.inf, 0
    safe = True
    for index, row in enumerate(rows):
        require(type(row["step"]) is int and row["step"] == index, "Noncontiguous steps")
        now = row["traffic_time"]
        close(now, contract["traffic_origin"] + index * .1, "Traffic clock")
        close(row["sensor_now"], contract["sensor_origin"] + now - contract["traffic_origin"], "Sensor clock", 1e-4)
        if index:
            close(row["ego"], rows[index - 1]["ego_after"], "Vehicle state continuity")
        for field in ("ego", "ego_after", "decisions", "observations"):
            require(set(row[field]) == set(ids), "Missing/distinct vehicle identity in " + field)
        for vid in ids:
            for message in bus.receive(vid, now):
                inboxes[vid][message.sender_id] = message
            inboxes[vid] = {sender: message for sender, message in inboxes[vid].items() if now < message.expires_at}
        decisions, origins, own_points = {}, {}, {}
        for vid in ids:
            observation = row["observations"][vid]
            scan_id = observation["scan_id"]
            require(scan_id == f"{vid}_{index:04d}", "Raw packet file identity mismatch")
            with np.load(checked_path(root, scan_id + ".npz"), allow_pickle=False) as raw:
                packet = {key: raw[key] for key in raw.files}
            require(str(np.asarray(packet["vehicle_id"]).item()) == vid, "Raw sensor vehicle identity mismatch")
            require(str(np.asarray(packet["sensor_path"]).item()) == contract["sensor_ids"][vid], "Raw sensor path mismatch")
            for key in ("xyz", "offset", "start_position", "end_position", "timestamp", "frame_start", "frame_end"):
                require(np.isfinite(packet[key]).all(), "Nonfinite raw packet " + key)
            require(packet["start_position"].shape == (3,) and packet["end_position"].shape == (3,), "Malformed sensor origin")
            require(int(packet["frame_start"]) <= int(packet["frame_end"]), "Reversed raw frame clock")
            own = row["ego"][vid]
            require(np.asarray(packet["scan_complete"]).shape == (), "Malformed native scan completion flag")
            require(np.asarray(packet["scan_complete"]).item() in (0, 1), "Invalid native scan completion flag")
            points, healthy, age = packet_points(packet, own, row["sensor_now"], require_scan_complete=True)
            own_points[vid] = points
            timestamp = int(packet["timestamp"])
            require(timestamp >= previous[vid], "Raw packet timestamp regressed")
            close(timestamp, observation["packet_timestamp"], "Packet timestamp")
            origins[vid] = packet["end_position"]
            pose_error = float(np.linalg.norm(origins[vid] - np.array([own["x"], own["y"], contract["sensor_height_m"]])))
            pose_valid = pose_error <= own["speed"] * .1 + .3
            fresh = healthy and pose_valid and -.001 <= observation["receipt_age"] <= .25
            is_new = timestamp > previous[vid]
            for key, expected in (("healthy", healthy), ("fresh", fresh), ("is_new", is_new),
                                  ("sensor_age", age), ("pose_error_m", pose_error), ("pose_valid", pose_valid)):
                close(observation[key], expected, vid + " observation " + key)
            dropout = config.get("dropout_step", -1)
            if vid == config.get("dropout_vehicle", "ego") and dropout >= 0 and index >= dropout:
                require(vid in frozen, "Dropout lacks pre-fault evidence")
                require(all(np.array_equal(packet[key], frozen[vid][key]) for key in frozen[vid]), "Dropout packet changed")
            else:
                frozen[vid] = packet
            if fresh and is_new:
                observation_time = contract["traffic_origin"] + (timestamp + int(np.max(packet["offset"]))) * 1e-9 - contract["sensor_origin"]
                tracks[vid] = trackers[vid].update(points, observation_time)
                previous[vid] = timestamp
                sensor_counts[vid] += 1
            close(observation["tracks"], [asdict(track) for track in tracks[vid]], vid + " replayed tracks")
            messages = [decode_message(data) for data in observation["messages"]]
            close([asdict(message) for message in messages], [asdict(message) for message in inboxes[vid].values()], vid + " replayed bus inbox")
            decisions[vid] = controllers[vid].step(own, tracks[vid], messages, now, fresh=fresh)
            close(row["decisions"][vid], asdict(decisions[vid]), vid + " replayed decision")
        require(not np.array_equal(origins["ego"], origins["peer"]), "Shared raw sensor origins")
        for vid in ids:
            other = next(other for other in ids if other != vid)
            bounds = body_bounds(row["ego"][other])
            points = own_points[vid]
            returns = int(np.sum((points[:, 0] >= bounds[0] - 2) & (points[:, 0] <= bounds[1] + 2) &
                                 (points[:, 1] >= bounds[2] - .3) & (points[:, 1] <= bounds[3] + .3)))
            close(row["peer_returns"][vid], returns, vid + " peer visibility returns")
        for vid in ids:
            own, decision = row["ego"][vid], decisions[vid]
            observed = ObservedObstacle(*decision.observed_obstacle) if decision.observed_obstacle else None
            bus.publish(vid, now, IntentPayload(own["x"], own["y"], own["speed"], decision.intended_lane, decision.phase, observed))
        swept = {}
        for vid in ids:
            before, after = body_bounds(row["ego"][vid]), body_bounds(row["ego_after"][vid])
            swept[vid] = (min(before[0], after[0]) - .02, max(before[1], after[1]) + .02,
                          min(before[2], after[2]) - .02, max(before[3], after[3]) + .02)
            gap = min(rectangle_gap(swept[vid], obstacle) for obstacle in obstacles)
            close(row["obstacle_gaps"][vid], gap, vid + " obstacle truth gap")
            min_obstacle = min(min_obstacle, gap)
            offroad = after[2] < -6.4 or after[3] > 0
            close(row["offroad"][vid], offroad, vid + " road bounds")
            intervention = abs(row["ego_after"][vid]["speed"] - decisions[vid].speed) > .15
            close(row["speed_interventions"][vid], intervention, vid + " realized speed")
            interventions += int(intervention)
            safe = safe and not offroad and gap > .3
        gap = rectangle_gap(swept["ego"], swept["peer"])
        close(row["vehicle_gap"], gap, "Vehicle truth gap")
        min_vehicle = min(min_vehicle, gap)
        safe = safe and gap > .3 and not row["collisions"]
    wall = summary["loop_wall_seconds"]
    require(math.isfinite(wall) and wall > 0, "Invalid wall duration")
    seconds = len(rows) * .1
    rtf = seconds / wall
    misses = sum(row["deadline_lateness"] > .1 for row in rows)
    if config.get("dropout_step", -1) < 0:
        behavior = controllers["ego"].phase == "complete"
    else:
        behavior = rows[-1]["ego_after"][config.get("dropout_vehicle", "ego")]["speed"] < .2
    isolation = all(any(row["peer_returns"][vid] >= 5 and row["observations"][vid]["fresh"]
                        for row in rows[:10]) for vid in ids)
    passed = bool(safe and behavior and isolation and .95 <= rtf <= 1.05 and misses == 0)
    computed = dict(passed=passed, behavior_passed=bool(safe and behavior), safe=bool(safe),
                    steps=len(rows), traffic_seconds=seconds, loop_real_time_factor=rtf,
                    deadline_misses_over_100ms=misses, min_vehicle_gap_m=min_vehicle,
                    min_obstacle_gap_m=min_obstacle, speed_interventions=interventions,
                    final_phases={vid: controllers[vid].phase for vid in ids},
                    communication=bus.telemetry, sensor_counts=sensor_counts,
                    peer_visibility_passed=isolation,
                    initial_peer_returns={vid: max(row["peer_returns"][vid] for row in rows[:10]) for vid in ids})
    for key, value in computed.items():
        close(summary[key], value, "Summary " + key)
    return dict(run=root.name, integrity_passed=True, **computed)


def audit(path):
    """Return a failed report for missing, corrupt, or inconsistent evidence."""
    try:
        return _audit(path)
    except Exception as error:
        return dict(run=Path(path).name, passed=False, integrity_passed=False,
                    audit_error=f"{type(error).__name__}: {error}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run")
    result = audit(parser.parse_args().run)
    print(json.dumps(result, indent=2, allow_nan=False))
    raise SystemExit(0 if result["passed"] else 1)
