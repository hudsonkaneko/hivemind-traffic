"""Generate a retained-attempt report from explicitly supplied fleet suites."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.audit_cooperative_lidar import audit


def resolve_project_path(value):
    path = Path(value)
    if not path.is_absolute():
        path = ROOT / path
    path = path.resolve()
    if not path.is_relative_to(ROOT):
        raise ValueError("Evidence must be inside the canonical project root")
    return path


def read_json(path):
    def reject(value):
        raise ValueError("Nonfinite JSON value " + value)
    return json.loads(path.read_text(encoding="utf-8"), parse_constant=reject)


def portable(path):
    return path.resolve().relative_to(ROOT).as_posix()


def repeat_comparison(first, second):
    result = dict(first=first.get("output"), second=second.get("output"), available=False,
                  exact=False, max_differences={}, phase_and_requests_equal=False,
                  comparison="Each car's x/y/angle/speed and phase/lane requests by simulation step; wall timestamps excluded")
    try:
        traces = [read_json(resolve_project_path(record["output"]) / "telemetry.json") for record in (first, second)]
        result["row_counts"] = list(map(len, traces))
        if not all(traces) or len(traces[0]) != len(traces[1]):
            raise ValueError("Missing or different trace lengths")
        differences = {}
        for vehicle_id in ("ego", "peer"):
            for field in ("x", "y", "angle", "speed"):
                delta = max(abs(a[pose_field][vehicle_id][field] - b[pose_field][vehicle_id][field])
                            for a, b in zip(*traces) for pose_field in ("ego", "ego_after"))
                if not math.isfinite(delta):
                    raise ValueError("Nonfinite repeat trace")
                differences[vehicle_id + "." + field] = delta
        same_phases = all(a["decisions"][vid][field] == b["decisions"][vid][field]
                          for a, b in zip(*traces) for vid in ("ego", "peer")
                          for field in ("phase", "lane_request", "intended_lane"))
        result.update(available=True, max_differences=differences,
                      phase_and_requests_equal=same_phases,
                      exact=same_phases and all(delta == 0 for delta in differences.values()))
    except (OSError, ValueError, KeyError, TypeError) as error:
        result["error"] = str(error)
    return result


def inspect_attempt(suite_path, index, record):
    attempt = dict(suite=portable(suite_path), attempt_index=index, case=record.get("case", "unknown"),
                   launch_record=record, launcher_passed=record.get("passed") is True,
                   passed=False, audit=dict(passed=False, integrity_passed=False, audit_error="No completed output"))
    summary = record.get("summary")
    attempt["summary"] = summary if isinstance(summary, dict) else {}
    output = record.get("output") or attempt["summary"].get("output")
    if not output:
        return attempt
    try:
        directory = resolve_project_path(output)
        attempt["output"] = portable(directory)
        attempt["audit"] = audit(directory)
        manifest_path = directory / "manifest.json"
        manifest = read_json(manifest_path)
        attempt["provenance"] = dict(manifest=portable(manifest_path),
            manifest_sha256=hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
            git_commit=manifest.get("git_commit"), git_status=manifest.get("git_status"),
            python=manifest.get("python"), sumo_version=manifest.get("sumo_version"),
            gpu=manifest.get("gpu"), runtime_dependencies=manifest.get("runtime_dependencies"),
            source_hashes=manifest.get("source_hashes"), config=manifest.get("config"),
            status=manifest.get("status"), failure_reason=manifest.get("failure_reason", manifest.get("error")))
        if (directory / "summary.json").exists():
            attempt["summary"] = read_json(directory / "summary.json")
        attempt["passed"] = (attempt["launcher_passed"] and record.get("exit_code") == 0
                              and not record.get("timed_out", False)
                              and attempt["audit"].get("integrity_passed") is True
                              and attempt["audit"].get("passed") is True)
    except (OSError, ValueError, TypeError) as error:
        attempt["report_error"] = str(error)
    return attempt


def git_snapshot():
    result = {}
    for name, command in (("commit", ["git", "rev-parse", "HEAD"]),
                          ("status", ["git", "status", "--porcelain"])):
        try:
            result[name] = subprocess.check_output(command, cwd=ROOT, text=True, stderr=subprocess.DEVNULL).strip()
        except (OSError, subprocess.CalledProcessError) as error:
            result[name + "_error"] = str(error)
    return result


def build_report(suite_values):
    report = dict(generated_at=datetime.now(timezone.utc).isoformat(), canonical_root=str(ROOT),
                  reporter_git=git_snapshot(), suites=[], attempts=[], repeat_checks=[], passed=False,
                  scope="Two-car scripted static-obstacle fixture with local lidar tracking and optional V2V intents; SUMO motion authority",
                  interpretation="Communication ablation is a fixture comparison, not statistical proof of cooperative-policy performance or general safety.")
    for value in suite_values:
        try:
            path = resolve_project_path(value)
            suite = read_json(path)
            retained = dict(path=portable(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                            completed=suite.get("completed", False), passed=suite.get("passed", False),
                            selected_cases=suite.get("selected_cases", []), source=suite)
            report["suites"].append(retained)
            attempts = []
            for index, record in enumerate(suite.get("runs", [])):
                attempt = inspect_attempt(path, index, record)
                report["attempts"].append(attempt)
                attempts.append(attempt)
            present = {attempt["case"] for attempt in attempts}
            for name in suite.get("selected_cases", []):
                if name not in present:
                    report["attempts"].append(dict(suite=portable(path), case=name, pending=True,
                        passed=False, launcher_passed=False, summary={}, audit=dict(passed=False,
                        integrity_passed=False, audit_error="Selected case has no retained attempt yet")))
            first = next((attempt for attempt in attempts if attempt["case"] == "headless-ideal"), None)
            second = next((attempt for attempt in attempts if attempt["case"] == "repeat-ideal"), None)
            if first is not None or second is not None:
                repeat = repeat_comparison(first or {}, second or {})
                repeat["suite"] = portable(path)
                repeat["audits_passed"] = bool(first and second and first["audit"].get("integrity_passed") and second["audit"].get("integrity_passed"))
                report["repeat_checks"].append(repeat)
        except (OSError, ValueError, TypeError, KeyError) as error:
            report["suites"].append(dict(input=str(value), completed=False, passed=False, error=str(error)))
    report["passed"] = bool(report["attempts"] and all(attempt["passed"] for attempt in report["attempts"])
                            and all(suite.get("completed") for suite in report["suites"])
                            and all(check["exact"] and check["audits_passed"] for check in report["repeat_checks"]))
    return report


def cell(value, digits=3):
    if value is None:
        return "N/A"
    if isinstance(value, float):
        return f"{value:.{digits}f}"
    return str(value).replace("|", "\\|").replace("\n", " ")


def private_growth(memory):
    for key in ("private_delta_bytes", "private_growth_bytes", "private_bytes_delta"):
        if memory.get(key) is not None:
            return memory[key]
    samples = memory.get("samples", [])
    if len(samples) >= 2 and all("private_bytes" in sample for sample in (samples[0], samples[-1])):
        return samples[-1]["private_bytes"] - samples[0]["private_bytes"]
    return None


def markdown(report):
    lines = ["# Cooperative lidar results", "", report["scope"], "", report["interpretation"], "",
             "Every supplied attempt is retained below, including failures and unfinished cases. Missing measurements are N/A.", "",
             "| Suite / case | Full pass | Audit integrity | Complete time (s) | Min car gap (m) | Min obstacle gap (m) | RTF | >100 ms late |",
             "|---|---|---|---|---|---|---|---|"]
    for attempt in report["attempts"]:
        summary, checked = attempt.get("summary", {}), attempt.get("audit", {})
        lines.append("| " + " | ".join(cell(value) for value in (
            Path(attempt["suite"]).stem + " / " + attempt["case"], attempt["passed"], checked.get("integrity_passed"),
            summary.get("completion_time"), checked.get("min_vehicle_gap_m", summary.get("min_vehicle_gap_m")),
            checked.get("min_obstacle_gap_m", summary.get("min_obstacle_gap_m")),
            summary.get("loop_real_time_factor"), summary.get("deadline_misses_over_100ms"))) + " |")
    lines += ["", "## Memory and communication", "",
              "| Suite / case | RSS delta (MiB) | Private growth (MiB) | Last-half RSS slope (MiB/s) | Sent | Delivered | Dropped | Expired |",
              "|---|---|---|---|---|---|---|---|"]
    for attempt in report["attempts"]:
        summary = attempt.get("summary", {})
        memory, communication = summary.get("memory", {}), summary.get("communication", {})
        values = (memory.get("rss_delta_bytes"), private_growth(memory), memory.get("last_half_slope_bytes_per_second"))
        lines.append("| " + " | ".join(cell(value) for value in (
            Path(attempt["suite"]).stem + " / " + attempt["case"],
            *(value / 2**20 if value is not None else None for value in values),
            *(communication.get(kind + "_messages") for kind in ("sent", "delivered", "dropped", "expired")))) + " |")
    lines += ["", "RSS measures the whole runtime process. Private growth is unavailable when the runtime did not record it. Bounded memory samples do not establish leak-free indefinite operation.",
              "", "## Repeat and second seed", ""]
    for check in report["repeat_checks"]:
        lines += [f"- {check['suite']}: exact simulation repeat={check['exact']}, audits={check['audits_passed']}; phase/request equality={check['phase_and_requests_equal']}; maximum differences={cell(check['max_differences'])}; error={cell(check.get('error'))}."]
    if not report["repeat_checks"]:
        lines.append("No ideal/repeat pair was supplied.")
    lines += ["", "The seed-43 case is listed separately. One additional seed does not provide statistical confidence; communication modes share a scripted fixture rather than a learned or randomized traffic policy.",
              "", "## Retained failures and evidence", ""]
    for attempt in report["attempts"]:
        errors = [attempt.get("report_error"), attempt.get("audit", {}).get("audit_error"),
                  attempt.get("launch_record", {}).get("error"), attempt.get("launch_record", {}).get("launch_error")]
        lines.append(f"- {attempt['case']}: output={cell(attempt.get('output'))}; log={cell(attempt.get('launch_record', {}).get('log'))}; result={attempt['passed']}; errors={cell('; '.join(str(error) for error in errors if error) or 'none')}.")
    for suite in report["suites"]:
        if suite.get("error"):
            lines.append(f"- Unreadable suite {cell(suite.get('input'))}: {cell(suite['error'])}.")
    lines += ["", "## Provenance and limits", "",
              "The JSON report retains each supplied suite snapshot, launch command, exit status, logs, manifest hash, archived source hashes, Git commit and working-tree state, Python/SUMO versions, GPU/driver information, and any recorded runtime dependency versions.",
              "", "Audits reconstruct per-car raw sensor filtering, tracking, controller decisions and seeded message delivery. Safety gaps and peer visibility are evaluated after decisions against fixture geometry. Receipt age and wall deadline measurements are hashed runtime observations; they are not independently timed by this report.",
              "", "Loop real-time factor excludes startup, warm-up, final hashing and shutdown. GUI and headless cases remain separate. Results do not certify hard real-time deadlines, vehicle physics, long-running service stability, or city-scale coordination.",
              "", f"All supplied attempts and available repeat checks passed: {report['passed']}.", ""]
    return "\n".join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("suites", nargs="+", help="Suite JSON paths, absolute or relative to canonical root")
    args = parser.parse_args(argv)
    report = build_report(args.suites)
    destination = ROOT / "documentation"
    destination.mkdir(exist_ok=True)
    (destination / "cooperative-lidar-results.json").write_text(json.dumps(report, indent=2, allow_nan=False), encoding="utf-8")
    (destination / "cooperative-lidar-results.md").write_text(markdown(report), encoding="utf-8")
    print(json.dumps(dict(passed=report["passed"], attempts=len(report["attempts"]),
                         output=str(destination / "cooperative-lidar-results.json"))))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
