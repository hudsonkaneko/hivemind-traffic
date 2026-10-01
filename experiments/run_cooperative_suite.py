"""Run serial cooperative lidar cases and retain every completed/failed result.

This launcher deliberately checks the result marker in addition to exit status:
Isaac's runtime batch wrapper can return zero after a failed Python script.
"""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from traffic.live_runtime import ROOT


CASES = (
    ("headless-none", 45, "none", 42, ["--headless"]),
    ("headless-ideal", 45, "ideal", 42, ["--headless"]),
    ("headless-degraded", 45, "degraded", 42, ["--headless"]),
    ("repeat-ideal", 45, "ideal", 42, ["--headless"]),
    ("seed43-ideal", 45, "ideal", 43, ["--headless"]),
    ("dropout-ego", 25, "ideal", 42, ["--headless", "--dropout-step", "20", "--dropout-vehicle", "ego"]),
    ("gui-points-off", 60, "ideal", 42, []),
    ("gui-points-on", 60, "ideal", 42, ["--points"]),
    ("extended-ideal", 120, "ideal", 42, ["--headless"]),
)


def stop_spawned_tree(process):
    """Terminate only the exact child PID/tree created by this suite."""
    if process.poll() is not None:
        return
    if os.name == "nt":
        subprocess.run(["taskkill", "/PID", str(process.pid), "/T", "/F"],
                       capture_output=True, timeout=20, check=False)
    else:
        os.killpg(process.pid, signal.SIGKILL)
    try:
        process.wait(timeout=20)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=10)


def parse_result(log, exit_code, timed_out):
    record = {"exit_code": exit_code, "timed_out": timed_out, "passed": False}
    try:
        markers = [line.split("FLEET_RESULT=", 1)[1] for line in log.read_text(
            encoding="utf-8", errors="replace").splitlines() if "FLEET_RESULT=" in line]
        if not markers:
            raise ValueError("No FLEET_RESULT summary; inspect retained log")
        def reject_nonfinite(value):
            raise ValueError("Nonfinite JSON result field: " + value)
        summary = json.loads(markers[-1], parse_constant=reject_nonfinite)
        if not isinstance(summary, dict):
            raise ValueError("FLEET_RESULT must contain a JSON object")
        record["summary"] = summary
        record["output"] = summary.get("output")
        record["passed"] = summary.get("passed") is True and exit_code == 0 and not timed_out
        if summary.get("passed") is not True:
            record["error"] = "Runner summary did not pass"
        elif exit_code != 0:
            record["error"] = "Process returned nonzero despite result summary"
        elif timed_out:
            record["error"] = "Process timed out despite result summary"
    except (ValueError, TypeError, OSError) as error:
        record["error"] = str(error)
    return record


def save_report(destination, report):
    temporary = destination.with_suffix(".partial.tmp")
    temporary.write_text(json.dumps(report, indent=2, allow_nan=False), encoding="utf-8")
    temporary.replace(destination)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cases", help="Comma-separated case names; default runs all cases")
    args = parser.parse_args(argv)
    selected = None if args.cases is None else set(args.cases.split(","))
    available = {case[0] for case in CASES}
    if selected is not None and (not selected or selected - available):
        parser.error("Unknown case names; choose from " + ",".join(sorted(available)))
    cases = [case for case in CASES if selected is None or case[0] in selected]
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "-" + uuid.uuid4().hex[:12]
    (ROOT / "logs").mkdir(exist_ok=True)
    output_directory = ROOT / "outputs/cooperative_lidar"
    output_directory.mkdir(parents=True, exist_ok=True)
    destination = output_directory / f"suite-{stamp}.json"
    report = {"suite_id": stamp, "started_at": datetime.now(timezone.utc).isoformat(),
              "selected_cases": [case[0] for case in cases], "completed": False,
              "passed": False, "runs": []}
    save_report(destination, report)
    for name, seconds, mode, seed, extra in cases:
        command = [sys.executable, str(ROOT / "scripts/demo_cooperative_lidar.py"),
                   "--seconds", str(seconds), "--comm", mode, "--seed", str(seed), *extra]
        log = ROOT / "logs" / f"cooperative-suite-{stamp}-{name}.log"
        print("START " + name, flush=True)
        timed_out, code, process = False, None, None
        failure = None
        try:
            with log.open("x", encoding="utf-8") as stream:
                process = subprocess.Popen(command, cwd=ROOT, stdout=stream,
                                           stderr=subprocess.STDOUT,
                                           start_new_session=os.name != "nt")
                try:
                    code = process.wait(timeout=270)
                except subprocess.TimeoutExpired:
                    timed_out = True
                    stop_spawned_tree(process)
                    code = process.returncode
        except Exception as error:
            failure = repr(error)
            if process is not None:
                try:
                    stop_spawned_tree(process)
                except Exception as cleanup_error:
                    failure += "; cleanup: " + repr(cleanup_error)
        record = parse_result(log, code, timed_out) if log.exists() else {"passed": False}
        record.update(case=name, seconds=seconds, comm_mode=mode, seed=seed,
                      command=command, log=str(log.relative_to(ROOT)),
                      spawned_pid=process.pid if process is not None else None)
        if failure is not None:
            record.update(passed=False, launch_error=failure)
        report["runs"].append(record)
        save_report(destination, report)
        print("DONE " + json.dumps(record, allow_nan=False), flush=True)
    report.update(completed=True, passed=all(run["passed"] for run in report["runs"]),
                  finished_at=datetime.now(timezone.utc).isoformat())
    save_report(destination, report)
    print("FLEET_SUITE=" + str(destination), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
