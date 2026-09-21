"""Keep small, dated experiment records independently of mutable outputs."""
from datetime import datetime, timezone
from importlib.metadata import PackageNotFoundError, version
import hashlib
import json
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import uuid


def fingerprint(path):
    path = Path(path).resolve()
    if not path.is_file():
        return {"path": str(path), "exists": False}
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return {"path": str(path), "exists": True, "bytes": path.stat().st_size,
            "sha256": digest.hexdigest()}


def json_default(value):
    if isinstance(value, Path):
        return str(value)
    if callable(value):
        return f"{value.__module__}.{value.__qualname__}"
    return str(value)


class RunRecord:
    def __init__(self, project, arguments, command):
        self.project = Path(project).resolve()
        now = datetime.now(timezone.utc)
        self.run_id = now.strftime("%Y-%m-%dT%H-%M-%SZ") + "_" + uuid.uuid4().hex[:8]
        self.directory = self.project / "documentation" / "runs" / self.run_id
        self.directory.mkdir(parents=True, exist_ok=False)
        source_dir = self.directory / "source"
        source_dir.mkdir()
        files = []
        for name in ("train_waypoint.py", "waypoint_env.py", "run_record.py"):
            source = self.project / "scripts" / name
            shutil.copy2(source, source_dir / name)
            files.append(fingerprint(source_dir / name))
        packages = {}
        for name in ("isaaclab", "stable-baselines3", "torch", "gymnasium", "warp-lang"):
            try:
                packages[name] = version(name)
            except PackageNotFoundError:
                packages[name] = None
        git = subprocess.run(["git", "rev-parse", "HEAD"], cwd=self.project,
                             capture_output=True, text=True, check=False)
        checkpoint = arguments.get("checkpoint")
        self.data = {
            "schema_version": 1, "run_id": self.run_id, "status": "started",
            "started_at_utc": now.isoformat(),
            "started_at_local": now.astimezone().isoformat(),
            "project": str(self.project), "working_directory": str(Path.cwd()),
            "command_argv": [sys.executable, *command], "arguments": dict(arguments),
            "python": sys.version, "platform": platform.platform(), "packages": packages,
            "git_commit": git.stdout.strip() if git.returncode == 0 else None,
            "source_snapshots": files,
            "input_checkpoint": fingerprint(checkpoint) if checkpoint else None,
        }
        self.write()

    def update(self, **values):
        self.data.update(values)
        self.write()

    def finish(self, status, summary=None, error=None, policy_path=None):
        self.data.update(status=status, ended_at_utc=datetime.now(timezone.utc).isoformat())
        if summary is not None:
            self.data["summary"] = summary
        if error is not None:
            self.data["error"] = error
        if policy_path is not None:
            self.data["output_policy"] = fingerprint(policy_path)
        self.write()

    def write(self):
        pending = self.directory / "manifest.tmp"
        pending.write_text(json.dumps(self.data, indent=2, default=json_default), encoding="utf-8")
        pending.replace(self.directory / "manifest.json")
        summary = self.data.get("summary", {})
        lines = [f"# Run {self.run_id}", "", f"Status: **{self.data['status']}**", "",
                 f"Started (local): {self.data['started_at_local']}", "",
                 f"Mode: `{self.data['arguments'].get('mode')}`", "",
                 "[Full record and parameters](manifest.json) · [Code snapshot](source/)", ""]
        if summary:
            lines += ["## Results", "", "```json", json.dumps(summary, indent=2), "```", ""]
        if self.data.get("error"):
            lines += ["## Error", "", "```text", self.data["error"], "```", ""]
        lines += ["A `started` record without an end time may be active or abruptly interrupted.",
                  "Checkpoint hashes identify files; the record does not back up model weights.", ""]
        (self.directory / "README.md").write_text("\n".join(lines), encoding="utf-8")
