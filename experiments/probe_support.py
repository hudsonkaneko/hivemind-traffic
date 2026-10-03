"""Small provenance and low-frequency Windows/NVIDIA instrumentation helpers."""
from datetime import datetime, timezone
import ctypes
from ctypes import wintypes
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import threading
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]


def process_sample(pid=None):
    if os.name != 'nt':
        return {'available': False, 'reason': 'Windows sampler; unsupported OS'}
    class Memory(ctypes.Structure):
        _fields_ = [('cb', wintypes.DWORD), ('PageFaultCount', wintypes.DWORD)] + [
            (n, ctypes.c_size_t) for n in ('PeakWorkingSetSize', 'WorkingSetSize', 'QuotaPeakPagedPoolUsage',
            'QuotaPagedPoolUsage', 'QuotaPeakNonPagedPoolUsage', 'QuotaNonPagedPoolUsage', 'PagefileUsage', 'PeakPagefileUsage', 'PrivateUsage')]
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    kernel.OpenProcess.argtypes = (wintypes.DWORD, wintypes.BOOL, wintypes.DWORD)
    kernel.OpenProcess.restype = wintypes.HANDLE
    kernel.CloseHandle.argtypes = (wintypes.HANDLE,)
    kernel.GetProcessTimes.argtypes = (wintypes.HANDLE,) + (ctypes.POINTER(wintypes.FILETIME),)*4
    psapi = ctypes.WinDLL('psapi', use_last_error=True)
    psapi.GetProcessMemoryInfo.argtypes = (wintypes.HANDLE, ctypes.POINTER(Memory), wintypes.DWORD)
    handle = kernel.OpenProcess(0x410, False, pid or os.getpid())
    if not handle:
        return {'available': False, 'error': ctypes.get_last_error()}
    try:
        memory = Memory(); memory.cb = ctypes.sizeof(memory)
        times = [wintypes.FILETIME() for _ in range(4)]
        if not psapi.GetProcessMemoryInfo(handle, ctypes.byref(memory), memory.cb) or not kernel.GetProcessTimes(handle, *[ctypes.byref(v) for v in times]):
            return {'available': False, 'error': ctypes.get_last_error()}
        cpu = sum((v.dwHighDateTime << 32) + v.dwLowDateTime for v in times[2:]) / 1e7
        return dict(available=True, rss_bytes=memory.WorkingSetSize, peak_rss_bytes=memory.PeakWorkingSetSize, cpu_seconds=cpu)
    finally:
        kernel.CloseHandle(handle)


def gpu_sample():
    text = subprocess.check_output(['nvidia-smi', '--query-gpu=name,memory.total,memory.used,utilization.gpu,driver_version', '--format=csv,noheader,nounits'], text=True, timeout=8)
    name, total, used, utilization, driver = [v.strip() for v in text.splitlines()[0].split(',')]
    return dict(wall=time.perf_counter(), name=name, total_mib=float(total), used_mib=float(used), utilization_percent=float(utilization), driver=driver)


class GpuMonitor:
    def __init__(self):
        self.rows = []; self.errors = []; self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def _run(self):
        while not self.stop.is_set():
            try:
                self.rows.append(gpu_sample())
            except Exception as error:
                self.errors.append(repr(error))
            self.stop.wait(1.)

    def __enter__(self):
        self.thread.start(); return self

    def __exit__(self, *args):
        self.stop.set(); self.thread.join(timeout=10)


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def run_package(study, config, source_files):
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    output = ROOT / 'outputs' / study / (stamp + '-' + uuid.uuid4().hex[:8])
    output.mkdir(parents=True, exist_ok=False)
    for rel in source_files:
        target = output / 'source' / rel
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT / rel, target)
    diff = subprocess.check_output(['git', 'diff', 'HEAD'], cwd=ROOT)
    (output / 'working-tree.patch').write_bytes(diff)
    manifest = dict(status='running', started_utc=stamp, config=config,
        git_commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        git_status=subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT, text=True),
        python=platform.python_version(), platform=platform.platform(), logical_cpus=os.cpu_count(),
        source_hashes={rel: hashlib.sha256((ROOT/rel).read_bytes()).hexdigest() for rel in source_files})
    write_json(output/'manifest.json', manifest)
    write_json(output/'resolved-config.json', config)
    return output, manifest


def finish(output, manifest, result):
    write_json(output/'summary.json', result)
    manifest['status'] = 'passed' if result.get('passed') else 'failed'
    manifest['finished_utc'] = datetime.now(timezone.utc).isoformat()
    manifest['output_hashes'] = {str(p.relative_to(output)): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in output.rglob('*') if p.is_file() and p.name != 'manifest.json'}
    write_json(output/'manifest.json', manifest)
