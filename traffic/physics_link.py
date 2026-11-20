"""Message link between the SUMO/policy process and the Isaac Sim physics process (standard library only).

Isaac Sim's Python has no TraCI/sumolib/PettingZoo and must not get them installed, and the
traffic venv has no Isaac Sim, so a physics run is two processes on this machine:

- the venv process runs SUMO, the v14 environment and the policy, and owns a `RemoteDriver`
  (the `driver` of environments/highway_flow_rl_v14_physical_env.py);
- RemoteDriver starts scripts/physics_drive_server.py under Isaac Sim's python, which connects
  back to a 127.0.0.1 socket and proves a per-run random token before any message is accepted.

Messages are length-prefixed JSON objects (no pickle). Every request gets exactly one reply;
a reply with "error" raises in the caller with the server's traceback.
"""
from __future__ import annotations

import json
import os
import secrets
import socket
import struct
import subprocess
import time
from pathlib import Path

HEADER = struct.Struct('<I')
MAX_MESSAGE = 64 * 1024 * 1024
TOKEN_ENV = 'HIGHWAYSIM_PHYSICS_LINK_TOKEN'
ROOT = Path(__file__).resolve().parents[1]
ISAAC_PYTHON = r'C:\isaacsim\python.bat'
ROAD_MODEL = 'highway_usd/la_10_110/la_10_110_flat.usdc'
VEHICLE_PACKAGE = 'vehicles/sim_ready/americano-i7-ev/v14'


def canonical_checkout():
    """The checkout that holds Git's common directory (the vehicle packages live there, not in worktrees)."""
    common = subprocess.run(['git', 'rev-parse', '--path-format=absolute', '--git-common-dir'], cwd=ROOT,
                            capture_output=True, text=True, check=True).stdout.strip()
    return Path(common).parent


def server_command(output, *, network, manifest, render_hz=0, vehicle_package=None, extra=()):
    """Command line of scripts/physics_drive_server.py under Isaac Sim's python (RemoteDriver adds --link-port)."""
    package = vehicle_package or canonical_checkout() / VEHICLE_PACKAGE
    return ['cmd', '/c', ISAAC_PYTHON, str(ROOT / 'scripts/physics_drive_server.py'), '--output', str(output),
            '--vehicle-package', str(package), '--road-model', str(ROOT / ROAD_MODEL),
            '--surface-manifest', str(manifest), '--network', str(network), '--render-hz', str(render_hz), *extra]


def send(sock, message):
    data = json.dumps(message, allow_nan=False).encode('utf-8')
    sock.sendall(HEADER.pack(len(data)) + data)


def _exact(sock, size):
    chunks, remaining = [], size
    while remaining:
        chunk = sock.recv(min(remaining, 1 << 20))
        if not chunk:
            raise ConnectionError('Physics link closed')
        chunks.append(chunk)
        remaining -= len(chunk)
    return b''.join(chunks)


def receive(sock):
    (size,) = HEADER.unpack(_exact(sock, HEADER.size))
    if size > MAX_MESSAGE:
        raise ValueError('Physics link message too large')
    return json.loads(_exact(sock, size).decode('utf-8'))


def connect_back(port, token, timeout_s=60.):
    """Server side: connect to the waiting client and prove the run token."""
    sock = socket.create_connection(('127.0.0.1', int(port)), timeout=timeout_s)
    sock.settimeout(None)
    send(sock, dict(kind='hello', token=token))
    return sock


class RemoteDriver:
    """Driver interface of the physical environment, carried out by the Isaac Sim server process."""

    def __init__(self, server_command, *, log_path, startup_timeout_s=900., cwd=None):
        self.server_command, self.log_path = list(server_command), Path(log_path)
        self.startup_timeout_s, self.cwd = float(startup_timeout_s), cwd
        self.process = self.sock = None
        self.cars, self.server_info, self.replies = set(), None, []

    # --- process ------------------------------------------------------------------------
    def start(self):
        if self.process is not None:
            raise RuntimeError('Physics server already started')
        token = secrets.token_hex(16)
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listener.bind(('127.0.0.1', 0))
        listener.listen(1)
        listener.settimeout(1.)
        port = listener.getsockname()[1]
        env = dict(os.environ, **{TOKEN_ENV: token})
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        log = open(self.log_path, 'w', encoding='utf-8', errors='replace')
        self.process = subprocess.Popen(self.server_command + ['--link-port', str(port)], stdout=log,
                                        stderr=subprocess.STDOUT, env=env, cwd=self.cwd)
        deadline = time.monotonic() + self.startup_timeout_s
        try:
            while True:
                if self.process.poll() is not None:
                    raise RuntimeError(f'Physics server exited during startup ({self.process.returncode}); see {self.log_path}')
                if time.monotonic() > deadline:
                    raise TimeoutError('Physics server did not connect in time')
                try:
                    sock, _ = listener.accept()
                except socket.timeout:
                    continue
                sock.settimeout(60.)
                hello = receive(sock)
                if hello.get('kind') != 'hello' or not secrets.compare_digest(str(hello.get('token')), token):
                    sock.close()
                    continue
                sock.settimeout(None)
                self.sock = sock
                break
        finally:
            listener.close()
        self.server_info = self._request(dict(kind='describe'))
        return self.server_info

    def _request(self, message):
        send(self.sock, message)
        reply = receive(self.sock)
        if reply.get('error'):
            raise RuntimeError('Physics server error:\n' + reply['error'])
        return reply

    # --- driver interface ---------------------------------------------------------------
    def reset(self, lanes):
        if self.process is None:
            self.start()
        self.cars.clear()

    def spawn(self, agent, pose):
        if self.cars:
            raise ValueError('The physics server drives one car')
        reply = self._request(dict(kind='spawn', agent=agent, pose=pose))
        self.cars.add(agent)
        return reply

    def advance(self, commands, dt, time=None, background=()):
        reply = self._request(dict(kind='advance', commands=commands, dt=dt, time=time, background=list(background)))
        self.replies.append(reply.get('telemetry'))
        return reply['poses']

    def remove(self, agent):
        if agent in self.cars:
            self._request(dict(kind='remove', agent=agent))
            self.cars.discard(agent)

    def active(self):
        return sorted(self.cars)

    def describe(self):
        return dict(kind='isaac_physx_remote', server=self.server_info, server_log=str(self.log_path))

    def close(self, timeout_s=600.):
        """Ask the server to finish (it writes its report and frames) and wait for it to exit."""
        report, failure = None, None
        if self.sock is not None:
            try:
                report = self._request(dict(kind='close'))
            except (OSError, RuntimeError, ValueError) as error:
                failure = error
            finally:
                self.sock.close()
                self.sock = None
        if self.process is not None:
            try:
                self.process.wait(timeout=timeout_s if failure is None else 60.)
            except subprocess.TimeoutExpired:
                # python.bat runs Kit under cmd; end the whole process tree
                subprocess.run(['taskkill', '/F', '/T', '/PID', str(self.process.pid)], capture_output=True)
                if failure is None:
                    raise
        if failure is not None:
            raise failure
        return report
