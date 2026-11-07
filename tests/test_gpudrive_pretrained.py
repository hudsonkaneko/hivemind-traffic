"""Feature-contract tests plus optional parity with the pinned author release."""

import ast
import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

torch = pytest.importorskip("torch")

from traffic.gpudrive_pretrained import (
    CONFIG_SHA256,
    OBSERVATION_DIM,
    SOURCE_REVISION,
    WEIGHTS_SHA256,
    FrozenGPUDrivePolicy,
    RoadType,
    action_values,
    encode_observation,
)


UPSTREAM = Path(__file__).resolve().parents[1] / "outputs/gpudrive-i110/v01/upstream"


def vehicle(**overrides):
    return dict(x=0.0, y=0.0, yaw=0.0, speed=20.0, length=4.8, width=2.0) | overrides


def test_observation_frame_units_and_padding():
    ego = vehicle(yaw=math.pi / 2)
    partner = vehicle(x=-3.0, y=10.0, speed=10.0, yaw=math.pi)
    road = dict(start=(-2.0, 0.0), end=(-2.0, 20.0), type=RoadType.EDGE)
    obs = encode_observation(ego, [partner], [road], goal=(0.0, 100.0))
    assert obs.shape == (OBSERVATION_DIM,)
    assert obs.dtype == np.float32
    np.testing.assert_allclose(obs[:6], [0.2, 4.8 / 30, 2 / 15, 0.1, 0, 0], atol=1e-7)
    # Other speed is absolute, not a velocity difference. Full dimensions
    # (not .7 collision dimensions) and orientation / 2pi are checkpoint-era.
    np.testing.assert_allclose(obs[6:12], [0.1, 0.01, 0.003, 0.25, 4.8 / 30, 2 / 15], atol=1e-7)
    assert np.count_nonzero(obs[12:384]) == 0
    road_rows = obs[384:].reshape(200, 13)
    np.testing.assert_allclose(road_rows[0, :6], [.01, .002, .1, .001, .001, 0], atol=1e-7)
    np.testing.assert_array_equal(road_rows[0, 6:], [0, 1, 0, 0, 0, 0, 0])
    np.testing.assert_array_equal(road_rows[1:, :6], 0)
    np.testing.assert_array_equal(road_rows[1:, 6], 1)
    np.testing.assert_array_equal(road_rows[1:, 7:], 0)


def test_visibility_radius_and_linear_selection():
    # Exact radius boundary is included; source filters only distances > 50.
    near = vehicle(x=50, speed=11)
    far = vehicle(x=50.01, speed=99)
    partners = [far, near] + [vehicle(x=1, speed=i) for i in range(70)]
    roads = [dict(start=(51, 0), end=(53, 0), type=1)]
    roads += [dict(start=(49, 0), end=(51, 0), type=2)]
    roads += [dict(start=(i / 10, 0), end=(i / 10 + 1, 0), type=3) for i in range(210)]
    obs = encode_observation(vehicle(), partners, roads, goal=(2000, 0))
    assert obs[3] == 2  # No normalization clipping.
    rows = obs[6:384].reshape(63, 6)
    assert rows[0, 0] == pytest.approx(.11)
    assert rows[-1, 0] == pytest.approx(.61)
    road_rows = obs[384:].reshape(200, 13)
    assert road_rows[0, 8] == 1  # First in radius, not nearest midpoint.
    assert road_rows[0, 0] == pytest.approx(.05, abs=1e-7)


def test_global_rigid_transform_keeps_local_features():
    ego = vehicle(x=4, y=5, yaw=.2)
    partner = vehicle(x=10, y=7, yaw=-.5)
    road = dict(start=(0, 1), end=(15, 1), type=3)
    goal = (30, 5)
    first = encode_observation(ego, [partner], [road], goal)
    angle, tx, ty = .7, 1200, -800

    def point(p):
        x, y = p
        return (math.cos(angle) * x - math.sin(angle) * y + tx,
                math.sin(angle) * x + math.cos(angle) * y + ty)

    def moved(v):
        x, y = point((v["x"], v["y"]))
        return v | dict(x=x, y=y, yaw=v["yaw"] + angle)

    second = encode_observation(
        moved(ego), [moved(partner)],
        [road | dict(start=point(road["start"]), end=point(road["end"]))], point(goal),
    )
    np.testing.assert_allclose(first, second, atol=1e-7)


def test_action_bins_are_the_authors_cartesian_product():
    expected_accel = torch.round(torch.linspace(-4, 4, 7), decimals=3)
    expected_steer = torch.round(torch.linspace(-torch.pi, torch.pi, 13), decimals=3)
    expected = [(float(a), float(s)) for a in expected_accel for s in expected_steer]
    assert [action_values(i) for i in range(91)] == expected
    assert action_values(45) == (0.0, 0.0)
    assert action_values(0) == pytest.approx((-4.0, -3.142))
    with pytest.raises(ValueError):
        action_values(91)


def test_observation_rejects_nonfinite():
    with pytest.raises(ValueError, match="finite"):
        encode_observation(vehicle(speed=float("nan")), [], [], (10, 0))


@pytest.fixture
def policy():
    if not (UPSTREAM / "model.safetensors").exists():
        pytest.skip("Pinned author checkpoint is an ignored local evaluation artifact")
    torch.set_num_threads(1)
    return FrozenGPUDrivePolicy.from_pretrained(UPSTREAM)


def test_pinned_checkpoint_is_frozen_cpu_and_repeatable(policy):
    assert hashlib.sha256((UPSTREAM / "model.safetensors").read_bytes()).hexdigest() == WEIGHTS_SHA256
    assert hashlib.sha256((UPSTREAM / "config.json").read_bytes()).hexdigest() == CONFIG_SHA256
    assert sum(p.numel() for p in policy.network.parameters()) == 51228
    assert not policy.network.training
    assert all(not p.requires_grad and p.device.type == "cpu" for p in policy.network.parameters())
    obs = encode_observation(vehicle(), [], [], (100, 0))
    before = obs.copy()
    first = policy.infer(obs)
    assert policy.infer(obs) == first
    np.testing.assert_array_equal(obs, before)
    assert first[:2] == action_values(first[2])
    with pytest.raises(ValueError, match="2984"):
        policy.infer(np.zeros(4))


def test_checksum_guard_rejects_replaced_weights(policy, tmp_path):
    (tmp_path / "config.json").write_bytes((UPSTREAM / "config.json").read_bytes())
    (tmp_path / "model.safetensors").write_bytes(b"not the approved checkpoint")
    with pytest.raises(ValueError, match="SHA256"):
        FrozenGPUDrivePolicy(tmp_path)


def test_logits_match_exact_pinned_upstream_class(policy):
    """Run the actual downloaded class with dependency-only shims, no runtime.

    This catches architecture drift against the author implementation, rather
    than comparing two copies of our own forward method. Only the reviewed,
    SHA256-pinned NeuralNet AST is executed; top-level imports are excluded.
    """
    manifest = json.loads((UPSTREAM / "manifest.json").read_text())
    assert manifest["github_revision"] == SOURCE_REVISION
    path = "gpudrive/networks/late_fusion.py"
    record = next(item for item in manifest["files"] if item["path"] == path)
    raw = (UPSTREAM / path).read_bytes()
    assert hashlib.sha256(raw).hexdigest() == record["sha256"]
    assert record["sha256"] == "966b28da3fe1c8659a8e2c00a1567233ff190b2dd4578d6fcb8734b80dc3af3e"
    parsed = ast.parse(raw.decode())
    cls = next(node for node in parsed.body if isinstance(node, ast.ClassDef) and node.name == "NeuralNet")
    cls.bases = [ast.Attribute(value=ast.Name(id="nn", ctx=ast.Load()), attr="Module", ctx=ast.Load())]
    cls.keywords = []  # Remove optional Hugging Face registration, not model code.
    module = ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[]))

    class Config(dict):
        __getattr__ = dict.__getitem__

    namespace = dict(
        nn=torch.nn, torch=torch, Box=Config, TOP_K_ROAD_POINTS=200,
        constants=SimpleNamespace(EGO_FEAT_DIM=6, PARTNER_FEAT_DIM=6, ROAD_GRAPH_FEAT_DIM=13),
        pufferlib=SimpleNamespace(pytorch=SimpleNamespace(layer_init=lambda layer, **kwargs: layer)),
    )
    exec(compile(module, str(UPSTREAM / path), "exec"), namespace)
    upstream = namespace["NeuralNet"](config={"vbd_in_obs": False}).eval()
    upstream.load_state_dict(policy.network.state_dict(), strict=True)
    rng = np.random.default_rng(42)
    observations = rng.normal(0, .1, (8, OBSERVATION_DIM)).astype(np.float32)
    observations[0] = encode_observation(vehicle(), [], [], (100, 0))
    with torch.inference_mode():
        expected = upstream.actor(upstream.encode_observations(torch.from_numpy(observations))).numpy()
    np.testing.assert_array_equal(policy.logits(observations), expected)
