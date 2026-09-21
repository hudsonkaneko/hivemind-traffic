"""Train/evaluate a waypoint policy, or run random actions to check the task."""
import argparse
import json
from pathlib import Path
import sys
import time

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--mode", choices=["train", "evaluate", "smoke"], default="train")
parser.add_argument("--num_envs", type=int, default=16)
parser.add_argument("--steps", type=int, default=32768)
parser.add_argument("--seed", type=int, default=42)
parser.add_argument("--checkpoint", type=str)
parser.add_argument("--target_angle", type=float, default=0.65)
parser.add_argument("--output", type=str, default="outputs/waypoint")
parser.add_argument("--capture", type=str, help="Save a viewport PNG during a --viz kit run")
parser.add_argument("--hold_open", action="store_true", help="Keep the GUI open after evaluation until Isaac Sim is closed")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
from run_record import RunRecord

record = RunRecord(Path(__file__).resolve().parents[1], vars(args), sys.argv)
try:
    launcher = AppLauncher(args)
    app = launcher.app

    import numpy as np
    import torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import CheckpointCallback
    from isaaclab_rl.sb3 import Sb3VecEnvWrapper
    from waypoint_env import WaypointCfg, WaypointEnv
except BaseException:
    import traceback
    record.finish("failed", error=traceback.format_exc())
    raise


def record_policy(model):
    settings = {key: getattr(model, key) for key in (
        "n_steps", "batch_size", "n_epochs", "gamma", "gae_lambda", "ent_coef",
        "vf_coef", "max_grad_norm", "normalize_advantage", "target_kl", "seed",
        "policy_kwargs", "num_timesteps",
    )}
    settings.update(learning_rate=model.lr_schedule(1.0), clip_range=model.clip_range(1.0),
                    optimizer=str(type(model.policy.optimizer)),
                    activation=str(model.policy.activation_fn), device=str(model.device))
    record.update(effective_ppo=settings)


def main():
    torch.set_num_threads(4)
    cfg = WaypointCfg()
    cfg.seed = args.seed
    cfg.scene.num_envs = args.num_envs
    cfg.sim.device = args.device
    cfg.target_angle = args.target_angle
    cfg.show_targets = bool(args.visualizer and "kit" in args.visualizer)
    cfg.viewer.eye = (6, 6, 5)
    cfg.viewer.lookat = (0, 0, 0)
    record.update(resolved_arguments=vars(args), resolved_task=cfg.to_dict())
    raw = WaypointEnv(cfg)
    env = Sb3VecEnvWrapper(raw)
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    (output / "config.json").write_text(json.dumps(vars(args), indent=2))
    start = time.monotonic()
    if args.mode == "train":
        if args.checkpoint:
            model = PPO.load(args.checkpoint, env=env, device="cpu")
            model.tensorboard_log = str(output / "tensorboard")
        else:
            model = PPO("MlpPolicy", env, n_steps=256, batch_size=256, n_epochs=5,
                        learning_rate=3e-4, gamma=0.99, gae_lambda=0.95,
                        ent_coef=0.01, policy_kwargs={"net_arch": [128, 128]},
                        seed=args.seed, device="cpu", verbose=1,
                        tensorboard_log=str(output / "tensorboard"))
        record_policy(model)
        callback = CheckpointCallback(save_freq=max(4096 // args.num_envs, 1),
                                      save_path=str(output / "checkpoints"), name_prefix="waypoint")
        model.learn(total_timesteps=args.steps, callback=callback)
        model.save(str(output / "policy"))
    else:
        rng = np.random.default_rng(args.seed)
        model = PPO.load(args.checkpoint, device="cpu") if args.checkpoint else None
        if model is not None:
            record_policy(model)
        if args.mode == "evaluate" and model is None:
            raise ValueError("Evaluation requires --checkpoint")
        obs = env.reset()
        trace = []
        capture = None
        executed_steps = 0
        for step in range(args.steps):
            if not app.is_running():
                break
            frame_start = time.monotonic()
            action = model.predict(obs, deterministic=True)[0] if model else rng.uniform(-1, 1, (args.num_envs, 2)).astype(np.float32)
            obs, reward, done, info = env.step(action)
            executed_steps += 1
            if not np.isfinite(obs).all() or not np.isfinite(reward).all():
                raise RuntimeError("Non-finite observations/rewards")
            if step % 10 == 0:
                trace.append({"step": step, "position": raw.car.data.root_pos_w.torch[0].tolist(),
                              "target": raw.targets[0].tolist(),
                              "raw_policy_action": action[0].tolist(),
                              "applied_action": np.clip(action[0], -1, 1).tolist()})
            if args.capture and step == min(100, args.steps // 2):
                from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
                capture_path = Path(args.capture).resolve()
                capture_path.parent.mkdir(parents=True, exist_ok=True)
                capture = capture_viewport_to_file(get_active_viewport(), str(capture_path))
            if cfg.show_targets:
                time.sleep(max(0.0, raw.step_dt - (time.monotonic() - frame_start)))
        (output / "trace.json").write_text(json.dumps(trace, indent=2))
        if args.hold_open and cfg.show_targets:
            print("[INFO] Evaluation finished. Isaac Sim is being kept open; close the window or press Ctrl+C to exit.", flush=True)
            while app.is_running():
                app.update()
    summary = {"mode": args.mode, "elapsed_s": time.monotonic() - start,
               "successes": raw.success_count, "failures": raw.failure_count,
               "timeouts": raw.timeout_count, "requested_steps": args.steps,
               "num_envs": args.num_envs}
    if args.mode == "train":
        summary["policy_path"] = str(output / "policy.zip")
        summary["model_timesteps"] = model.num_timesteps
    else:
        summary["executed_control_steps"] = executed_steps
    episodes = raw.success_count + raw.failure_count + raw.timeout_count
    summary["completed_episodes"] = episodes
    summary["success_rate"] = raw.success_count / episodes if episodes else None
    (output / "summary.json").write_text(json.dumps(summary, indent=2))
    print("RUN_RESULT=" + json.dumps(summary), flush=True)
    env.close()
    return summary, output / "policy.zip" if args.mode == "train" else None


if __name__ == "__main__":
    exit_code = 0
    try:
        summary, policy_path = main()
        record.finish("completed", summary=summary, policy_path=policy_path)
    except BaseException:
        import traceback
        traceback.print_exc()
        sys.stderr.flush()
        record.finish("failed", error=traceback.format_exc())
        exit_code = 1
    finally:
        app.close(exit_code=exit_code)
