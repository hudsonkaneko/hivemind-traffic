# NVIDIA AV Simulation Research Notebook

> Living notebook initialized 2026-09-11. Status labels: **Confirmed** = supported by a cited public source; **Inference** = reasoned connection to validate; **Unconfirmed** = no authoritative public match found. Keep dated updates and prefer primary sources.

## Project Goal

Build an interactive highway simulation where autonomous vehicles can communicate and coordinate their behavior. Eventually compare independent AV behavior against coordinated/hivemind AV behavior using metrics such as traffic throughput, average travel time, congestion, braking events, lane changes, collisions, and overall traffic efficiency.

## Possible Architecture

```text
Real-world data / images
→ NuRec / world reconstruction
→ OpenUSD environment
→ Isaac Sim or AlpaSim
→ PhysX / Newton / Warp physics
→ Isaac Lab or AlpaGym training
→ AV policy / Alpamayo
→ multi-agent coordination
→ traffic experiments
```

Working interpretation: use **Isaac Sim** when interactive 3D authoring, sensors, general robotics, and higher-fidelity rigid-body simulation matter; investigate **AlpaSim + AlpaGym** for data-driven, closed-loop end-to-end driving research. The proposed chain is conceptual—not every arrow is a turnkey integration.

## Technology Notes

### NVIDIA Isaac Sim

- **What / does:** **Confirmed.** Open-source reference framework built on Omniverse libraries for robotics simulation, testing, synthetic data generation, and SIL/HIL evaluation in physically based environments.
- **How:** Ingests CAD, URDF, and real captures (including NuRec), assembles OpenUSD scenes, supplies RTX rendering, sensors, physics, and extensible Python/Kit workflows.
- **Connections:** Foundation for Isaac Lab; uses OpenUSD and Omniverse libraries; normally uses PhysX, with newer Newton interoperability; accepts NuRec scenes and can feed/augment data with Cosmos.
- **AV usefulness:** High for sensor simulation, scenario visualization, interactive control, and system validation; it is broader robotics infrastructure rather than an AV-specialized traffic simulator.
- **Hivemind fit:** High for the interactive sandbox and instrumentation; multi-vehicle communication, traffic logic, and coordination policy must be custom-built or integrated.
- **APIs/libraries:** Isaac Sim Python API, Kit extensions, `pxr`/USD, Replicator, OmniGraph, ROS 2 bridge, `omni.physx`.
- **Sources:** [Product overview](https://developer.nvidia.com/isaac/sim) · [Documentation](https://docs.isaacsim.omniverse.nvidia.com/)
- **Questions:** Vehicle dynamics fidelity and scale? Current traffic/vehicle samples? Best path for hundreds of communicating vehicles? Licensing/deployment constraints?

### Isaac Lab

- **What / does:** **Confirmed.** Open-source, GPU-accelerated framework on Isaac Sim for scalable robot learning, including reinforcement learning, imitation learning, and motion planning.
- **How:** Defines vectorized environments, tasks, observations, actions, rewards, domain randomization, and training integrations; parallelizes environments on GPU.
- **Connections:** Runs over Isaac Sim and can work with PhysX, Newton, Warp, and learning libraries such as RL-Games, RSL-RL, SKRL, or RLlib.
- **AV usefulness:** Medium. Useful for policy learning and controlled experiments, but examples and abstractions are primarily robotics-oriented rather than road-traffic-native.
- **Hivemind fit:** Medium-high if a custom multi-agent vehicle environment is built; attractive for vectorized ablations and coordination rewards.
- **APIs/libraries:** `isaaclab`, manager-based/direct environments, Gymnasium-style interfaces, RL library adapters.
- **Sources:** [Isaac Lab overview](https://developer.nvidia.com/isaac/lab) · [GitHub](https://github.com/isaac-sim/IsaacLab)
- **Questions:** Native multi-agent support maturity? Vehicle articulation templates? Scaling limits with cameras versus state-only observations?

### NVIDIA Alpamayo 2 / Alpamayo 2 Super

- **What / does:** **Confirmed as of 2026.** Alpamayo is an open AV platform/family; Alpamayo 2 Super is NVIDIA’s open reasoning vision-language-action driving foundation model, producing driving trajectories plus causal reasoning traces.
- **How:** Uses camera/context inputs to generate/plausibly score trajectories and reasoning; recipes support inference, fine-tuning, evaluation, and connection to closed-loop simulation/RL.
- **Connections:** Evaluated in AlpaSim/NuRec scenes; AlpaGym closes the RL loop; Cosmos and Physical AI datasets support data generation/curation.
- **AV usefulness:** Very high for end-to-end policy research; safety claims still require independent validation and it is not a complete production driving stack.
- **Hivemind fit:** Medium. Strong candidate for each ego policy or a baseline, but collective communication/planning is not established as a built-in feature.
- **APIs/libraries:** PyTorch/model weights, `alpamayo-recipes`, Hugging Face model assets, AlpaSim gRPC interfaces.
- **Sources:** [Official platform page](https://www.nvidia.com/en-us/solutions/autonomous-vehicles/alpamayo/) · [Official model/platform reference](https://www.nvidia.com/en-us/solutions/autonomous-vehicles/alpamayo/llm-info/) · [Recipes](https://github.com/NVlabs/alpamayo-recipes)
- **Questions:** Exact input/output contract and hardware needs for 2 Super? Multi-ego batching? License limits? How to add messages/shared latent state without invalidating pretrained behavior?

### AlpaSim

- **What / does:** **Confirmed.** Open-source, closed-loop AV research simulator emphasizing sensor fidelity, horizontal scalability, and hackability.
- **How:** Python microservices communicate through gRPC; runtime coordinates driver, renderer, traffic, controller/vehicle model, physics, logging, and external evaluation. Precise real-time physics is explicitly a non-goal.
- **Connections:** Uses NuRec/Neural Rendering Engine and can use OmniDreams/FlashDreams rendering; supplies rollouts to AlpaGym and Alpamayo policies; records AlpaSim Log (`.asl`) protobuf streams.
- **AV usefulness:** Very high for closed-loop end-to-end AV evaluation and scalable policy experiments.
- **Hivemind fit:** Potentially high because services and drivers are replaceable, but public support for many jointly controlled ego agents must be verified.
- **APIs/libraries:** Python packages, gRPC/protobuf, Hydra configuration, Wizard launcher, plugin entry points, `.asl` logs.
- **Sources:** [GitHub](https://github.com/NVlabs/alpasim) · [Design](https://github.com/NVlabs/alpasim/blob/main/docs/DESIGN.md) · [Tutorial](https://github.com/NVlabs/alpasim/blob/main/docs/TUTORIAL.md)
- **Questions:** Can multiple ego vehicles be policy-controlled concurrently? TrafficSim public status? Network-message simulation? Physics fidelity for collision/braking metrics? Required datasets/containers/GPUs?

### AlpaGym

- **What / does:** **Confirmed.** Early, actively developed RL framework for end-to-end autonomous-driving policies.
- **How:** Runs policy rollouts closed-loop in AlpaSim, computes rewards, and trains from consequences; Cosmos-RL provides distributed rollout/training orchestration, with gRPC connecting the runtime to AlpaSim.
- **Connections:** AlpaSim is the environment; Alpamayo is a supported policy family; Cosmos-RL is the trainer/orchestrator.
- **AV usefulness:** Very high for closed-loop policy improvement and reward-based safety/progress experiments.
- **Hivemind fit:** High in principle for shared rewards or centralized/decentralized policies, but multi-agent APIs are not yet confirmed.
- **APIs/libraries:** `alpagym_host`, runtime/policy/reward packages, Hydra, Cosmos-RL, AlpaSim gRPC, W&B; current public docs mention GRPO workflows.
- **Sources:** [GitHub and architecture](https://github.com/NVlabs/alpagym)
- **Questions:** Multi-agent rollout semantics? Custom reward hooks for throughput/congestion? Support for Alpamayo 2 Super? Compute budget and determinism?

### Cosmos / Cosmos-Dreams

- **What / does:** **Confirmed with naming caveat.** Cosmos is NVIDIA’s world-foundation-model platform (Reason, Predict, Transfer and newer releases) for physical-AI reasoning, video/world generation, and data augmentation. “Cosmos-Dreams” is not clearly one single public product name; public projects include Cosmos-Drive-Dreams and GR00T-Dreams, while AlpaSim documents **OmniDreams** via FlashDreams.
- **How:** Generative models predict future visual states, transform structured simulation inputs into photorealistic video, reason about physical scenes, or synthesize diverse training data.
- **Connections:** Complements simulation rather than replacing deterministic physics; augments Isaac Sim/NuRec/CARLA pipelines; OmniDreams can serve as an AlpaSim renderer.
- **AV usefulness:** High for sensor realism, long-tail scenario/data generation, reasoning, and learned rendering; lower for ground-truth traffic dynamics and safety metrics.
- **Hivemind fit:** Medium as a visual/data layer; coordination behavior should be measured in a deterministic state/physics loop.
- **APIs/libraries:** Cosmos models/cookbook, PyTorch, Hugging Face/NGC assets, Cosmos-Transfer/Reason/Predict; FlashDreams/OmniDreams interfaces require more study.
- **Sources:** [Cosmos overview](https://www.nvidia.com/en-us/ai/cosmos/) · [AV workflow](https://developer.nvidia.com/blog/simplify-end-to-end-autonomous-vehicle-development-with-new-nvidia-cosmos-world-foundation-models/) · [Cosmos-Drive-Dreams paper](https://arxiv.org/abs/2506.09042) · [AlpaSim design](https://github.com/NVlabs/alpasim/blob/main/docs/DESIGN.md)
- **Questions:** Which “Dreams” component did the mentor mean? Is OmniDreams publicly deployable? State consistency, controllability, latency, and ground-truth access?

### NuRec

- **What / does:** **Confirmed.** Omniverse NuRec is a set of agent-friendly neural-reconstruction and 3D Gaussian-splatting libraries that turns camera/lidar captures into interactive OpenUSD simulation scenes.
- **How:** Standardizes sensor inputs with NCore, reconstructs geometry/appearance and trajectories, packages results in USD, and renders novel views using Gaussian-based methods; NuRec Fixer can address novel-view artifacts.
- **Connections:** Integrates with Isaac Sim, AlpaSim, and CARLA; provides reconstructed worlds to which Cosmos can add variation.
- **AV usefulness:** Very high for replayable real-road digital twins and sensor-view generation; reconstructed appearance is not itself a full physics/traffic model.
- **Hivemind fit:** High for realistic highway test environments; dynamic actors, lanes/semantics, collision shapes, and coordinated traffic remain separate work.
- **APIs/libraries:** NuRec/NCore, 3D Gaussian splatting/`gsplat`, USD packaging, neural renderer APIs, NGC containers.
- **Sources:** [NuRec overview](https://developer.nvidia.com/omniverse/nurec) · [AV pipeline article](https://developer.nvidia.com/blog/accelerating-av-simulation-with-neural-reconstruction-and-world-foundation-models/)
- **Questions:** Minimal capture rig? Smartphone versus fleet quality? Semantic map/collision extraction? Dataset licensing and disk/GPU costs?

### Newton Physics

- **What / does:** **Confirmed.** Open-source, extensible, differentiable physics engine built on Warp and OpenUSD, developed by NVIDIA, Google DeepMind, and Disney Research and managed by the Linux Foundation.
- **How:** Offers a modular architecture with pluggable rigid/deformable/multiphysics solvers and automatic differentiation through Warp; integrates with Isaac Sim/Lab.
- **Connections:** Warp is its GPU kernel foundation; OpenUSD describes worlds; Isaac Sim/Lab are host simulation/learning frameworks. It is a sibling/alternative backend to PhysX in supported workflows.
- **AV usefulness:** Medium today: extensibility and differentiability are valuable, but public emphasis is robot contact/manipulation/locomotion, not validated tire/road/vehicle dynamics.
- **Hivemind fit:** Medium-low initially; relevant only if custom differentiable vehicle/traffic physics materially improves research.
- **APIs/libraries:** Newton Python API, solver/model/control/sensor interfaces, Warp, MJWarp/Kamino components, USD import.
- **Sources:** [Newton overview](https://developer.nvidia.com/newton-physics) · [Announcement/architecture](https://developer.nvidia.com/blog/announcing-newton-an-open-source-physics-engine-for-robotics-simulation/)
- **Questions:** Vehicle and tire models? Deterministic stepping? Isaac Sim production support level? Is writing a solver worth the complexity versus PhysX vehicles?

### NVIDIA Warp

- **What / does:** **Confirmed.** Open-source Python framework for JIT-compiling GPU/CPU kernels for simulation, geometry, data generation, and differentiable computation.
- **How:** Decorated Python kernels operate in parallel and compile to efficient CUDA/CPU code; supports arrays, meshes/spatial queries, simulation primitives, and reverse-mode automatic differentiation.
- **Connections:** Foundation of Newton; usable inside Omniverse/Isaac workflows and interoperable with PyTorch, JAX, and NumPy.
- **AV usefulness:** Medium as a low-level accelerator for custom traffic, sensor, cost, communication, or physics kernels—not a ready AV simulator.
- **Hivemind fit:** High if coordination computations or thousands of vehicle interactions need custom GPU acceleration; otherwise premature optimization.
- **APIs/libraries:** `warp`/`wp`, kernels, arrays, launch, tape/autodiff, mesh/BVH, PyTorch/JAX interop.
- **Sources:** [Warp overview](https://developer.nvidia.com/warp) · [GitHub](https://github.com/NVIDIA/warp) · [Computational-physics example](https://developer.nvidia.com/blog/build-accelerated-differentiable-computational-physics-code-for-ai-with-nvidia-warp/)
- **Questions:** Best data layout for traffic agents? Stable integration with Isaac Sim timeline? Determinism and profiling? Existing vehicle/traffic kernels?

### PhysX

- **What / does:** **Confirmed.** NVIDIA’s real-time physics SDK and the primary physics engine behind Omniverse simulation, covering rigid bodies, collisions, joints/articulations, vehicles, and other features.
- **How:** `omni.physx` translates USD Physics/PhysX schema data into PhysX, steps simulation, then writes results back to USD or the faster Fabric cache.
- **Connections:** Default physics path for Isaac Sim/Omniverse; represented through OpenUSD schemas; may coexist with or be compared against Newton.
- **AV usefulness:** High for collisions and conventional real-time vehicle dynamics, subject to validating the vehicle model against project fidelity needs.
- **Hivemind fit:** High as the initial deterministic physical substrate; keep coordination logic above it.
- **APIs/libraries:** PhysX SDK, `omni.physx`, `UsdPhysics`, `PhysxSchema`, vehicle SDK/APIs, Fabric.
- **Sources:** [PhysX SDK](https://developer.nvidia.com/physx-sdk) · [Omniverse physics architecture](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/) · [`omni.physx`](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/110.1/extensions/runtime/source/omni.physx/docs/index.html)
- **Questions:** Isaac Sim vehicle samples/API stability? Tire friction calibration? Collision/event telemetry? Performance at target vehicle count?

### PhysicsNeMo

- **What / does:** **Confirmed.** Open-source deep-learning framework for building, training, fine-tuning, and deploying Physics-AI/scientific-ML models, including neural operators, GNNs, transformers, PINNs, and diffusion models.
- **How:** Combines data and physics constraints in scalable PyTorch pipelines, with model/datapipe/distributed/mesh/symbolic modules and optimized GPU operators.
- **Connections:** Can learn surrogate models from simulation/engineering data and uses some Warp-accelerated operators; it is not Isaac Sim’s physics engine.
- **AV usefulness:** Low-medium for the first sandbox; potentially useful later for learned traffic-flow, aerodynamics, tire, or surrogate dynamics models.
- **Hivemind fit:** Medium for macroscopic traffic prediction or a learned fast surrogate, but not a ready multi-agent coordination framework.
- **APIs/libraries:** `physicsnemo.models`, `.datapipes`, `.distributed`, `.sym`, `.mesh`, `.diffusion`, PyTorch/ONNX.
- **Sources:** [Overview](https://docs.nvidia.com/physicsnemo/25.08/overview.html) · [Current API docs](https://docs.nvidia.com/physicsnemo/latest/physicsnemo/index.html)
- **Questions:** Relevant traffic/vehicle examples? Coupling with Isaac/AlpaSim? Can it learn a stable surrogate without obscuring experimental causality?

### OpenUSD

- **What / does:** **Confirmed.** Open framework and APIs for composing, layering, referencing, and exchanging complex 3D scene descriptions; it is a scene model, not a renderer or physics engine.
- **How:** Stores scenes as stages, layers, prims, attributes, relationships, variants, and schemas; Hydra connects scene data to renderers. USD Physics/PhysX schemas attach simulation meaning.
- **Connections:** Common environment/asset representation across Omniverse, Isaac Sim, NuRec, Newton, and PhysX integration.
- **AV usefulness:** High for reusable roads, vehicles, sensors, labels, and scenario variants; traffic behavior needs additional runtime systems.
- **Hivemind fit:** High as the stable world/experiment description layer, including custom schemas or metadata for infrastructure and communication topology.
- **APIs/libraries:** `pxr.Usd`, `UsdGeom`, `UsdPhysics`, `UsdShade`, `Sdf`, `Gf`, Hydra; USD Composer/Exchange tools.
- **Sources:** [OpenUSD](https://openusd.org/) · [Omniverse USD docs](https://docs.omniverse.nvidia.com/usd/latest/)
- **Questions:** Schema for lanes/routes/signals/V2X? Runtime edits through Fabric versus authored USD? Asset versioning strategy?

### Omniverse Libraries

- **What / does:** **Confirmed.** Modular SDKs and libraries for building OpenUSD-native industrial and physical-AI applications; Isaac Sim is now positioned as a reference framework built from these libraries.
- **How:** Kit application framework plus extensions provide USD, RTX rendering, Fabric, OmniGraph, physics, asset exchange, UI, sensors, and application services.
- **Connections:** Underpins Isaac Sim; hosts PhysX and OpenUSD workflows; integrates NuRec and synthetic-data tooling.
- **AV usefulness:** High when building a tailored simulator/application rather than accepting Isaac Sim’s full reference app.
- **Hivemind fit:** High long-term for a purpose-built interactive UI and runtime; higher engineering cost than extending Isaac Sim first.
- **APIs/libraries:** Kit SDK/extensions, Carbonite, OmniGraph, Fabric, RTX Renderer, `omni.usd`, `omni.physx`, OpenUSD Exchange SDK.
- **Sources:** [Omniverse for developers](https://developer.nvidia.com/omniverse) · [Developer guide](https://docs.omniverse.nvidia.com/dev-guide/latest/)
- **Questions:** Which libraries are redistributable? When to graduate from Isaac Sim extension to custom Kit app? Packaging and version compatibility?

### Astra / Unconfirmed NVIDIA Research

- **What / does:** **Unconfirmed.** Searches of public NVIDIA product, developer, research, and repository material found no authoritative current NVIDIA technology named “Astra” matching “create simulations from pictures and write/customize simulation solvers.” Do not treat the mentor’s name or combined description as verified.
- **How (possible decomposition, not identification):** **Inference:** NuRec covers reconstruction from captured images/lidar; Cosmos/Drive-Dreams or OmniDreams covers generative visual simulation; Newton provides pluggable/custom solvers; Warp lets developers write accelerated differentiable kernels/solvers; PhysicsNeMo builds learned physics surrogates. Alpamayo/AlpaSim are the AV policy/simulator context. One remembered name may conflate several layers.
- **Connections:** No confirmed “Astra” integration graph exists. The candidate public stack is NuRec → OpenUSD → Isaac Sim/AlpaSim, with Cosmos for visual variation and PhysX or Newton/Warp for dynamics.
- **AV usefulness:** Unknown for “Astra”; high collectively for the confirmed candidate technologies.
- **Hivemind fit:** Unknown. Do not plan around it until a spelling, slide, repo, paper, or contact reference is obtained.
- **APIs/libraries:** None confirmed under this name.
- **Sources:** Negative search is not proof of nonexistence. Compare [NuRec](https://developer.nvidia.com/omniverse/nurec), [Cosmos](https://www.nvidia.com/en-us/ai/cosmos/), [Newton](https://developer.nvidia.com/newton-physics), [Warp](https://developer.nvidia.com/warp), [PhysicsNeMo](https://docs.nvidia.com/physicsnemo/latest/), [AlpaSim](https://github.com/NVlabs/alpasim), and [Alpamayo](https://www.nvidia.com/en-us/solutions/autonomous-vehicles/alpamayo/).
- **Questions:** Ask mentor for spelling, team, demo date, screenshot/link, whether it was internal, and whether “solver” meant a numerical physics solver, a learned world model, or an agent writing simulator code.

## Astra Investigation

### Confirmed public matches

| Claimed capability | Closest confirmed public technology | Confidence |
|---|---|---|
| Create an interactive scene from pictures/sensor captures | NuRec neural reconstruction into OpenUSD | High |
| Generate or transform simulated visual worlds | Cosmos; Cosmos-Drive-Dreams; OmniDreams renderer mentioned by AlpaSim | High on family, medium on intended name |
| Write/customize simulation solvers | Newton’s pluggable solvers and Warp kernels/autodiff | High |
| Learn physics approximations/surrogates | PhysicsNeMo | High |
| Apply this to AV closed-loop simulation | AlpaSim + Alpamayo + AlpaGym | High |
| Agentic manipulation of simulation workflows | Isaac Sim/NuRec describe agent-ready or agent-friendly workflows; a specific “Astra” agent product was not found | Low |

### Current hypothesis

**Speculation only:** “Astra” may be a misheard/internal codename, or shorthand for an agentic workflow combining image-to-world reconstruction (NuRec/Cosmos) with generated/custom physics code (Warp/Newton/PhysicsNeMo). There is not enough public evidence to select one product. Maintain this as an open lead, not an architectural dependency.

### Next verification actions

- Obtain a spelling, URL, slide title, team name, or demo recording from the mentor.
- Search NVIDIA Research/GTC session catalogs and GitHub again using that anchor.
- Determine whether “pictures” meant monocular photos, multi-camera video, lidar, or text/image prompts.
- Determine whether “solver” meant numerical PDE/physics code, rigid-body dynamics, policy planning, or an AI agent that edits simulator code.

## Experiments

Use one subsection per run and place code/artifacts under `experiments/<YYYY-MM-DD>-<slug>/`; put small reusable snippets in `examples/` and downloaded papers/source snapshots or bibliographic notes in `sources/`.

### Experiment template

```markdown
### YYYY-MM-DD — Experiment name
- Hypothesis:
- Simulator/version/hardware:
- Scene and seed:
- Independent or coordinated policy:
- Communication model (range, latency, bandwidth, packet loss):
- Variables and controls:
- Metrics: throughput, travel time, congestion, braking, lane changes, collisions, efficiency
- Code/config/artifacts:
- Results:
- Problems/limitations:
- Conclusion:
- Next experiment:
```

## Cross-Cutting Research Questions

- What is the smallest viable stack for a deterministic, interactive, multi-vehicle highway baseline?
- Which simulator exposes simultaneous control of many ego vehicles and reproducible state-level metrics?
- How will independent and coordinated policies receive equivalent sensing and compute budgets?
- What communication assumptions (latency, range, loss, topology, trust) make “hivemind” scientifically meaningful?
- Which components require NVIDIA GPU hardware, Linux, containers, restricted datasets, or noncommercial terms?
- How will stochastic/rendering differences be separated from coordination effects?

## Decision Log

| Date | Decision / observation | Evidence | Revisit when |
|---|---|---|---|
| 2026-09-11 | Begin with research only; no simulator implementation selected. | This notebook | Requirements and hardware are inventoried |
| 2026-09-11 | Treat “Astra” as unconfirmed and non-blocking. | No authoritative matching public source located; adjacent tools documented above | Mentor supplies an identifying anchor |

## Source Hygiene

- Record access date and version/commit for implementation-critical sources.
- Prefer NVIDIA docs, official GitHub repositories, model cards, papers, and release notes over summaries.
- Copy only material whose license permits it; otherwise store a citation and notes in `sources/`.
- Flag statements as **Confirmed**, **Inference**, or **Unconfirmed** when certainty matters.
