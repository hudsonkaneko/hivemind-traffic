# Hivemind Traffic — Engineering &amp; Learning Guide

[Google document](https://docs.google.com/document/d/1qeYucQnP4nznmLNJrlUz1hCIcY7KOigRPuhBVH-Zo0c/edit) · [Update workflow](learning-guide-sync.md#chat-managed-markdown-mirror)

This is the complete GitHub-readable copy of the Google guide. Milestone entries retain their original dates and claims; they are historical evidence, not a substitute for the [current implementation notes](README.md). Updates made through the project chat refresh this file after native readback. This is chat-managed synchronization, not a background Google Docs watcher.

## Contents

- [00 Start here](#docs-t-0)
- [01 Repository boundaries](#docs-t-w2twc8yzdth8)
- [02 Dependencies and portability](#docs-t-nc1hv1juyfll)
- [03 SUMO and TraCI](#docs-t-6z084o1j9bo2)
- [04 Isaac and lidar decision](#docs-t-xx03z3lvew1x)
- [05 PettingZoo and curved traffic](#docs-t-1aizbsu5kpcr)
- [06 Scripted and random baselines](#docs-t-odx5w8vvnbs9)
- [07 Shared PPO](#docs-t-2oadx9o9j7v9)
- [08 Awareness and safety refinement](#docs-t-kzvpcax50f94)
- [09 Static RTX lidar](#docs-t-nbbmkv8ek2tk)
- [10 Moving traffic in OpenUSD](#docs-t-utnb9vbixqfp)
- [11 Repository transfer](#docs-t-i3zm9qaea3qe)
- [12 GUI lidar fix](#docs-t-pmiduualw22c)
- [13 ovrtx architecture decision](#docs-t-kcaya7u45f24)
- [14 Shared exporter and ovrtx](#docs-t-7yw661dv42iu)
- [15 Interview and study workbook](#docs-t-fslvdlft9ot)
- [16 Moving lidar geometry and timing](#docs-t-cchxbirgtixb)
- [17 Curved replay lidar validation](#docs-t-2ctsja7co7rg)
- [18 Ground truth vs lidar perception](#docs-t-c9rggs6xex7k)
- [19 Lidar failure diagnostics](#docs-t-jz0vf0ad2a8r)
- [20 Two-car lidar identity isolation](#docs-t-c8cnwa4eowg)
- [21 Portable demonstration launcher](#docs-t-2ivlli89tzj8)
- [22 Lidar identity map-source investigation](#docs-t-mum52kmdfd9i)
- [23 Initial visibility and lidar registration](#docs-t-df6w7wqf3jkt)
- [24 Explicit visibility diagnostic](#docs-t-tcee1jwa6vj6)
- [25 Hybrid roadmap and GitHub workflow](#docs-t-yt6tixhcaw4x)
- [26 Physics-driven car foundation](#docs-t-n7c34hqtzn8v)
- [27 Dynamics, resets and Isaac Lab](#docs-t-egh5rmb516m4)
- [28 Scripted lane following](#docs-t-70inmvd9orgv)
- [29 Physics car and LiDAR view](#docs-t-rb5nks6ogqck)
- [30 Wheel orientation fix](#docs-t-ivzd9eyrxgjj)
- [31 Adaptive obstacle bypass](#docs-t-cbgq2dwnrach)
- [32 Demo readiness and profiling](#docs-t-9wkfs52ky6df)
- [33 Live preview timing](#docs-t-e9ctk5gfkhg)
- [34 Loop traffic showcase](#docs-t-9ah0315nta8b)
- [35 Showcase performance](#docs-t-dwsxfv73kedy)
- [36 Responsive showcase](#docs-t-jndashuyzqn1)
- [37 Live showcase controls](#docs-t-q3vcdoqqkcjq)
- [38 Project skills in GitHub](#docs-t-124coz3q4xks)
- [39 Clean-clone readiness](#docs-t-mriyksl2h83o)
- [40 Showcase validation](#docs-t-lz1t1b5s17js)
- [41 Isaac Lab lane task](#docs-t-k82ts3ro3qz9)

<a id="docs-t-0"></a>

## 00 Start here

GitHub Markdown copy: [`https://github.com/hudsonkaneko/hivemind-traffic/blob/codex/semantic-lidar-fusion/documentation/engineering-guide.md`](<https://github.com/hudsonkaneko/hivemind-traffic/blob/codex/semantic-lidar-fusion/documentation/engineering-guide.md>)

Updates requested through the Highway Sim project chat keep this Google guide and its GitHub Markdown copy aligned. Changes made directly outside the chat are reconciled on the next requested sync; this is not a background watcher. Historical milestone entries keep their recorded dates and results.

<a id="docs-h-5x4oup46pnt6"></a>

### **Choose your reading path**

**Understand the system:** start with the separate Hivemind Traffic — Concepts &amp; Recruiter Guide document. It defines the tools and separates live control, training and 3D replay.

**Run a demo:** complete the one-time local setup in tab 21, then use the portable commands in each numbered tab with its expected results.

**Study implementation: tabs 01–24 preserve the earlier setup, learning, rendering and LiDAR investigations. Start with tabs 25–29 for the active hybrid roadmap, GitHub workflow, physical-car evidence, Isaac Lab connection, scripted lane following and the visible physics/LiDAR demo; repository documentation tracks the complete implementation.**

**Prepare for an interview:** read the separate concepts guide, then tab 15 in this document. Explain one decision, one measured result and one limitation.

**Visual key:** blue-gray shaded text is a command to copy; pale-yellow bold terms are vocabulary to explain. Recruiter sections use a teal accent. Labels such as CURRENT and PLANNED carry meaning even without color.

Commands remain plain, copyable text. Visual line wrapping does not add a terminal line break. The styling is a code-style treatment, not syntax highlighting or an executable block.

<a id="docs-h-5u92w78nsazh"></a>

### **Purpose and scope**

This is a learning companion to the implementation, not a claim that every planned capability already exists. Each change tab explains the problem, the mechanism, the evidence, and a small task you can repeat yourself. Historical changes are grouped by meaningful engineering step; several steps share one commit.

<a id="docs-h-eubp4hkjgseb"></a>

### **The system in one sentence**

**SUMO models scalable traffic and remains the preserved traffic demos’ movement engine; TraCI connects Python to it. PettingZoo exposes multi-agent interactions and PPO learns policies. A separate Isaac/PhysX car now follows a known map with explicit driver commands and LiDAR emergency braking. The planned hybrid stack retains SUMO background traffic; ovrtx remains a separate rendering path.**

<a id="docs-h-ur86s2bxltx1"></a>

### **What works and what does not yet**

Current: preserved SUMO/PettingZoo/PPO and two-car V2V baselines coexist with a separate physical-car fixture, verified Lab reset/step interface, scripted lane following, and a visible curved-road LiDAR-braking demo. The physical demo is one scripted low-speed car, not learned or real-time ready. Hybrid traffic, full sensing/reset and endurance gates remain open; historical scene-reference warning failures remain recorded in tabs 27–28.

<a id="docs-h-ii0q887seb2g"></a>

### **How to use this guide**

Read tabs 01–08 for simulation and learning, then 09–14 for 3D and sensors. Use tab 15 to rehearse explanations. Open a terminal in your clone’s repository root and activate the traffic environment as explained in tab 21. The shared commands use python; use python3 if that is your interpreter’s name. Local checkpoints and generated outputs are intentionally not shipped in Git.

<a id="docs-h-adhcyfpg9ond"></a>

### **Keeping it useful**

For every future meaningful change, add a new tab with its purpose, changed files, technology, alternatives, reproduction steps, evidence, limitations, and commit reference. Preserve your personal notes. Treat untested work as untested, and distinguish measured results from plans.

<a id="docs-h-p50o311ei3"></a>

### **Sources and code**

[Repository](<https://github.com/hudsonkaneko/hivemind-traffic>)

<a id="docs-h-7yczhxbb1480"></a>

### **Portable demonstration**

After the setup in tab 21, use Command Prompt, PowerShell or a supported Linux shell from the repository root. Paste each command paragraph as one complete line; visual wrapping is fine. Do not copy a terminal prompt or add continuation characters. Run one demo at a time.

```text
python -m hivemind demo sumo
```

Quick demo: a live 2D curved-road episode with two controlled vehicles and background traffic. This runs SUMO, TraCI and PettingZoo with a scripted policy; it is not training.

For trained PPO use tab 07 or 08; for 3D replay use tab 10; for static lidar use tab 09; for an ovrtx image use tab 14. Other tabs below include their own demonstration commands.

These commands run the current checkout, not an automatic restoration of historical commits. Representative demos are labeled. Configure your own runtimes and a trusted compatible checkpoint in hivemind.local.json. Dependencies, checkpoints and recordings are not all included in a fresh GitHub clone.

Let finite demos finish normally; rerun to watch again. SUMO rollout windows close at episode end. Add --delay 200 to the SUMO demo command to slow it down. Isaac can take a while to initialize and closes after its frame budget; frame counts are not guaranteed wall-clock seconds. If stuck, use Ctrl+C in the terminal, then close the remaining window.

Output caution: rollouts create result folders; sensor validation creates run folders. The 3D viewer refreshes shared generated replay layers, preview.png, sample scans and replay\_report.json in outputs\\sumo\_replay. ovrtx rendering refreshes its generated image/manifest and scene files. Source code and trained weights are not changed by these demo commands.

Verification: launcher arguments, local input files and the refined checkpoint hash were checked while adding these notes. Every GUI demo was not rerun for this documentation update. Historical measurements above remain historical.

<a id="docs-h-odhzyfmyeiyn"></a>

### **Portable command reference**

From any supported clone’s root, after local dependency setup and environment activation (see tab 21):

```text
python -m hivemind demo sumo
```

```text
python -m hivemind demo trained
```

```text
python -m hivemind demo lidar
```

```text
python -m hivemind demo replay
```

```text
python -m hivemind demo ovrtx
```

The meaning and limitations of the demo above are unchanged. Tab 21 explains the ignored local configuration, checkpoint/recording prerequisites and --check. These commands do not make unsupported hardware compatible. Commands now use portable entry points; complete the local setup in tab 21 first.

<a id="docs-t-w2twc8yzdth8"></a>

## 01 Repository boundaries

<a id="docs-h-1no8ku2cm9st"></a>

### **Change and purpose**

The repository was initialized and reduced to the requested scaffold rather than importing unrelated earlier work. Commits a308dbe and 6d42287 establish a clean starting point. The aim is one understandable project with clear ownership.

<a id="docs-h-sljnz49aoeec"></a>

### **Technology and mechanism**

scenarios/ contains networks, routes, and **SUMO** configurations. traffic/ owns simulator access. environments/ defines the learning interface. policies/ contains networks and policies. usd/ contains scene conventions and assets. visualization/ owns viewer integrations. experiments/ contains training/evaluation entry points and settings. tests/ verifies behavior; results/ records experiment outputs. Supporting scripts/, documentation/, outputs/, and logs/ were added for integration work.

<a id="docs-h-k3r4y0lipc9e"></a>

### **Why this design**

Separating responsibilities lets you replace a renderer without rewriting the traffic environment. Git records snapshots and differences, so v1/v2/v3 copies of source folders are unnecessary. Named run folders still matter for experiments: they represent different seeds or settings, not alternative copies of the source tree.

<a id="docs-h-ce7uuneqelqz"></a>

### **Try it yourself**

Run git log --oneline, then git show 6d42287 --stat. Pick a file and explain why it belongs in its directory. Before a commit, inspect git diff and ensure generated scans, virtual environments, and private credentials are not included.

<a id="docs-h-boe4051z83lm"></a>

### **Recruiter explanation**

“I organized the project around simulator, environment, policy, and rendering boundaries so components can evolve independently and changes remain traceable.” Explain an actual boundary rather than merely listing folder names.

<a id="docs-h-bj6by6xrs0g2"></a>

### **Sources and code**

[Scaffold change](<https://github.com/hudsonkaneko/hivemind-traffic/commit/6d42287>)

<a id="docs-h-zg9ek68nfoij"></a>

### **Portable demonstration**

Repository demonstration—not a traffic simulation. These commands show today’s canonical project folder and recent version history without changing the checkout.

```text
git ls-tree --name-only HEAD
```

```text
git log --oneline -8
```

What to show: point out scenarios, traffic, environments, policies and visualization. Explain that Git records changes while separate directories express responsibilities. The folder contains later work too; it is not the original empty scaffold.

<a id="docs-t-nc1hv1juyfll"></a>

## 02 Dependencies and portability

<a id="docs-h-xnhql4wqpmrx"></a>

### **Change and purpose**

Commit 687e785 added the Python/**SUMO** development setup. Portable paths and separate runtimes make the project usable beyond one computer.

<a id="docs-h-k8oupugo2zvk"></a>

### **Technology and mechanism**

A Python virtual environment isolates traffic and learning packages. SUMO is a simulator executable; **TraCI** is its Python control interface. **PettingZoo** defines the multi-agent API; PyTorch supplies tensors, neural networks, and automatic differentiation. The later Isaac installation uses its own bundled Python, while **ovrtx** has a separate virtual environment.

<a id="docs-h-so65ygu5u9kt"></a>

### **Why separate runtimes**

Isaac's bundled packages and Python version should not be mixed casually with the training environment. Separate environments reduce dependency conflicts. Project assets should be resolved relative to the repository or supplied as arguments. SUMO discovery can use an explicit binary, PATH, or SUMO\_HOME; a machine-specific fallback is convenience, not a universal requirement.

<a id="docs-h-lqar2s9j34a2"></a>

### **Try it yourself**

From the repository root: python -m pytest

Inspect the repository setup instructions and dependency files before recreating an environment. Activate the traffic environment for direct python commands. The demo launcher selects traffic, Isaac or ovrtx runtimes separately; configure isaac\_root in hivemind.local.json rather than embedding an installation path in commands.

<a id="docs-h-1rhr2c20l677"></a>

### **Limit and explanation**

A working local environment is not proof of compatibility with every operating system or driver. Record versions when reproducing results. Recruiter phrasing: “I isolated the learning and rendering runtimes and kept project asset paths configurable to avoid tying the repository to my workstation.”

<a id="docs-h-j0tz883y1wvx"></a>

### **Sources and code**

[Dependency setup](<https://github.com/hudsonkaneko/hivemind-traffic/commit/687e785>)

<a id="docs-h-4xnqpi3xd4py"></a>

### **Portable demonstration**

Quick dependency check, followed by a visible current simulation. This illustrates the setup milestone using today’s installed environment.

```text
python -c "import traci, pettingzoo, torch; print('TraCI, PettingZoo and PyTorch imports OK')"
```

```text
python -m hivemind demo sumo
```

Expected: the import confirmation appears, then SUMO opens with moving traffic. This checks basic availability, not every dependency or cross-machine compatibility. Activate the traffic environment first (tab 21). The demo launcher selects the separate Isaac and ovrtx runtimes when needed.

<a id="docs-h-j5aof32ici54"></a>

### **Portable command reference**

From any supported clone’s root, after local dependency setup and environment activation (see tab 21):

```text
python -m hivemind demo sumo --check
```

The meaning and limitations of the demo above are unchanged. Tab 21 explains the ignored local configuration, **checkpoint**/recording prerequisites and --check. These commands do not make unsupported hardware compatible. Commands now use portable entry points; complete the local setup in tab 21 first.

<a id="docs-t-6z084o1j9bo2"></a>

## 03 SUMO and TraCI

<a id="docs-h-q9twkvp45jo9"></a>

### **Change and purpose**

Commit d5a82fd created a deterministic single-vehicle **baseline**. Starting small made it possible to verify that a vehicle loads, moves, and produces sensible telemetry before adding learning.

<a id="docs-h-9umgsh4h5lam"></a>

### **Technology and mechanism**

**SUMO** loads road geometry, routes, and vehicle definitions. A **TraCI** connection lets Python read state, request controls, and advance the simulation. The backend owns connection startup and cleanup. The control interval is 0.2 seconds: a command changes the requested behavior, then simulationStep advances the world.

<a id="docs-h-2zslimuomr8o"></a>

### **Why start here**

If the simulator or timing is wrong, a training algorithm can optimize the wrong problem. A fixed-seed, non-learning run isolates simulator integration from policy behavior. SUMO handles lane geometry and road following; the policy is not learning how to turn a steering wheel.

<a id="docs-h-oon82fbrckvf"></a>

### **Try it yourself**

Inspect scenarios/single\_vehicle/ and traffic/sumo\_backend.py. Find where the configuration is loaded, where a speed request is issued, where time advances, and where the connection closes. Repeat a run with the same settings and compare timestamps and vehicle positions.

<a id="docs-h-o0eu9cfk5ooh"></a>

### **Knowledge check and explanation**

Why should a control loop read a consistent state and advance time exactly once per joint action? Advancing once per vehicle would give different agents different effective worlds. Recruiter phrasing: “I built a deterministic simulator backend first so later learning results could be attributed to policy choices rather than an inconsistent control loop.”

<a id="docs-h-xpledhnhx1qq"></a>

### **Sources and code**

[Single-vehicle baseline](<https://github.com/hudsonkaneko/hivemind-traffic/commit/d5a82fd>)

[TraCI documentation](<https://sumo.dlr.de/docs/TraCI.html>)

<a id="docs-h-19w18f1o8kba"></a>

### **Portable demonstration**

First show the simple road in SUMO’s own GUI:

```text
sumo-gui -c scenarios/single_vehicle/scenario.sumocfg --start --delay 100
```

Expected: a single vehicle travels down the straight two-lane road. This exact single-vehicle command requires sumo-gui on PATH and uses SUMO’s built-in behavior; it does not run the Python TraCI control loop. Close the GUI after viewing. For the configured portable launcher’s current curved-road demo, use the command reference below.

Then demonstrate the actual Python/TraCI baseline in the terminal:

```text
python -m traffic.scripted_baseline
```

Expected: a non-learning simulation completes and saves telemetry under results\\single\_vehicle. This launcher is headless, so no second GUI is expected. Together these two demos show the road and the Python integration without claiming the GUI-only run used TraCI.

<a id="docs-h-h4czha9rhuhx"></a>

### **Portable command reference**

From any supported clone’s root, after local dependency setup and environment activation (see tab 21):

```text
python -m hivemind demo sumo
```

This is a representative current curved multi-vehicle demo, not the original single-vehicle TraCI test above. Tab 21 explains the ignored local configuration, **checkpoint**/recording prerequisites and --check. These commands do not make unsupported hardware compatible. Commands now use portable entry points; complete the local setup in tab 21 first.

<a id="docs-t-xx03z3lvew1x"></a>

## 04 Isaac and lidar decision

<a id="docs-h-sysu4aci5br6"></a>

### **Change and purpose**

Commit 9d9d07e recorded Isaac Sim with **RTX lidar** as a project destination. This changed the roadmap, not the current **PPO** input.

<a id="docs-h-h0dxnbhnyjqi"></a>

### **Technology and mechanism**

**SUMO** efficiently models traffic interactions. Isaac Sim can place sensors in a 3D scene and generate ray-traced returns. Isaac Lab can later support selected physics-based validation tasks. These tools answer different questions; adding lidar does not make a traffic simulator irrelevant.

<a id="docs-h-dqkd72p6wjbk"></a>

### **Why staged integration**

First establish traffic behavior, then validate scene coordinates and sensors, then define a sensor-based observation pipeline, and only then train or evaluate closed-loop perception-driven control. This avoids debugging learning, rendering, timing, and sensor geometry simultaneously.

<a id="docs-h-bmbdgzooagtp"></a>

### **What is still required**

A lidar-driven agent needs a defined scan representation, frame transforms, timestamps, filtering or encoding, a consistent observation size, and a policy trained for those inputs. Recorded motion with a sensor attached is not yet sensor-driven autonomy. A physical validation stage also needs one clearly defined owner of each vehicle's motion.

<a id="docs-h-5vj9y1udwo06"></a>

### **Try it yourself and explanation**

Draw the current data path: simulator state → policy → simulator action. Draw the future path separately: sensor returns → observation processing → policy → control. Mark which boxes exist today. Recruiter phrasing: “I retained SUMO for scalable traffic experiments and introduced Isaac for targeted sensor validation, with a staged plan for closing the loop.”

<a id="docs-h-qos17jmlwl9t"></a>

### **Sources and code**

[Architecture decision](<https://github.com/hudsonkaneko/hivemind-traffic/commit/9d9d07e>)

<a id="docs-h-b04u59ucbcz7"></a>

### **Portable demonstration**

Representative later demo of the sensor capability that this earlier architecture decision aimed toward; the decision itself did not yet implement lidar.

```text
python -m hivemind demo lidar
```

Expected: Isaac opens a static scene with boxes and lidar point visualization. There is no PPO, traffic training or sensor-driven steering here. The script saves a new smoke-test folder under outputs\\lidar\_smoke and closes after its frame budget.

<a id="docs-h-gskk0dhcu655"></a>

### **Portable command reference**

From any supported clone’s root, after local dependency setup and environment activation (see tab 21):

```text
python -m hivemind demo lidar
```

The meaning and limitations of the demo above are unchanged. Tab 21 explains the ignored local configuration, **checkpoint**/recording prerequisites and --check. These commands do not make unsupported hardware compatible. Commands now use portable entry points; complete the local setup in tab 21 first.

<a id="docs-t-1aizbsu5kpcr"></a>

## 05 PettingZoo and curved traffic

<a id="docs-h-63yrxty1mq47"></a>

### **Change and purpose**

Commit a322a11 introduced a curved two-agent scenario and a **PettingZoo** ParallelEnv. The road is approximately 1,074 meters long, with two controlled vehicles and three background vehicles.

<a id="docs-h-p7hn2xkxshup"></a>

### **Technology and mechanism**

ParallelEnv accepts a dictionary of actions for all active agents, applies the joint action, and advances **SUMO** once. It returns **observations**, rewards, terminations, truncations, and information. An arrival or collision is different from reaching the episode time limit. Agent lifecycle handling keeps those cases explicit.

<a id="docs-h-pu2psdrfn7dq"></a>

### **Controls and observations**

Actions use MultiDiscrete(\[3,3\]): decrease/hold/increase requested speed and right/stay/left lane request. Speed changes are −2, 0, or +2 m/s per control step. The refined environment uses 16 observation values: speed, lane, progress, front/rear traffic features in both lanes, and lane-change cooldown. This is tactical traffic control, not steering.

<a id="docs-h-mniihy2gcu1f"></a>

### **Why this interface**

A stable multi-agent API separates environment mechanics from the learning algorithm. The curved road tests geometry and neighbor handling, but SUMO still follows the road; curvature alone does not demonstrate learned lane keeping.

<a id="docs-h-x2qbmsi85y85"></a>

### **Try it yourself**

Read reset() and step() in environments/highway\_parallel\_env.py. Follow one action through the backend and identify the single simulation advance. Run the test suite. Explain why an agent disappears from the active set after finishing, and why a timeout is marked as truncation.

<a id="docs-h-nhgfqotrhnyc"></a>

### **Recruiter explanation**

“I wrapped the simulator in a simultaneous-action multi-agent interface, with explicit lifecycle and **reproducibility** rules, so policies can be tested against the same environment contract.”

<a id="docs-h-4shjafezokaq"></a>

### **Sources and code**

[Environment change](<https://github.com/hudsonkaneko/hivemind-traffic/commit/a322a11>)

[Parallel API](<https://pettingzoo.farama.org/api/parallel/>)

<a id="docs-h-1665sc22o8mi"></a>

### **Portable demonstration**

Live demonstration of the current PettingZoo wrapper and curved SUMO scenario:

```text
python -m hivemind demo sumo
```

Expected: two controlled vehicles and background traffic move along the curved road. A scripted policy provides joint actions through PettingZoo; **TraCI** advances SUMO. PettingZoo has no separate window. SUMO follows the road geometry—this is not learned steering or lane-line detection. A result folder is written under results\\rollouts.

<a id="docs-h-cnm11n8spust"></a>

### **Portable command reference**

From any supported clone’s root, after local dependency setup and environment activation (see tab 21):

```text
python -m hivemind demo sumo
```

The meaning and limitations of the demo above are unchanged. Tab 21 explains the ignored local configuration, checkpoint/recording prerequisites and --check. These commands do not make unsupported hardware compatible. Commands now use portable entry points; complete the local setup in tab 21 first.

<a id="docs-t-odx5w8vvnbs9"></a>

## 06 Scripted and random baselines

<a id="docs-h-yzh4un6e9r2t"></a>

### **Change and purpose**

Commit 91a1e41 added visible rollouts for scripted and seeded-random policies. A **baseline** is a reference behavior against which learning can be judged.

<a id="docs-h-tqn9clkneeta"></a>

### **Technology and mechanism**

The scripted policy applies explicit rules; the random policy samples actions. Neither trains a network. A rollout runs an existing policy, records metrics, and optionally opens **SUMO**'s GUI. A seed helps reproduce a run but does not make random behavior sensible.

<a id="docs-h-3vaooupf49z9"></a>

### **Try it yourself**

Run python -m hivemind demo sumo

Then run python -m hivemind demo random

Compare completion, collisions, safety overrides, **reward**, and steps—not just how smooth the animation looks.

<a id="docs-h-eh17bx52w0rf"></a>

### **Why random vehicles can look broken**

Independent random acceleration and lane requests can repeatedly conflict with traffic and safety rules. This is useful as a weak reference and stress test, not a target driving style. Overrides show where requested actions were changed; they do not necessarily mean a collision occurred.

<a id="docs-h-bie9zgn4z5fa"></a>

### **Fair-comparison limitation**

The random baseline samples declared actions without **PPO**'s dynamic **action mask**. Differences in override counts therefore reflect both learned choices and action filtering. Report that distinction. Recruiter phrasing: “I added non-learning references and telemetry so I could test whether learning improves useful outcomes, not merely produces an animation.”

<a id="docs-h-lwojoetfdo4m"></a>

### **Sources and code**

[Baseline rollouts](<https://github.com/hudsonkaneko/hivemind-traffic/commit/91a1e41>)

<a id="docs-h-jm50z4gkt8gh"></a>

### **Portable demonstration**

Run these separately, waiting for the first episode to finish:

```text
python -m hivemind demo sumo
```

```text
python -m hivemind demo random
```

Expected: scripted traffic is relatively purposeful; seeded random actions can cause repeated speed/lane requests, safety overrides and poor progress. Neither command trains a model. Both save evidence under results\\rollouts. The random run uses the same seed but is not guaranteed to look smooth; apparent indecision is part of this deliberately weak baseline.

<a id="docs-h-ps9305cknjh3"></a>

### **Portable command reference**

From any supported clone’s root, after local dependency setup and environment activation (see tab 21):

```text
python -m hivemind demo sumo
```

```text
python -m hivemind demo random
```

The meaning and limitations of the demo above are unchanged. Tab 21 explains the ignored local configuration, checkpoint/recording prerequisites and --check. These commands do not make unsupported hardware compatible. Commands now use portable entry points; complete the local setup in tab 21 first.

<a id="docs-t-2oadx9o9j7v9"></a>

## 07 Shared PPO

<a id="docs-h-xttnz2e0oifg"></a>

### **Change and purpose**

Commit 32dc5c6 introduced shared **PPO** training and **checkpoint** replay. PPO learns parameters from repeated interaction instead of relying entirely on hand-written driving decisions.

<a id="docs-h-io2jsy1ji41h"></a>

### **Technology and mechanism**

Both controlled vehicles use the same PyTorch actor-critic weights, but each acts on its own observation. Two hidden layers of 128 Tanh units feed separate categorical speed and lane heads; a scalar critic estimates value. Shared weights do not create communication between vehicles.

<a id="docs-h-36uv6tyq42q9"></a>

### **The learning cycle**

Collect **observations**, actions, action probabilities, rewards, and values. Estimate discounted returns and advantages. Compare new action probabilities with those stored during collection. PPO clips the policy ratio contribution to discourage overly large updates. Value loss trains the critic; an entropy term encourages exploration. Repeat with fresh experience.

<a id="docs-h-fe43p4g09dw7"></a>

### **Implementation choices**

The **baseline** uses Adam, learning rate 0.0003, clip 0.2, gamma 0.99, GAE lambda 0.95, four update epochs, and minibatches of 256. Observation normalization statistics are saved with checkpoints and frozen for evaluation. These settings are a starting point, not proof of optimal tuning.

<a id="docs-h-kabxie7a6xxh"></a>

### **Try it yourself**

Read policies/ppo.py and experiments/train\_ppo.py before running python -m experiments.train\_ppo. Locate rollout collection, advantage calculation, optimizer updates, and checkpoint saving. Training consumes time; viewing a checkpoint does not retrain it.

<a id="docs-h-o1mrr4s4aae"></a>

### **Limits and explanation**

This is shared PPO with a local critic, not MAPPO with a centralized critic or learned messages. Time-limit value bootstrapping deserves a future audit: the current calculation resets the final next value. Recruiter phrasing: “I implemented an on-policy actor-critic baseline and separated training from repeatable checkpoint evaluation.”

<a id="docs-h-jg8hfj9194pk"></a>

### **Sources and code**

[PPO implementation](<https://github.com/hudsonkaneko/hivemind-traffic/commit/32dc5c6>)

[PPO learning reference](<https://spinningup.openai.com/en/latest/algorithms/ppo.html>)

<a id="docs-h-yy3ph3rx2px2"></a>

### **Portable demonstration**

Watch inference from the trusted compatible shared-PPO checkpoint selected in your local configuration. To reproduce the historical refined-policy demo, select that checkpoint if you have it. Old 9-observation checkpoints are incompatible with the current 16-observation environment.

```text
python -m hivemind demo trained
```

Expected: **SUMO** opens and two agents select speed/lane actions with shared trained weights. Surrounding traffic and safety rules also affect movement. No optimization or training occurs. The command uses your trusted compatible checkpoint selected in local configuration and writes new evidence under results/rollouts, not new weights.

<a id="docs-h-dif0fzvqpmik"></a>

### **Portable command reference**

From any supported clone’s root, after local dependency setup and environment activation (see tab 21):

```text
python -m hivemind demo trained
```

The meaning and limitations of the demo above are unchanged. Tab 21 explains the ignored local configuration, checkpoint/recording prerequisites and --check. These commands do not make unsupported hardware compatible. Commands now use portable entry points; complete the local setup in tab 21 first.

<a id="docs-t-kzvpcax50f94"></a>

## 08 Awareness and safety refinement

<a id="docs-h-wd48w4gkrgf6"></a>

### **Change and purpose**

Commit 61e1b6a expanded **observations**, refined safety handling, extended training, and changed **checkpoint** selection. A **reward**-only checkpoint choice had favored cautious behavior that could fail to finish.

<a id="docs-h-4q604hygyqls"></a>

### **Technology and mechanism**

The observation grew from 9 to 16 values, adding both-lane front/rear context and cooldown. **PPO** masks impossible or unsafe lane choices. A safety shield can override actions for close traffic, occupied target lanes, or cooldown; **SUMO** retains its own safeguards. Selection now scores validation completion strongly, with reward as an additional component: 1000 × completion rate + mean team reward.

<a id="docs-h-xhnxt0w7orn2"></a>

### **Measured evidence**

For refined training seed 101, held-out seeds 2001–2003 completed both controlled vehicles without recorded collisions. PPO averaged 52.63 team reward, 37 overrides, and 245.33 steps. Scripted behavior averaged 56.92 reward, zero overrides, and 269 steps. Random behavior failed to complete within 400 steps and averaged 498 overrides. This supports a mixed result: PPO finished faster in these runs but did not beat the scripted policy on every metric.

<a id="docs-h-t7bph6hryad7"></a>

### **Limits of the claim**

These are three evaluation seeds from one training seed, not broad robustness evidence. Multiple settings changed together, so improvements cannot be assigned to one change without ablations. Gaps use route-distance differences rather than fully audited bumper clearances. Safety filtering is not a formal guarantee. Old 9-input checkpoints are not interchangeable with the new observation contract.

<a id="docs-h-x8ijepr2686l"></a>

### **Try it yourself and explanation**

Inspect the run under results/training/shared-ppo-no-communication/seed-101-20260922T035008Z, if present locally. Recompute the averages from its evaluation outputs. Recruiter phrasing: “I found that reward alone selected the wrong behavior, revised the evaluation criterion, and reported tradeoffs against a strong scripted baseline.”

<a id="docs-h-7ly7z66ui8ey"></a>

### **Sources and code**

[Refinement and evidence](<https://github.com/hudsonkaneko/hivemind-traffic/commit/61e1b6a>)

<a id="docs-h-r9x3jonukn00"></a>

### **Portable demonstration**

View your configured checkpoint on seed 2001. To reproduce the historical refinement result, select the corresponding refined checkpoint first:

```text
python -m hivemind demo trained
```

Expected: a trained-policy episode in the curved SUMO scenario. Explain the richer traffic observations, lane-choice mask and safety overrides while watching. A vehicle being passed does not by itself diagnose a failure. Inspect the newly written rollout metrics in results\\rollouts for completion, collisions and overrides; an animation alone cannot establish safety or superiority.

<a id="docs-h-6xqxvigk6yrk"></a>

### **Portable command reference**

From any supported clone’s root, after local dependency setup and environment activation (see tab 21):

```text
python -m hivemind demo trained
```

The meaning and limitations of the demo above are unchanged. Tab 21 explains the ignored local configuration, checkpoint/recording prerequisites and --check. These commands do not make unsupported hardware compatible. Commands now use portable entry points; complete the local setup in tab 21 first.

<a id="docs-t-nbbmkv8ek2tk"></a>

## 09 Static RTX lidar

<a id="docs-h-spt6ihpx96el"></a>

### **Change and purpose**

The static lidar smoke test, included in commit 1b75cda, established that Isaac could start a sensor and produce geometrically plausible data before attaching it to moving traffic.

<a id="docs-h-geibnuli1g9y"></a>

### **Technology and mechanism**

A rotary **RTX lidar** sits at (0,0,1) meters. Three two-meter boxes provide known targets, including one centered at (5,0,1). Its front face is roughly four meters from the sensor. Comparing returns with this simple expected distance checks more than merely confirming that a window opens.

<a id="docs-h-71lsktv5h6gm"></a>

### **Evidence and interpretation**

The recorded smoke test observed 30 callbacks and 24,010 front-box-near-four-meter hits. These are historical test results, not a promise for every new machine. Saved raw samples contain azimuth in degrees, elevation in degrees, and range in meters; they are not Cartesian XYZ points just because a filename sounds like a point cloud.

<a id="docs-h-8b6x2mw1z3e4"></a>

### **Try it yourself**

Run python -m hivemind demo lidar after configuring isaac\_root in hivemind.local.json. Inspect documentation/lidar-smoke.md and the generated metadata. Predict how the measured distance should change before moving a target in a separate controlled experiment.

<a id="docs-h-9nu1hiefo4pg"></a>

### **Why this design and limits**

Known geometry makes coordinate and unit mistakes visible. A successful static target test does not validate dynamic scan timing, perception quality, or safe driving. Recruiter phrasing: “I validated sensor output against known geometry before integrating it into a more complex moving scene.”

<a id="docs-h-mm2jz322xmri"></a>

### **Sources and code**

[Integration commit](<https://github.com/hudsonkaneko/hivemind-traffic/commit/1b75cda>)

[Lidar notes](<https://github.com/hudsonkaneko/hivemind-traffic/blob/69d3cb3/documentation/lidar-smoke.md>)

<a id="docs-h-uvbzezyvmcuc"></a>

### **Portable demonstration**

Visible static RTX lidar demonstration:

```text
python -m hivemind demo lidar
```

Expected: three boxes, a stationary lidar and visible point returns. The front box is centered 5 m ahead, so its near face is about 4 m away. The scene is for checking known geometry, not driving. Inspect the new outputs\\lidar\_smoke run for saved data/report; a plausible picture alone is not an accuracy test. Isaac closes after 1800 frames.

<a id="docs-h-f16njftvih3l"></a>

### **Portable command reference**

From any supported clone’s root, after local dependency setup and environment activation (see tab 21):

```text
python -m hivemind demo lidar
```

The meaning and limitations of the demo above are unchanged. Tab 21 explains the ignored local configuration, **checkpoint**/recording prerequisites and --check. These commands do not make unsupported hardware compatible. Commands now use portable entry points; complete the local setup in tab 21 first.

<a id="docs-t-utnb9vbixqfp"></a>

## 10 Moving traffic in OpenUSD

<a id="docs-h-8n2gtsvrrqrm"></a>

### **Change and purpose**

The same integration commit, 1b75cda, added recorded traffic playback in a 3D **OpenUSD** scene. Recording first decouples simulation and learning from rendering.

<a id="docs-h-vc2pt3bv3pbu"></a>

### **Technology and mechanism**

A seeded rollout records vehicle identity, dimensions, position, heading, speed, and time. The example recording contains 244 snapshots over approximately 48.6 seconds and five vehicles. Reusable vehicle assets are referenced by stable prims, while road geometry and time-sampled motion occupy separate USD layers.

<a id="docs-h-qik549vpmkfi"></a>

### **Coordinate and timing choices**

The scene uses meters, Z up, X east, and Y north. **SUMO** heading converts to USD yaw as 90 degrees minus heading. SUMO position denotes the front bumper; the vehicle asset extends backward along local X. Applying the network offset twice would shift the scene incorrectly. All layers use 60 time codes per second; an earlier 24-versus-60 mismatch caused endpoint errors.

<a id="docs-h-t3g1gkm96esp"></a>

### **Try it yourself**

Run python -m hivemind demo replay after creating the expected recording. Read usd/TRAFFIC\_SNAPSHOT.md and scripts/replay\_export.py. Compare a first and last vehicle pose in the recording with its USD transform at the corresponding time.

<a id="docs-h-mmpd55v0k76a"></a>

### **Evidence and limits**

A moving lidar run produced 300 callbacks over 30 seconds after enabling motion BVH support. That confirms activity, not full geometric or timestamp validation of every scan. Cars are kinematically replayed blocks, not wheel-driven physics vehicles. Recruiter phrasing: “I designed deterministic, timestamped scene playback with explicit unit and coordinate conventions so the same traffic data can be inspected in 3D.”

<a id="docs-h-g29c3zwg2tk"></a>

### **Sources and code**

[Replay implementation](<https://github.com/hudsonkaneko/hivemind-traffic/commit/1b75cda>)

[Snapshot contract](<https://github.com/hudsonkaneko/hivemind-traffic/blob/69d3cb3/usd/TRAFFIC_SNAPSHOT.md>)

[OpenUSD introduction](<https://openusd.org/release/intro.html>)

<a id="docs-h-qk60k170k5af"></a>

### **Portable demonstration**

Play the existing traffic recording in 3D:

```text
python -m hivemind demo replay
```

Expected: simplified block cars move along the road, the camera follows the ego vehicle, and green lidar points appear. This uses your existing outputs/sumo\_replay/recording.json and network.net.xml; see tab 21 to create them. It is recorded kinematic playback, not live SUMO, new **PPO** training, physics-driven steering or lidar-driven control.

The viewer refreshes generated replay layers, preview.png, scan samples and replay\_report.json in outputs\\sumo\_replay. It does not change the source recording or trained **checkpoint**. Close other Isaac instances first; the viewer closes after 2900 frames.

<a id="docs-h-3i3fxoz7a0o"></a>

### **Portable command reference**

From any supported clone’s root, after local dependency setup and environment activation (see tab 21):

```text
python -m hivemind demo replay
```

The meaning and limitations of the demo above are unchanged. Tab 21 explains the ignored local configuration, checkpoint/recording prerequisites and --check. These commands do not make unsupported hardware compatible. Commands now use portable entry points; complete the local setup in tab 21 first.

<a id="docs-t-i3zm9qaea3qe"></a>

## 11 Repository transfer

<a id="docs-h-nau40w9d8oaz"></a>

### **Change and purpose**

Integration work had been placed outside the chosen repository. It was transferred into hivemind-traffic, and the project scope was recorded to prevent repeating that mistake. This work is included in 1b75cda.

<a id="docs-h-od89dydmyg96"></a>

### **Technology and mechanism**

The transfer inventory covered 25 files. SHA-256 digests compared source and destination bytes before source copies were removed. Verification recorded that the targeted originals were gone and unrelated files were not part of the move.

<a id="docs-h-3e92spardel8"></a>

### **Why verification matters**

A successful copy operation alone does not prove the right files moved intact. An inventory defines scope; hashes verify byte identity; a post-transfer check confirms the intended final locations. Historical logs can retain old path strings without meaning active code still depends on that path.

<a id="docs-h-gris93ijxaqn"></a>

### **Try it yourself**

Read documentation/transfer-verification.md and AGENTS.md. Check git rev-parse --show-toplevel before editing. Use git status to distinguish tracked source from ignored outputs. Trace an input path from a command-line argument to its use in the script.

<a id="docs-h-s9d9ktrwvazi"></a>

### **Recruiter explanation and limit**

“I consolidated the implementation into the canonical repository and verified file integrity with an explicit inventory and hashes.” This is a configuration and traceability lesson, not an algorithmic innovation. External Isaac and Isaac Lab installations remain dependencies; they are not copied into the project.

<a id="docs-h-fh5v0s7mfr6u"></a>

### **Sources and code**

[Transfer verification](<https://github.com/hudsonkaneko/hivemind-traffic/blob/69d3cb3/documentation/transfer-verification.md>)

<a id="docs-h-jjksheulybph"></a>

### **Portable demonstration**

Demonstrate the canonical location without moving anything again:

```text
git rev-parse --show-toplevel
```

```text
git ls-tree --name-only HEAD
```

Expected: Git reports your clone’s repository root and lists tracked top-level entries. To demonstrate the transferred runtime integration from this folder:

```text
python -m hivemind demo replay
```

This is today’s working replay, not a repetition of the historical transfer. It refreshes the viewer’s generated replay artifacts as described in tab 10; no files are moved.

<a id="docs-h-1r1phurplrbr"></a>

### **Portable command reference**

From any supported clone’s root, after local dependency setup and environment activation (see tab 21):

```text
python -m hivemind demo replay
```

The meaning and limitations of the demo above are unchanged. Tab 21 explains the ignored local configuration, **checkpoint**/recording prerequisites and --check. These commands do not make unsupported hardware compatible. Commands now use portable entry points; complete the local setup in tab 21 first.

<a id="docs-t-pmiduualw22c"></a>

## 12 GUI lidar fix

<a id="docs-h-ex05zbg97zw0"></a>

### **Change and purpose**

The GUI replay initially failed with WriterRegistryError: no writer named draw-point-cloud. Headless sensor execution had not exercised this visualization-only path. The fix is included in 1b75cda.

<a id="docs-h-tyafzgjosoi4"></a>

### **Technology and mechanism**

Isaac extensions register services and writers during startup. The lidar class resolves writer aliases when initialized. Enabling isaacsim.sensors.rtx.nodes after creating the sensor was too late for the cached alias mapping. The fix enabled that extension before sensor construction.

<a id="docs-h-rsp07gu3sls0"></a>

### **Why this diagnosis**

The traceback pointed to writer lookup, not the traffic policy or sensor range calculation. The repair addressed initialization order rather than suppressing the exception. Optional visualization services can have dependencies that a headless data path never uses.

<a id="docs-h-9p622ovan6c5"></a>

### **Evidence and reproduction**

The GUI verification ran 240 frames and observed 40 sensor callbacks, with a screenshot inspected. Re-run the GUI replay command and confirm both startup and visible point output. Keep SimulationApp cleanup explicit so failures do not leave the runtime in an ambiguous shutdown state.

<a id="docs-h-rh5lax2ujrjp"></a>

### **Knowledge check and explanation**

Why did a passing headless smoke test miss this? It did not request the point-cloud drawing writer. Recruiter phrasing: “I traced a GUI-only failure to extension registration order and added verification of the actual visualization path rather than assuming a headless pass covered it.”

<a id="docs-h-jejb9n20skci"></a>

### **Sources and code**

[GUI integration notes](<https://github.com/hudsonkaneko/hivemind-traffic/blob/69d3cb3/documentation/sumo-3d-replay.md>)

<a id="docs-h-71b6y5s512mc"></a>

### **Portable demonstration**

Exercise the actual GUI point-cloud writer path that previously failed:

```text
python -m hivemind demo replay
```

Expected: Isaac starts the moving replay with visible green points instead of the earlier draw-point-cloud registry exception. This is the fixed current implementation, not a recreation of the broken version. Headless success alone does not test this path. The viewer refreshes its generated replay artifacts; visible points do not prove the stricter geometry/identity checks pass.

<a id="docs-h-drap1ndtqbd5"></a>

### **Portable command reference**

From any supported clone’s root, after local dependency setup and environment activation (see tab 21):

```text
python -m hivemind demo replay
```

The meaning and limitations of the demo above are unchanged. Tab 21 explains the ignored local configuration, **checkpoint**/recording prerequisites and --check. These commands do not make unsupported hardware compatible. Commands now use portable entry points; complete the local setup in tab 21 first.

<a id="docs-t-kcaya7u45f24"></a>

## 13 ovrtx architecture decision

<a id="docs-h-m20zvajlh351"></a>

### **Change and purpose**

Commit 6c8d007 made ovstage plus **ovrtx** an explicit target alongside Isaac Sim. This was an architecture decision before standalone rendering was implemented.

<a id="docs-h-z9mq8z5ux0bp"></a>

### **Technology and mechanism**

**OpenUSD** is the interchange representation. ovstage populates runtime scene state, and ovrtx renders it. Isaac provides its own application and sensor integration. Isaac using RTX internally is not the same as a standalone ovrtx integration.

<a id="docs-h-v8g0sxszmbud"></a>

### **Why share assets, not application internals**

Road geometry, vehicle assets, motion samples, and timestamps can be shared. Cameras, UI, and sensor-specific nodes belong in separate runtime adapters or layers. Otherwise an Isaac-only extension requirement can accidentally make a supposedly portable scene unusable elsewhere.

<a id="docs-h-16pf9ga577ut"></a>

### **Try it yourself**

Read experiments/ROADMAP.md. Classify each scene element as shared data or runtime-specific behavior. Explain how you would add another renderer without changing the **SUMO** environment or retraining the policy.

<a id="docs-h-dne77rcvh0iv"></a>

### **Limit and explanation**

At this decision point, standalone rendering was still planned; tab 14 describes its later implementation. Recruiter phrasing: “I selected a shared scene representation and isolated runtime-specific integrations so the project can support both sensor experiments and a lighter rendering path.”

<a id="docs-h-wwcetblqtssm"></a>

### **Sources and code**

[ovrtx decision](<https://github.com/hudsonkaneko/hivemind-traffic/commit/6c8d007>)

<a id="docs-h-xcn2jxh0k7bw"></a>

### **Portable demonstration**

Quick illustration of the renderer separation using an already-generated image:

```text
python -c "from pathlib import Path; import webbrowser; webbrowser.open(Path('outputs/ovrtx/traffic_2s.png').resolve().as_uri())"
```

Expected: your default browser opens the saved ovrtx traffic snapshot at replay time 2 seconds. This is later implementation evidence for an earlier architecture decision—not an interactive viewer, a new render, or ovrtx lidar. The image must already exist. Tab 14 shows how to regenerate it; opening it does not change artifacts.

<a id="docs-h-46te5ilel3u5"></a>

### **Portable command reference**

From any supported clone’s root, after local dependency setup and environment activation (see tab 21):

```text
python -m hivemind demo ovrtx
```

This runs the later RGB renderer, illustrating the earlier architecture decision. Tab 21 explains the ignored local configuration, **checkpoint**/recording prerequisites and --check. These commands do not make unsupported hardware compatible. Commands now use portable entry points; complete the local setup in tab 21 first.

<a id="docs-t-7yw661dv42iu"></a>

## 14 Shared exporter and ovrtx

<a id="docs-h-4rrrlznh92a5"></a>

### **Change and purpose**

Commit 69d3cb3 extracted a shared replay exporter and added standalone **ovrtx** RGB snapshots. The same recorded traffic can now feed both rendering paths without embedding Isaac-only sensor setup in the saved scene.

<a id="docs-h-23jh3131kds9"></a>

### **Technology and mechanism**

scripts/replay\_export.py authors the portable USD layers. scripts/render\_ovrtx.py loads a requested scene time through ovstage, advances renderer work with increasing ordinals, maps the RGB render variable, copies it to CPU memory, and releases the mapped view before unmapping. Eight warmup steps improve the snapshot without advancing traffic time.

<a id="docs-h-tjqdrb1ithcx"></a>

### **Important integration details**

The ovstage population API receives time in seconds; direct USD transform checks use seconds multiplied by 60 time codes per second. Confusing those units selects the wrong pose. Isaac sensors and cameras are kept in runtime/session composition. Do not save the runtime-mutated root layer back over clean shared assets.

<a id="docs-h-tmq0t3uiog25"></a>

### **Versions and evidence**

The dedicated environment pins ovrtx 0.5.0.377615, ovstage 0.2.0.377349, usd-core 26.8, NumPy 2.4.6, and Pillow 12.3.0. RGB snapshots at 2 and 15 seconds were visually inspected. Ten first/last vehicle pose checks passed; hashes confirmed the four shared USD files were unchanged by rendering. A subsequent Isaac regression ran 120 frames with 20 callbacks. These are recorded checks, not fresh tests performed while writing this guide.

<a id="docs-h-mndn3jrjmnng"></a>

### **Try it yourself**

Run python -m hivemind demo ovrtx --time 15. Inspect the image and manifest described in documentation/ovrtx-replay.md. Compare two requested times and verify the shared scene files remain unchanged.

<a id="docs-h-sbzx2byzcycp"></a>

### **Limits and explanation**

This is still-image RGB rendering, not an interactive viewer or an ovrtx lidar pipeline. Initialization-inclusive wall time is not a steady-state FPS benchmark. Recruiter phrasing: “I reused a portable motion scene across two runtimes and checked timing, pose consistency, and asset immutability.”

<a id="docs-h-psudmkqr59qr"></a>

### **Sources and code**

[Shared export and rendering](<https://github.com/hudsonkaneko/hivemind-traffic/commit/69d3cb3>)

[Rendering instructions](<https://github.com/hudsonkaneko/hivemind-traffic/blob/69d3cb3/documentation/ovrtx-replay.md>)

<a id="docs-h-l3kbumne0691"></a>

### **Portable demonstration**

Generate and then open a standalone ovrtx RGB snapshot:

```text
python -m hivemind demo ovrtx --time 2
```

```text
python -c "from pathlib import Path; import webbrowser; webbrowser.open(Path('outputs/ovrtx/traffic_2s.png').resolve().as_uri())"
```

Wait for rendering to finish successfully before opening the image. Expected: a still traffic image at replay time 2 seconds, using the same portable recording/scene family as Isaac. No Isaac GUI opens. This is an RGB snapshot, not live playback or lidar.

The portable launcher selects your ovrtx runtime and requires a local replay recording. It refreshes outputs/ovrtx/traffic\_2s.png, its JSON manifest and generated scene files; it may regenerate shared replay layers. To show another moment, use --time 15 and open outputs/ovrtx/traffic\_15s.png. To present instantly, use only the saved-image opening command after confirming the image exists.

<a id="docs-h-gacyu6cuuqnl"></a>

### **Portable command reference**

From any supported clone’s root, after local dependency setup and environment activation (see tab 21):

```text
python -m hivemind demo ovrtx
```

The meaning and limitations of the demo above are unchanged. Tab 21 explains the ignored local configuration, **checkpoint**/recording prerequisites and --check. These commands do not make unsupported hardware compatible. Commands now use portable entry points; complete the local setup in tab 21 first.

<a id="docs-t-fslvdlft9ot"></a>

## 15 Interview and study workbook

<a id="docs-h-u6ndujwlh4h4"></a>

### **Thirty-second explanation**

“Hivemind Traffic explores multi-agent traffic control. **SUMO** simulates the traffic, **PettingZoo** provides a consistent interface, and a shared **PPO** policy chooses speed and lane actions. I evaluate it against scripted and random baselines, then replay recorded traffic in **OpenUSD** for Isaac lidar checks and standalone **ovrtx** rendering. Lidar-driven control and learned communication are future stages.”

<a id="docs-h-tn7pgf1izcdg"></a>

### **Be accurate about your contribution**

Describe implementation as AI-assisted where relevant. Claim understanding only when you can demonstrate it. A useful answer covers the requirement you chose, the alternative you considered, the evidence you checked, and the limitation you still see. Do not claim that the policy learns steering, communicates explicitly, or drives using lidar today.

<a id="docs-h-uanuytr0m7rx"></a>

### **Questions worth practicing**

What does PettingZoo do that SUMO does not? Why is PPO called on-policy? How do actor and critic differ? Why are shared weights not communication? What is the difference between termination and truncation? Why can a higher reward hide failure to finish? What changes when an observation vector grows? Why does a render look correct yet have wrong timestamps? What would prove a lidar scan is geometrically correct?

<a id="docs-h-skfoco9lsq73"></a>

### **Independence exercises**

1\. Trace one action from the policy through the environment to TraCI and back to the reward.

2\. Run two matched baseline evaluations and explain every metric difference.

3\. Predict the effect of one reward change, then test it separately rather than changing several parameters.

4\. Recalculate one vehicle's USD yaw and time sample by hand.

5\. Design a moving-lidar target test with an expected distance and acceptable error.

6\. Explain why a saved scene should not depend on an Isaac-only sensor extension.

<a id="docs-h-uggfcw1kybl2"></a>

### **Vocabulary in plain language**

Policy: a rule or learned function mapping observations to actions. Observation: what an agent can see through the interface. Reward: numerical feedback used for learning. Value: expected future discounted reward. Advantage: how much better an action was than the baseline estimate. Checkpoint: saved learned state needed for replay or continued training. Seed: an input controlling pseudorandomness. Ablation: a comparison that changes one component to isolate its effect. Prim: a named object in a USD scene. Layer: a composable set of scene opinions.

<a id="docs-h-610zrtudhqc7"></a>

### **Next learning milestone**

Before claiming sensor-driven autonomy, validate moving lidar geometry and timing, define a sensor observation pipeline, and compare a policy trained for those observations against defensible baselines. Before claiming coordinated communication, implement an explicit message mechanism and test it against a no-communication control. Keep measured results, hypotheses, and planned work separate.

<a id="docs-h-sg0xwcil30ci"></a>

### **Sources and code**

[Project roadmap](<https://github.com/hudsonkaneko/hivemind-traffic/blob/69d3cb3/experiments/ROADMAP.md>)

<a id="docs-h-ywajzt795mwm"></a>

### **Portable demonstration**

Short presentation sequence: run one demonstration at a time, and explain the boundary before moving on.

```text
python -m hivemind demo trained
```

Say: “This is inference from a shared PPO checkpoint using simulator ground-truth observations; it is not training live.” After the SUMO episode finishes:

```text
python -m hivemind demo replay
```

Say: “This replays an existing traffic recording in 3D and generates RTX lidar returns. The lidar does not control the cars.” The replay need not exactly match the preceding live episode. After Isaac closes:

```text
python -c "from pathlib import Path; import webbrowser; webbrowser.open(Path('outputs/ovrtx/traffic_2s.png').resolve().as_uri())"
```

Say: “This saved image was rendered through the separate ovrtx path using portable USD scene data.” For a limitations discussion, open tab 20’s saved report. Do not describe a visual demo as proof of autonomy or sensor validation.

<a id="docs-h-jt97hhsp8ykm"></a>

### **Portable command reference**

From any supported clone’s root, after local dependency setup and environment activation (see tab 21):

```text
python -m hivemind demo trained
```

```text
python -m hivemind demo replay
```

```text
python -m hivemind demo ovrtx
```

The meaning and limitations of the demo above are unchanged. Tab 21 explains the ignored local configuration, checkpoint/recording prerequisites and --check. These commands do not make unsupported hardware compatible. Commands now use portable entry points; complete the local setup in tab 21 first.

<a id="docs-t-cchxbirgtixb"></a>

## 16 Moving lidar geometry and timing

<a id="docs-h-3pectgiw9yw3"></a>

### **What changed and why**

Commit 90bb92c adds a controlled moving **RTX lidar** test, a pure NumPy geometry checker, a frozen configuration, unit tests, and an offline analyzer. We previously knew the sensor generated returns. This change asks whether those returns match a known target at the correct time. It does not modify **PPO** or the traffic replay.

<a id="docs-h-y229xhkc3tht"></a>

### **The experiment**

An ego vehicle moves at 1 m/s and a target moves at 3 m/s along X. The target begins at (12, 2, 1) meters and is a 4 × 2 × 2 meter box. Lidar is mounted one meter above the ego origin. A static variant holds both still. One USD timeline owns all motion; there is no competing physics controller. The sideways target offset makes a reversed azimuth sign easy to detect.

<a id="docs-h-e0c3csfh3fw4"></a>

### **Technology and mechanism**

Isaac Sim generates spherical RTX lidar samples. **OpenUSD** supplies time-sampled box poses. NumPy converts azimuth/elevation to unit ray directions and computes ray–box intersections. Each measured range is compared with the expected first surface distance. This is why vehicle center distance is not the correct range reference. The installed validity flag is checked against the runtime enum; raw output is not assumed to be Cartesian XYZ.

<a id="docs-h-q2qxkntff3t0"></a>

### **The timing lesson**

A rotary scan spans time. Use the scan timestamp plus each ray’s offset, not the time Python receives the callback. Exploratory frame poses exposed a two-frame difference between sensor time and USD pose time in this installed runtime. The frozen mapping subtracts 2/60 seconds; confirmation runs independently check sensor frame poses against it. This calibration is specific to this test and runtime, not a universal correction for every replay.

<a id="docs-h-aqoeslns76uw"></a>

### **A rejected assumption**

One earlier static run failed a rule that required callback time to be within one frame of sensor frame-end time. One callback differed by two frames even though acquisition metadata and geometry were consistent. Protocol revision 2 treats callback timing as a diagnostic, keeps the acquisition and pose checks, and was tested on fresh runs. The failed run and protocol change are retained rather than silently excluded.

<a id="docs-h-ayuztvqexlpu"></a>

### **Acceptance and evidence**

Three fresh runs—one static, two moving—each produced 50 scans; the first three were predeclared warmup, leaving 47 evaluated scans per run. All passed. Static worst-scan 95th-percentile range error was 0.76 cm; moving was at most 1.25 cm. The largest moving range error was 2.38 cm. Configured limits were 10 cm for each scan’s 95th percentile and 20 cm for its maximum. Scan cadence was 0.1 seconds and sensor-frame pose errors were below 0.02 m. All 28 repository tests passed.

<a id="docs-h-22ztgfd8ce32"></a>

### **Proving the test can fail**

The offline analyzer verifies saved artifact hashes before recomputing results. Deliberately shifting timestamps by 250 ms or reversing azimuth makes the checks fail. In the moving case, the time shift produces approximately 52 cm 95th-percentile range error. Rays are not selected because their measured range happens to agree with the answer.

<a id="docs-h-zc7ixnv929sn"></a>

### **Reproduce it yourself**

From the repository root after tab 21 setup: the following portable Python dispatch reads isaac\_root from local configuration and runs the strict validator using Isaac’s own Python. Paste each full command as one line. It is not a demo alias.

```text
python -c "import subprocess,sys; from hivemind.launcher import ROOT,load_config,isaac_runtime; sys.exit(subprocess.call([str(isaac_runtime(ROOT,load_config(ROOT))),*sys.argv[1:]],cwd=ROOT))" scripts/validate_moving_lidar.py --variant static
```

```text
python -c "import subprocess,sys; from hivemind.launcher import ROOT,load_config,isaac_runtime; sys.exit(subprocess.call([str(isaac_runtime(ROOT,load_config(ROOT))),*sys.argv[1:]],cwd=ROOT))" scripts/validate_moving_lidar.py --variant moving
```

Repeat the moving command. Each run prints a unique directory under outputs/lidar\_validation. With the traffic environment active, recompute a chosen run without Isaac; replace YOUR\_RUN\_ID with an existing run directory:

```text
python scripts/analyze_lidar_validation.py outputs/lidar_validation/YOUR_RUN_ID
```

Run python -m pytest -q for the automated repository tests.

<a id="docs-h-orbm6ibdai11"></a>

### **Files and reproducibility**

scripts/validate\_moving\_lidar.py builds and runs the scene. scripts/lidar\_geometry.py implements the numerical checks. scripts/analyze\_lidar\_validation.py recomputes results and tests corruption. experiments/configs/lidar\_validation.json holds thresholds and clock calibration. Each run saves raw scans, metadata, a USD scene, resolved settings, source snapshots, Git dirty state, versions, and hashes. Generated scans stay local; the concise report and code are committed.

<a id="docs-h-8bqioolizls2"></a>

### **Limitations and next gate**

The test evaluates returned-ray consistency, not complete detection recall: omitted emitted rays are not available in this output. It covers straight translation with fixed orientations, not turns, complicated occlusions, physical steering, or the full **SUMO** traffic replay. GPU sensor output is not claimed to be bitwise repeatable. Apply the clock and geometry checks to the actual curved replay before turning lidar into policy observations. **ovrtx** still has a separate RGB rendering path, not a validated lidar adapter.

<a id="docs-h-yhxgstrwo0zr"></a>

### **Practice and recruiter explanation**

Exercise: predict how the gap changes after one second, then calculate where a 10-degree ray intersects the box. Explain why using the callback timestamp would be a mistake. Interview phrasing: “I built a sensor-validation fixture with analytic ground truth, diagnosed a clock offset, and used deliberately corrupted inputs to check that the validator rejects timing and coordinate mistakes.” Describe implementation as AI-assisted where relevant; claim understanding only when you can demonstrate it.

<a id="docs-h-lmv934ajblhn"></a>

### **Sources and code**

[Validation implementation and report](<https://github.com/hudsonkaneko/hivemind-traffic/blob/90bb92c/documentation/moving-lidar-validation.md>)

[Committed change](<https://github.com/hudsonkaneko/hivemind-traffic/commit/90bb92c>)

[NVIDIA GMO field reference](<https://docs.isaacsim.omniverse.nvidia.com/latest/py/docs/source/generic_model_output/generic_model_output.html>)

<a id="docs-h-tz1cd1tf88q8"></a>

### **Portable demonstration**

Run the controlled moving-target test with its GUI enabled:

```text
python -c "import subprocess,sys; from hivemind.launcher import ROOT,load_config,isaac_runtime; sys.exit(subprocess.call([str(isaac_runtime(ROOT,load_config(ROOT))),*sys.argv[1:]],cwd=ROOT))" scripts/validate_moving_lidar.py --variant moving --gui
```

Expected: a brief straight-line ego/target fixture with point visualization, followed by validation output. This is a small analytic test, not the curved traffic replay. Its default 300-frame capture is short; initialization can take longer than the visible motion. Let it finish to save a unique run under outputs\\lidar\_validation.

For a stationary comparison, rerun the same command with --variant static. Read the printed acceptance result and saved report; do not infer a pass just from seeing points. This test passing does not resolve the broader curved-replay failures.

<a id="docs-t-2ctsja7co7rg"></a>

## 17 Curved replay lidar validation

<a id="docs-h-1vfx3r8lnebz"></a>

### **What changed and why**

The controlled two-box check is now applied to the actual seed-2001 **PPO** traffic recording. New capture and offline-analysis scripts check rotating vehicle bodies, changing visibility, nearest-object occlusion, per-return time, and vehicle identities. The normal replay viewer, trained policy, and earlier controlled test remain unchanged.

<a id="docs-h-whmc9hxgqt11"></a>

### **How the geometry check works**

**SUMO** records front-bumper positions and headings. A separate NumPy reference interpolates them using the same declared time and yaw conventions as the USD exporter. The sensor ray is rotated into world coordinates, then into each vehicle’s coordinates. This turns an oriented-box intersection back into the simpler axis-aligned slab test. The nearest visible box determines the expected range and vehicle identity. Bodies match the current simplified USD asset, not detailed real cars.

<a id="docs-h-4qzgxksx1w5k"></a>

### **Why identity matters**

A plausible range alone does not prove the ray hit the intended vehicle. The validator also requests RTX object IDs and a StableIdMap. The renderer requires the stable-ID setting to be enabled at startup. Actual IDs are compared with independently predicted vehicle identities; missing IDs stay unresolved. Assigning missing IDs from the expected answer would make the test circular.

<a id="docs-h-71atis2ahe8o"></a>

### **Full-run findings**

The first complete capture produced 483 scans across 48.3 seconds. Three predeclared startup scans were excluded, leaving 480 evaluated scans and approximately 6.59 million checked vehicle returns. All four other vehicles were represented in the geometric reference. Timing and coverage passed. The worst scan’s 95th-percentile range error was 2.63 cm, but the largest individual discrepancy was 14.02 m and identity agreement was 92.8%, below the required 99%. Overall acceptance therefore FAILED. See the evidence report for the repeated run and exact metrics.

<a id="docs-h-xct29rtn4pfw"></a>

### **What the timing pass establishes**

The 0.1-second scan cadence remained consistent. Sensor frame-start/end positions matched the recorded curved trajectory under the explicit 2/60-second clock mapping, with maximum position discrepancy about 0.000031 m in the first full run. This checks position-based timing, not a separate quaternion-orientation acceptance test. Per-ray timestamps remain distinct from Python callback times.

<a id="docs-h-daastxmrerxo"></a>

### **What remains unresolved**

The supplied identity map stayed unchanged and listed only the two vehicles present at startup, omitting later background vehicles. A late interval had zero identity agreement after agent\_1 disappeared. Separately, rare silhouette returns can hit the road instead of the ideal ray’s predicted vehicle. The sensor has nonzero angular noise, so this is a plausible contributor, not a proven complete diagnosis. The strict 10 cm per-scan p95 and 20 cm maximum limits were not loosened to manufacture a pass.

<a id="docs-h-pg6uz8y2pgs2"></a>

### **Reproduce and inspect**

From the repository root after tab 21 setup: the following portable Python dispatch reads isaac\_root from local configuration and runs the strict validator using Isaac’s own Python. Paste each full command as one line. It is not a demo alias.

```text
python -c "import subprocess,sys; from hivemind.launcher import ROOT,load_config,isaac_runtime; sys.exit(subprocess.call([str(isaac_runtime(ROOT,load_config(ROOT))),*sys.argv[1:]],cwd=ROOT))" scripts/validate_replay_lidar.py
```

Configure isaac\_root once as described in tab 21. The recording and network must exist under outputs/sumo\_replay. Each capture gets a new directory under outputs/replay\_lidar\_validation. With the traffic environment active, recompute and run corruption controls; replace YOUR\_RUN\_ID with an existing run directory:

```text
python scripts/analyze_replay_lidar.py outputs/replay_lidar_validation/YOUR_RUN_ID
```

A failed acceptance gate produces a nonzero exit code. A short --frames 600 capture intentionally cannot pass full-route coverage.

<a id="docs-h-gjq23jrrlumk"></a>

### **Evidence and testing**

Each run preserves the source recording, **checkpoint**/network hashes, configuration, code snapshots, Git state, runtime versions, raw samples, per-scan metadata, and artifact hashes. The checker also tries a 250 ms timestamp shift and reversed azimuth. A corrupted input failing does not imply the original passed. Tests cover coordinate rotation, yaw interpolation, visibility transitions, nearest-object occlusion, unknown IDs, and empty-run rejection. The second full capture adds raw ID bytes for future offline investigation without changing the scene or sensor settings.

<a id="docs-h-8y3uyokxme9l"></a>

### **Limits and next decision**

This is returned-ray consistency against block-car geometry, not complete detection recall, independently validated road geometry, lidar-driven control, or physical steering. One traffic seed does not establish general robustness. Next, resolve the dynamic ID mapping and design an explicit noise-aware boundary test or controlled noise-off comparison. Keep the raw strict failures visible while doing that; do not feed the data into a policy and assume validation is complete.

<a id="docs-h-bnvrjg26iqmx"></a>

### **Practice and interview explanation**

Exercise: explain why a one-centimeter sideways change near a vehicle silhouette can produce a many-meter range change. Then explain why low p95 error does not justify hiding maximum errors. Interview phrasing: “I extended an analytic sensor test to a recorded multi-vehicle scenario and separated timing, geometry, identity, and coverage gates. The broader test exposed limitations that a simple scene missed, and I retained those failures as evidence.”

<a id="docs-h-eagfii1ggcqd"></a>

### **Repeat and code verification**

The repeat also produced 483 scans and the same pass/fail pattern. It checked approximately 6.59 million vehicle returns, with 2.64 cm worst-scan p95, 10.20 m maximum discrepancy, and 92.8% identity agreement. Its 248 returns over 20 cm were retained; the first run had 278. All 34 automated code tests passed. Implementation and evidence are committed as 01abf5e. This is a working validator reporting a failed acceptance test—not a fully validated sensor pipeline.

<a id="docs-h-em13isk6fnda"></a>

### **Sources and code**

[Curved replay validation report](<https://github.com/hudsonkaneko/hivemind-traffic/blob/01abf5e/documentation/replay-lidar-validation.md>)

[Implementation commit](<https://github.com/hudsonkaneko/hivemind-traffic/commit/01abf5e>)

<a id="docs-h-58pscqjxctji"></a>

### **Portable demonstration**

For the visual context, open the current moving traffic replay:

```text
python -m hivemind demo replay
```

This is a representative viewer, not the strict validator. It refreshes viewer outputs and can look convincing even when acceptance fails. After closing it, open the saved strict-validation evidence:

```text
python -m json.tool documentation/replay-lidar-results.json
```

Expected: a report of the earlier full-route checks, including failed geometry/identity acceptance. Opening the report does not rerun Isaac. To capture a fresh full-route validation instead, use the headless command below; it is slower than a presentation demo and has no GUI switch:

```text
python -c "import subprocess,sys; from hivemind.launcher import ROOT,load_config,isaac_runtime; sys.exit(subprocess.call([str(isaac_runtime(ROOT,load_config(ROOT))),*sys.argv[1:]],cwd=ROOT))" scripts/validate_replay_lidar.py
```

A nonzero exit can mean failed acceptance; inspect the report to distinguish that from a crash. Fresh data goes to a unique outputs\\replay\_lidar\_validation folder. Do not present this as a repaired sensor pipeline.

<a id="docs-h-hxntikuaetbb"></a>

### **Portable command reference**

From any supported clone’s root, after local dependency setup and environment activation (see tab 21):

```text
python -m hivemind demo replay
```

This launches only the visual replay; use the portable strict-validation commands above for strict acceptance tests. Tab 21 explains the ignored local configuration, checkpoint/recording prerequisites and --check. These commands do not make unsupported hardware compatible. Commands now use portable entry points; complete the local setup in tab 21 first.

<a id="docs-t-c9rggs6xex7k"></a>

## 18 Ground truth vs lidar perception

<a id="docs-h-nmdeado5bikf"></a>

### **The key distinction**

**SUMO** does not infer nearby vehicles from lidar in our current project. It maintains the simulated world’s state because it calculates vehicle movement. Position, speed, lane, and vehicle dimensions are simulator **ground truth**. Our code reads selected traffic information through **TraCI**; the policy does not automatically receive everything SUMO knows.

<a id="docs-h-s7m0lwlrfrok"></a>

### **Who does what today**

SUMO simulates traffic and applies its configured car-following, lane-change, and safety behavior. TraCI is the interface our Python code uses to read state and send control requests. **PettingZoo** packages selected state into agent observations and defines actions, rewards, and episode handling. **PPO** is the learning algorithm that trains the policy; during a rollout, the trained policy selects acceleration/braking and lane-change requests. SUMO executes the resulting requests under the configured constraints. The policy is not learning continuous steering to follow the road’s curves.

Current decision loop: SUMO state → TraCI / PettingZoo observation → trained policy action → TraCI request → next SUMO state.

<a id="docs-h-haae7kg5gulm"></a>

### **What the 3D replay and lidar do**

Isaac Sim currently replays recorded SUMO trajectories in a 3D OpenUSD scene. RTX lidar measures that scene, but its returns are not fed back into the driving policy. It does not cause a replayed vehicle to brake, accelerate, or go around another vehicle. Seeing a point cloud beside moving cars does not mean those cars are driving from lidar.

<a id="docs-h-dhkfibd3yv6u"></a>

### **Ground truth versus perception**

Ground-truth example: the simulator reports that the vehicle ahead is 15 metres away. Sensor-based example: lidar returns are processed to detect or represent an obstacle and estimate its distance before a policy makes a decision. Sensor-derived observations can include noise, occlusion, missed detections, and delay. A future policy could consume processed object features or a learned point-cloud representation; that design has not yet been implemented.

<a id="docs-h-otdgjq7bgotj"></a>

### **Why develop in this order**

Using simulator state first isolates coordination and reward design from perception errors. It gives us a baseline before adding sensor uncertainty. Moving to lidar requires validating the sensor pipeline, defining sensor-derived observations, and retraining or adapting and evaluating the policy. We cannot assume the current state-based policy will work unchanged on lidar input. The curved replay’s strict lidar acceptance test is still failing; it is not yet a validated lidar-driven control system.

<a id="docs-h-rjudz4bkjxpn"></a>

### **Why one car can lead and later be passed**

A car’s speed can change with policy requests, surrounding traffic, and SUMO’s configured constraints. A car that starts slower may later have a clearer route. An overtake alone does not prove the policy is better, worse, or reacting to lidar. Explaining a particular overtake requires checking the vehicles’ actions, speeds, and nearby traffic at that moment; the visual observation alone is insufficient.

<a id="docs-h-857hqq1f39rv"></a>

### **Explain it to a recruiter**

“I first trained a traffic-coordination policy using simulator ground-truth observations exposed through TraCI and a PettingZoo environment. I then replayed the traffic in Isaac Sim to develop and validate an RTX lidar pipeline. Sensor-based closed-loop control is a separate next stage, not something the current replay already demonstrates.”

<a id="docs-h-joxix8rg6763"></a>

### **Learning check**

If lidar is disabled in the current replay, the recorded vehicle trajectories should stay the same: the recording, not the lidar, determines their movement. This note documents the architecture discussed in our conversation; it makes no code or simulation changes.

<a id="docs-h-ecu9dv87x8ru"></a>

### **Portable demonstration**

Demonstrate the two separate data paths, one at a time:

```text
python -m hivemind demo trained
```

In this live SUMO episode, PPO observes simulator state through TraCI/PettingZoo. SUMO already owns the vehicle positions and road geometry; it does not infer them from lidar.

After SUMO finishes, launch the recorded 3D path:

```text
python -m hivemind demo replay
```

In Isaac, the existing recording determines vehicle motion and RTX generates sensor returns. Those returns do not feed back into the current PPO policy. This is not guaranteed to be a frame-for-frame replay of the live episode you just watched. The viewer refreshes its generated artifacts. Explain why seeing lidar beside moving cars is not proof of lidar-based driving.

<a id="docs-h-f5ztmwf44nhq"></a>

### **Portable command reference**

From any supported clone’s root, after local dependency setup and environment activation (see tab 21):

```text
python -m hivemind demo trained
```

```text
python -m hivemind demo replay
```

The meaning and limitations of the demo above are unchanged. Tab 21 explains the ignored local configuration, checkpoint/recording prerequisites and --check. These commands do not make unsupported hardware compatible. Commands now use portable entry points; complete the local setup in tab 21 first.

<a id="docs-t-jz0vf0ad2a8r"></a>

## 19 Lidar failure diagnostics

<a id="docs-h-oy66ajrcr2ao"></a>

### **Purpose and current status**

We investigated the curved replay’s missing vehicle identities and large range discrepancies before connecting lidar to a driving policy. This change adds diagnostic tools and evidence, not a successful sensor fix. **SUMO** still supplies recorded poses; Isaac Sim renders them; the **PPO** policy still consumes simulator state rather than lidar.

<a id="docs-h-5h9bd9zdgq05"></a>

### **What changed and how it works**

The capture script now offers opt-in identity-delta, zero-azimuth-noise, visibility-preflight, and uninstancing experiments. All switches default off. Renderer ID/path updates are retained across callbacks, including callbacks between complete lidar scans. Noise and scene changes are confined to the temporary session layer. Unknown object IDs remain unknown; we do not assign the geometrically nearest car as their answer.

A new offline NumPy audit checks saved artifact hashes, recomputes ray/vehicle intersections, builds an expected-versus-observed vehicle confusion matrix, and lists the largest range errors. It also measures how close the ideal intersection is to a box edge. That proximity is diagnostic only: no point is discarded and no acceptance threshold is relaxed. Unknown/nonvehicle labels combine unresolved IDs with road or ground hits, so that category is not a pure missing-ID count.

<a id="docs-h-8rpeb6fl3vz4"></a>

### **Experiments and evidence**

Five exploratory captures used the same seed-2001 recording and 600 frames each: **baseline**, identity deltas, zero azimuth noise, visibility preflight, and uninstanced vehicles with deltas. Each produced 100 scans, of which 97 were evaluated. All finished without callback errors, passed timing, and left the four shared USD layers unchanged. All still failed strict geometry/identity acceptance. Their short duration also intentionally fails the full-route coverage gate. These percentages must not be presented as improvements over the earlier full-route results.

Identity deltas did not add the later background cars. Making all known cars visible during paused initialization and restoring their recorded visibility did not solve it. Disabling instancing did not solve it either. These are retained rejected experiments, not recommended workarounds.

With azimuth noise disabled, 52 returns still exceeded 20 cm error and the maximum error was 8.337 m. Only 24 of those ideal intersections were within 2 cm of a box edge. Therefore neither angular noise nor silhouette proximity explains all remaining errors. Other sensor settings, including range accuracy, remain active. One capture per variant cannot establish a statistical noise effect.

<a id="docs-h-objki884850g"></a>

### **Why these design choices matter**

An ablation tests a suspected cause by changing it while preserving the rest of the setup as far as possible. Keeping rejected experiments prevents repeating guesses later. Separating diagnostic metrics from pass/fail gates avoids making the test easier just to obtain a pass. Immutable scans and hashes let us investigate without rerunning a GPU simulation or altering the original evidence. The experiments were chosen sequentially; this is exploratory debugging, not a pre-registered benchmark.

<a id="docs-h-8gvjfwmcvtl"></a>

### **Reproduce and study**

From the repository root after tab 21 setup, run the short baseline with the portable dispatch below. It reads isaac\_root from local configuration and runs the strict validator in Isaac’s Python; it is not a visual demo alias.

```text
python -c "import subprocess,sys; from hivemind.launcher import ROOT,load_config,isaac_runtime; sys.exit(subprocess.call([str(isaac_runtime(ROOT,load_config(ROOT))),*sys.argv[1:]],cwd=ROOT))" scripts/validate_replay_lidar.py --frames 600
```

Add --identity-deltas, --zero-azimuth-noise, --identity-preflight, or --identity-deltas --uninstance-vehicles to reproduce the corresponding diagnostic. Run captures serially. The expected nonzero exit signals failed acceptance; inspect the summary to distinguish that from a runtime crash.

Audit an existing capture without launching Isaac:

```text
python scripts/audit_replay_lidar.py outputs/replay_lidar_validation/YOUR_RUN_ID
```

Replace YOUR\_RUN\_ID with an existing local run folder. The audit can accept several run folders and --output NEW\_REPORT.json; it refuses to overwrite an existing report. Audit execution succeeding does not mean the sensor passed. Raw scans remain local; the checked-in generated report includes run IDs, configurations, hashes, confusion matrices, and outlier examples.

<a id="docs-h-ygv1oxuqs5m7"></a>

### **Tests, limits, and next step**

All 37 automated code tests passed. New tests cover edge distance, replay coordinates, preservation of unknown labels, read-only auditing, and tampered-artifact rejection. Passing these tests establishes code checks, not sensor accuracy. Implementation and evidence are committed and pushed as c004218.

Next isolate a two-vehicle appear/disappear case to separate renderer identity-map behavior from the replay adapter, and capture emitter/echo metadata to check the nearest-surface assumption. Investigate non-edge errors too. After a fix, repeat full-route validation twice and run negative controls. Do not start lidar-driven policy training on the assumption that these failures are resolved. No runtime upgrade or upstream issue submission was performed.

<a id="docs-h-48w2vhwohsid"></a>

### **Interview explanation and learning exercise**

“I built reproducible sensor diagnostics and used controlled experiments to test likely failure causes. I kept unknown identities and large errors visible, distinguished passing code tests from passing sensor acceptance, and documented unsuccessful fixes rather than claiming a plausible point cloud proved autonomous driving.” Exercise: explain why a 2 cm typical error and an 8 m worst error can coexist, then explain why neither metric alone describes the whole system.

<a id="docs-h-qn833bf9xbov"></a>

### **Sources and implementation**

[Diagnostic implementation and report](<https://github.com/hudsonkaneko/hivemind-traffic/blob/c004218/documentation/lidar-diagnostics.md>)

[Generated evidence from all five captures](<https://github.com/hudsonkaneko/hivemind-traffic/blob/c004218/documentation/lidar-diagnostic-results.json>)

[Implementation commit](<https://github.com/hudsonkaneko/hivemind-traffic/commit/c004218>)

[NVIDIA RTX sensor annotator documentation](<https://docs.isaacsim.omniverse.nvidia.com/6.0.0/sensors/isaacsim_sensors_rtx_annotators.html>)

<a id="docs-h-gzgc14vazmqw"></a>

### **Portable demonstration**

Fast demonstration of the diagnostic work without spending GPU time:

```text
python -m json.tool documentation/lidar-diagnostic-results.json
```

Expected: saved evidence from the five diagnostic captures. Show the failed acceptance results and retained outliers rather than claiming the experiments fixed the issue. This opens historical results; it does not execute a new test.

Optional fresh short capture (headless; no GUI window):

```text
python -c "import subprocess,sys; from hivemind.launcher import ROOT,load_config,isaac_runtime; sys.exit(subprocess.call([str(isaac_runtime(ROOT,load_config(ROOT))),*sys.argv[1:]],cwd=ROOT))" scripts/validate_replay_lidar.py --frames 600
```

Expected: a new outputs\\replay\_lidar\_validation run. The short capture intentionally cannot meet full-route coverage and currently also exposes unresolved geometry/identity failures; a failed acceptance exit is expected, not by itself a runtime crash. Use tab 10 for the representative visual scene. Run captures serially, not alongside the viewer.

<a id="docs-t-c8cnwa4eowg"></a>

## 20 Two-car lidar identity isolation

<a id="docs-h-slxwkoi5bs1x"></a>

### **What changed and why**

We built a minimal two-car sensor fixture to find whether missing lidar labels require the full traffic replay. It uses two stationary box-shaped cars and no **SUMO**, **PPO**, traffic exporter, vehicle instancing, or physics actuation. The test isolates the sensor/annotator mapping path; it does not replace the original curved traffic acceptance test.

<a id="docs-h-lgvj607pp0fw"></a>

### **Test design and technology**

**OpenUSD** defines the scene in metres with Z up. The ego body is behind a lidar at (0,0,1). A 4 by 2 by 2 metre target is centered at (12,0,1), so its front face is 10 metres from the sensor. Isaac Sim generates RTX returns using Example\_Rotary. NumPy independently computes expected ray/box intersections. The renderer’s exact object IDs are checked against its StableIdMap and StableIdMapDeltas; unknown IDs are never filled using the expected geometry.

The control keeps the target visible throughout. The lifecycle variant hides it for the first second, shows it for one second, hides it for one second, then shows it again. Each run contains 288 frames at 60 Hz. The lifecycle variant was repeated once. The variants, repeat, interior region and thresholds were specified before capture.

To isolate steady labeling behavior, the test checks rays within two degrees of the target’s center and excludes whole scans near visibility transitions, startup and shutdown using a 0.2-second guard. Each phase requires at least four scans. Visible phases require 100 hits, at least 99% identity agreement, at most 10 cm scan p95 range error and 20 cm maximum error. Hidden phases require no valid hits in that region. Scan cadence must remain 0.1 seconds within 100 ns.

<a id="docs-h-tlr7zbiyrog2"></a>

### **What the runs established**

The always-visible control passed: 34,402 checked returns, 100% correct target labels, and maximum range error below 1.9 cm. Both lifecycle runs passed geometry, hidden-target checks, cadence and phase coverage, but failed identity with 0% correct target labels during both visible phases. They checked 14,393 and 14,420 visible-phase returns respectively. The target path never appeared in their captured maps, despite accurate returns carrying the same nonzero raw object ID in both repeats.

All three runs produced 48 scans and had no callback errors. Every checked return had echo ID 0. The full automated code suite passed all 42 tests. Offline analysis verified artifact hashes and recomputed summaries exactly matching the captured summaries.

<a id="docs-h-gf0obvfd7d4"></a>

### **Interpretation and limits**

The missing-label behavior does not require our traffic exporter or learning policy. It is reproducible in the installed RTX sensor/annotator mapping path or our minimal use of that API. This is a useful narrowed diagnosis, not proof of a particular vendor defect and not a repaired sensor pipeline.

The interior region and guarded phases deliberately do not test silhouette accuracy or transition latency. Timing here checks scan cadence, not moving sensor pose alignment. Echo 0 in this fixture does not rule out multi-return effects elsewhere. The earlier large curved-replay range errors remain unresolved. We must not copy these exclusions into the broader test merely to obtain a pass.

<a id="docs-h-pbfygmucahv5"></a>

### **Reproduce it yourself**

From the repository root after tab 21 setup, run the control and then the lifecycle command twice. The portable dispatch reads isaac\_root from local configuration and runs the validator in Isaac’s Python. Paste each full command as one line.

```text
python -c "import subprocess,sys; from hivemind.launcher import ROOT,load_config,isaac_runtime; sys.exit(subprocess.call([str(isaac_runtime(ROOT,load_config(ROOT))),*sys.argv[1:]],cwd=ROOT))" scripts/validate_lidar_lifecycle.py --variant always-visible
```

```text
python -c "import subprocess,sys; from hivemind.launcher import ROOT,load_config,isaac_runtime; sys.exit(subprocess.call([str(isaac_runtime(ROOT,load_config(ROOT))),*sys.argv[1:]],cwd=ROOT))" scripts/validate_lidar_lifecycle.py --variant lifecycle
```

The lifecycle command currently exits nonzero because identity acceptance fails. That is the reproduced result, not a callback crash. Each run gets a unique folder under outputs/lidar\_lifecycle containing the scene, config, raw scans, source snapshots, runtime manifest, Git state and hashes. Emitter, channel and echo IDs are preserved alongside raw object IDs.

With the traffic environment active, recompute an existing run without launching Isaac. Replace YOUR\_RUN\_ID with an existing local run directory:

```text
python scripts/analyze_lidar_lifecycle.py outputs/lidar_lifecycle/YOUR_RUN_ID
```

The analyzer accepts multiple run folders and --output NEW\_REPORT.json, refuses to overwrite an existing report, and rejects altered artifacts or mismatched summaries. Successful analysis is distinct from sensor acceptance; read summary.passed. Raw scans stay local rather than being uploaded to GitHub.

<a id="docs-h-vk4z9t4nnpiz"></a>

### **Next step and interview explanation**

Use this small fixture to investigate documented map-refresh or initialization behavior, or a separately approved runtime version. A candidate fix must then survive the full curved traffic protocol and repeated validation. No runtime upgrade, public bug report, policy change, or lidar-driven control was performed in this step.

“I reduced a complex simulation failure to a minimal reproducible test. A static control labeled objects correctly, but the same target appearing later produced accurate ranges with missing labels. Repeating the experiment separated identity metadata from geometry and excluded the traffic and learning layers as necessary causes.” Exercise: explain why a passing distance check cannot prove correct object identity.

<a id="docs-h-l7m4gep5nb8r"></a>

### **Code and evidence**

[Implementation and reproduction notes](<https://github.com/hudsonkaneko/hivemind-traffic/blob/dd31657/documentation/lidar-lifecycle-validation.md>)

[Generated three-run evidence](<https://github.com/hudsonkaneko/hivemind-traffic/blob/dd31657/documentation/lidar-lifecycle-results.json>)

[Commit dd31657](<https://github.com/hudsonkaneko/hivemind-traffic/commit/dd31657>)

<a id="docs-h-n98yvocx7qhn"></a>

### **Portable demonstration**

Fastest demonstration: open the saved three-run comparison:

```text
python -m json.tool documentation/lidar-lifecycle-results.json
```

Explain: the always-visible control labeled the target correctly, while both appear/disappear runs had accurate ranges but failed identity. These are historical saved results, not a fresh rerun.

To repeat the small experiment, close other Isaac processes and run these commands one at a time:

```text
python -c "import subprocess,sys; from hivemind.launcher import ROOT,load_config,isaac_runtime; sys.exit(subprocess.call([str(isaac_runtime(ROOT,load_config(ROOT))),*sys.argv[1:]],cwd=ROOT))" scripts/validate_lidar_lifecycle.py --variant always-visible
```

```text
python -c "import subprocess,sys; from hivemind.launcher import ROOT,load_config,isaac_runtime; sys.exit(subprocess.call([str(isaac_runtime(ROOT,load_config(ROOT))),*sys.argv[1:]],cwd=ROOT))" scripts/validate_lidar_lifecycle.py --variant lifecycle
```

These launchers are headless and do not support --gui. Expected on the tested runtime: control acceptance passes, lifecycle identity acceptance fails with a nonzero exit, while distance checks can still pass. Read the new summary rather than assuming identical counts. Each capture creates a unique outputs\\lidar\_lifecycle folder. This isolates a metadata problem; it does not fix the full curved scene or demonstrate autonomous driving.

<a id="docs-t-2ivlli89tzj8"></a>

## 21 Portable demonstration launcher

<a id="docs-h-eejwbaomi4oo"></a>

### **What changed and why**

Commit ea80c27 adds a shared command interface so someone cloning this repository does not need Hudson’s username, Isaac path or **checkpoint** directory embedded in their commands. The launcher is a standard-library Python package; simulator, environment, policy and renderer code remain in their existing folders.

<a id="docs-h-a888fs5hzs89"></a>

### **First use on another machine**

Open a terminal in the cloned repository root. Install Python and the project’s traffic dependencies in .venv as described in README.md and documentation/portable-demos.md. Cloning alone does not install **SUMO**, Isaac, **ovrtx**, a GPU driver, trained weights or replay data. Supported runtime/platform requirements still apply.

If python is not available in your terminal, activate the project environment once per terminal. Windows Command Prompt:

```text
.venv\Scripts\activate.bat
```

Linux bash: source .venv/bin/activate. PowerShell: .\\.venv\\Scripts\\Activate.ps1, subject to execution policy. These are shell-specific setup steps; the demo commands below are shared. Use python3 instead of python if required by your environment.

<a id="docs-h-1lf9cxfhd3g8"></a>

### **Portable demo commands**

```text
python -m hivemind demo sumo
```

Live curved SUMO/**PettingZoo** episode using a scripted policy; not the original one-car-only scenario or training. Default seed 42.

```text
python -m hivemind demo random
```

The same current environment with seeded random actions. Erratic requests are expected; this is a **baseline**, not training.

```text
python -m hivemind demo trained
```

Live shared-PPO checkpoint inference. Default seed 2001; configure a trusted compatible 16-input checkpoint first.

```text
python -m hivemind demo lidar
```

Static three-box Isaac RTX lidar smoke test with a GUI; not moving traffic or sensor-driven driving.

```text
python -m hivemind demo replay
```

Isaac GUI playback of the existing traffic recording with lidar. Cars follow recorded poses; lidar does not control them.

```text
python -m hivemind demo ovrtx
```

Standalone RGB still render at replay time 2 seconds. Open outputs/ovrtx/traffic\_2s.png afterwards with your image viewer. This command does not open the image automatically, and does not provide an interactive viewer or ovrtx lidar.

<a id="docs-h-kh17qnizbeuu"></a>

### **Local settings without hardcoded shared paths**

Copy hivemind.example.json to hivemind.local.json and fill needed paths with forward slashes. The local file is Git-ignored. Configure isaac\_root and checkpoint; optionally configure traffic\_python, ovrtx\_python, sumo\_binary and sumo\_gui\_binary. Relative paths are resolved against the repository. These are paths, not commands with arguments.

Environment variables named HIVEMIND\_ plus the uppercase field override JSON; Isaac also accepts ISAAC\_SIM\_PATH. The launcher discovers the conventional .venv and .venv-ovrtx Python executables using Windows or POSIX layouts. Without one it checks the current Python. SUMO discovery supports explicit settings, SUMO\_BINARY, PATH, SUMO\_HOME and conventional Windows locations. An explicit SUMO\_BINARY must match the GUI/headless mode.

Each clone supplies its own Isaac installation directory and compatible checkpoint in the ignored local configuration. No personal installation or checkpoint path is required in the shared commands.

<a id="docs-h-r0dtupni3si7"></a>

### **Missing models and recording data**

No model is downloaded or selected by scanning arbitrary checkpoints. Use a trusted current-project checkpoint or run the existing training workflow separately. The existing policy loader uses pickle-based deserialization; untrusted .pt files can execute code. Older 9-input checkpoints are incompatible. A setup check does not load or validate the checkpoint.

```text
python -m hivemind demo trained --checkpoint results/training/YOUR_RUN/checkpoints/best.pt
```

Replace YOUR\_RUN with the actual run folder. After choosing that same checkpoint, generate the two inputs needed by replay/ovrtx:

```text
python -m hivemind demo record --checkpoint results/training/YOUR_RUN/checkpoints/best.pt
```

This headless recorder creates outputs/sumo\_replay/recording.json and network.net.xml. It refuses to replace either existing input unless you add --overwrite. Back up anything you want to keep first. Replay/rendering never silently starts training or regenerates a missing recording.

<a id="docs-h-n2ldeoqkvuou"></a>

### **Checks and controls**

```text
python -m hivemind demo replay --check
```

Append --check to any demo: it verifies paths and Python module availability in the selected runtime, without starting a simulation, GPU rendering or checkpoint deserialization. It does not prove driver support, sensor correctness or model compatibility.

```text
python -m hivemind demo sumo --delay 200
```

```text
python -m hivemind demo trained --headless --horizon 400
```

```text
python -m hivemind demo replay --frames 2900
```

```text
python -m hivemind demo ovrtx --time 15
```

Delay/horizon apply to rollouts; seed applies to traffic; frames applies to Isaac demos; time applies to ovrtx. The existing recorder has its own horizon. Use one demo at a time. Let finite demos finish; Ctrl+C interrupts a stuck run. Isaac startup can be slow.

<a id="docs-h-igqmnq8wxodu"></a>

### **Outputs and boundaries**

Rollouts create result folders and lidar smoke tests create run folders. Replay refreshes generated USD layers, sample scans, preview and replay\_report in outputs/sumo\_replay. ovrtx refreshes generated layers and its output image/manifest. Neither changes source code or trained weights. Only record with --overwrite deliberately replaces existing replay inputs.

The strict moving, curved and lifecycle validation tools remain separate scripts; demo commands do not replace their acceptance protocols. Their longer portable python -c commands in tabs 16–20 reuse the launcher’s Isaac path discovery and preserve the validator arguments and exit status. Configure isaac\_root first. Direct tests, analyzers and training commands use the activated traffic environment. Existing lidar failures are not fixed by this documentation change.

<a id="docs-h-m8c90gu6hbpc"></a>

### **Verification and limits**

All 65 automated tests passed. Tests cover configuration, Windows/POSIX path layouts, spaces in paths, missing dependencies and artifacts, overwrite protection, dispatch, no-launch setup checks and child exit codes. Setup checks passed locally for sumo, trained, lidar, replay and ovrtx.

After CMD activation, the exact python -m hivemind entry point completed a four-step scripted headless rollout and a four-step trained headless rollout. These verify dispatch and checkpoint loading, not performance. GUI/GPU demos were not rerun for this change. Windows is the verified machine; POSIX path tests are not end-to-end Linux/macOS validation.

<a id="docs-h-3pio199j5ipl"></a>

### **Design and interview explanation**

The launcher builds argument lists, selects the right runtime, sets the child working directory to the repository, checks prerequisites and forwards the exit status. Keeping this thin avoids a second implementation of simulation or rendering. It does not auto-install dependencies or silently choose a model.

“I separated a portable user interface from machine-specific runtime configuration. One command dispatches to an existing component, with explicit dependency and artifact checks, while personal paths stay out of version control.” Exercise: configure a nonexistent interpreter, run --check, and explain why failing clearly is safer than silently choosing another environment.

<a id="docs-h-a8af46bz1ewp"></a>

### **Implementation reference**

The launcher was committed and pushed as ea80c27. Main files: hivemind/launcher.py, hivemind.example.json, tests/test\_demo\_launcher.py and documentation/portable-demos.md. The demonstration sections throughout this guide now use portable commands rather than machine-specific shortcuts.

<a id="docs-t-mum52kmdfd9i"></a>

## 22 Lidar identity map-source investigation

<a id="docs-h-mco1o92hyhqb"></a>

### **Outcome and purpose**

Commit 424dec4 adds controlled diagnostics for missing lidar object identities. This is progress in isolating the failure, not a validated sensor fix. Neither candidate passed the complete fixture test. Lidar-driven policy observations and further PPO training remain on hold.

The earlier two-car test measured a target at the correct distance but could not map its raw lidar ID to the target’s USD path when the target first became visible. This change asks whether that failure is specific to the sensor render product, and whether explicit renderer initialization helps.

<a id="docs-h-j5nklzgjn2eb"></a>

### **Technology and design**

A render product is a renderer output target. The diagnostic adds a separate 64×64 camera render product and requests its StableIdMap and StableIdMapDeltas. These map exact **128-bit** renderer object IDs to USD prim paths. Camera RGB is not being used as a perception model.

The sensor’s original map and the camera map are both preserved. A new --map-source option selects which map supplies target labels; the default remains sensor. Camera data is read after each app update, and callbacks only use mappings already received. No future mapping, nearest-object guess, ID-bit masking or discarded unknown points are used.

A second, opt-in --render-preflight diagnostic renders the known target visible for 12 frozen-time steps before capture. It restores the authored visibility schedule and time zero, then attaches the capture writer. This was explicitly tested as a limited initialization hypothesis—not a general solution for newly created objects.

<a id="docs-h-gh5kcdymeebl"></a>

### **Unchanged acceptance rules**

The same stationary two-car geometry, rotary sensor, 288-frame duration, 60 Hz timeline, ±2-degree interior ROI and 0.2-second transition guards apply. Visible phases require at least 100 hits, 99% identity agreement, scan p95 range error ≤10 cm and maximum error ≤20 cm. Hidden phases require zero valid ROI hits. Coverage and scan-cadence gates remain unchanged.

All objects remain a kinematic USD fixture. This does not involve SUMO, PPO, the traffic exporter, physics-driven steering or lidar-based vehicle control.

<a id="docs-h-fx5smju2d7gx"></a>

### **Fresh results**

Sensor-map lifecycle baseline: 0% identity agreement in both steady visible phases. Geometry, hidden-target checks, cadence and coverage passed.

Camera-map lifecycle and repeat: both still had 0% visible-phase identity agreement. The target path was absent from the maps. Distance and the other fixture gates passed. A separate camera map therefore did not fix this failure.

Always-visible camera controls: both passed, with 34,398 and 34,390 checked hits and 100% identity agreement. The second control was inadvertently launched while the first shell’s completion was uncertain; both completed runs are retained. Maximum checked range error across the baseline and camera-only runs stayed below 1.86 cm.

Rendered preflight: visible-phase identity agreement reached 100%, but 4,001 and 3,992 valid hits appeared in the two phases where the target should be invisible. The USD schedule and time-reset checks passed, but renderer behavior was not equivalent. This candidate is **REJECTED**. It remains opt-in solely to reproduce the failure and is not used by the normal viewer.

All six captures produced 48 scans with valid cadence and no callback errors. No full-route rerun or promotion followed because neither candidate passed the prerequisite fixture gates. The existing curved-replay geometry and identity failures remain unresolved.

<a id="docs-h-ocloilrh1hpz"></a>

### **Evidence and verification**

The offline analyzer now reconstructs target labels from raw **128-bit** IDs and saved renderer maps. It rejects missing keys, wrong paths, low-bit-only matches and saved labels inconsistent with those maps. Sensor-only labels are also verified alongside selected camera labels. All 66 automated tests passed.

All six fresh packages passed artifact-hash checks and reproduced their captured summaries offline. The three earlier lifecycle packages were also rechecked without modification. Raw scans, maps, source snapshots, settings and manifests stay in unique local outputs/lidar\_lifecycle folders; concise generated results are committed.

Baseline: 20260924T163218Z-lifecycle-sensor-efd302. Camera: 20260924T163355Z-lifecycle-camera-d35145. Repeat: 20260924T163434Z-lifecycle-camera-678f98. Controls: 20260924T163455Z-always-visible-camera-f95271 and 20260924T163528Z-always-visible-camera-97bc2e. Rejected preflight: 20260924T163626Z-lifecycle-camera-ec1d1e.

<a id="docs-h-h4eybyuysue0"></a>

### **Portable demonstration and reproduction**

For a quick explanation without launching Isaac, display the committed report. Start in your clone’s repository root with the traffic environment activated as described in tab 21:

```text
python -m json.tool documentation/lidar-map-source-results-complete.json
```

For a fresh headless camera-map lifecycle capture, configure isaac\_root in hivemind.local.json, close other Isaac processes, and paste this entire command as one line. It discovers your local Isaac runtime; no personal installation path is embedded:

```text
python -c "import subprocess,sys; from hivemind.launcher import ROOT,load_config,isaac_runtime; sys.exit(subprocess.call([str(isaac_runtime(ROOT,load_config(ROOT))),*sys.argv[1:]],cwd=ROOT))" scripts/validate_lidar_lifecycle.py --variant lifecycle --map-source camera
```

Run captures serially. Repeat that command once; use --map-source sensor for the baseline and --variant always-visible for the control. To reproduce the rejected initialization specifically, append --render-preflight to the camera command. Do not use that option as a recommended fix. Validation failure exits nonzero; inspect the summary to distinguish it from a runtime crash.

```text
python scripts/analyze_lidar_lifecycle.py outputs/lidar_lifecycle/YOUR_RUN_ID
```

Replace YOUR\_RUN\_ID with the printed run directory. Successful analysis means the evidence was checked, not that the sensor passed. Inspect summary.passed.

```text
python -m pytest -q
```

<a id="docs-h-78w34j7vqlxt"></a>

### **Limits and next investigation**

The results narrow the issue beyond the sensor writer alone but do not prove a specific vendor defect. Initialization can change renderer behavior even when authored USD values look restored. Next, compare explicit per-frame visibility updates against time-sampled visibility while retaining the same negative controls. Runtime upgrades or an external bug report would be separate actions; neither was performed.

<a id="docs-h-h3mp1j8fw9ty"></a>

### **Learning and interview explanation**

**Negative control:** a condition that should not produce the signal—in this test, the hidden target should produce no valid ROI returns. A workaround that improves one metric while breaking a negative control is not an accepted fix.

“I compared independent renderer maps and initialization as competing hypotheses. Initialization appeared to repair object labels, but it created returns from hidden objects. I rejected it using unchanged acceptance gates and preserved the raw evidence rather than calling a plausible point cloud a validated sensor.”

Source: scripts/validate\_lidar\_lifecycle.py, scripts/analyze\_lidar\_lifecycle.py and tests/test\_lidar\_lifecycle.py. Protocol and interpretation: documentation/lidar-map-source-study.md. Generated evidence: documentation/lidar-map-source-results-complete.json. Implemented and pushed as 424dec4.

<a id="docs-t-df6w7wqf3jkt"></a>

## 23 Initial visibility and lidar registration

<a id="docs-h-qmj0kx657rvp"></a>

### **Outcome and purpose**

Commit e293acf isolates whether a target must be visible at startup for its lidar object ID to be registered. Starting visible passed twice; starting hidden still failed. This narrows the cause but does not fix the original scenario.

<a id="docs-h-wc86x6fn13hn"></a>

### **Technology and design**

OpenUSD time samples control visibility. RTX lidar produces distances and **128-bit** object IDs. **StableIdMap** associates those IDs with scene paths. A **negative control** is a condition that should produce no signal: the hidden target must yield zero valid interior hits.

The capture now retains every received sensor map buffer, including callbacks without populated scans, plus cumulative map snapshots, effective stable-ID settings and composed body visibility. USD visibility is not proof of the renderer’s internal state.

<a id="docs-h-juxi894buly8"></a>

### **Controlled results**

The initially-hidden baseline had 14,403 checked visible returns with 0% target-ID agreement. Both initially-visible repeats passed, with 18,409 and 18,396 checked visible returns and 100% agreement. All hidden-phase checks had zero hits. Maximum checked range error stayed below 1.86 cm.

All three captures had 48 scans, no callback errors, and passed offline hash, exact-ID label and summary verification. The baseline’s stable-ID setting was true at startup and attachment; none of its 96 recorded map buffers contained the target path. At this implementation checkpoint, 67 automated tests passed.

Evidence: documentation/lidar-initial-visibility-results.json and documentation/lidar-initial-visibility-study.md. Baseline run: 20260927T193657Z-lifecycle-sensor-1b46bb. Repeats: 20260927T193814Z-initially-visible-sensor-805f07 and 20260927T193847Z-initially-visible-sensor-d61faf.

<a id="docs-h-xb5z16138njj"></a>

### **Portable reproduction**

From the repository root, activate the traffic Python environment and configure isaac\_root in hivemind.local.json as described in tab 21. Close other Isaac processes. Paste each command as one line.

```text
python -c "import subprocess,sys; from hivemind.launcher import ROOT,load_config,isaac_runtime; sys.exit(subprocess.call([str(isaac_runtime(ROOT,load_config(ROOT))),*sys.argv[1:]],cwd=ROOT))" scripts/validate_lidar_lifecycle.py --variant initially-visible
```

Repeat once; change initially-visible to lifecycle for the failing baseline. These are headless diagnostics, not GUI demos.

```text
python scripts/analyze_lidar_lifecycle.py outputs/lidar_lifecycle/YOUR_RUN_ID
```

Replace YOUR\_RUN\_ID with the printed folder name. Raw scans remain local; analysis success means evidence consistency, not sensor acceptance.

```text
python -m json.tool documentation/lidar-initial-visibility-results.json
```

<a id="docs-h-6071lc5wk259"></a>

### **Limits and learning**

Making traffic visible earlier changes the scenario and is not a production fix. No policy, normal viewer, runtime installation or physics control changed. Full curved-replay range and identity failures remain unresolved.

“I changed one initial condition, repeated the comparison, and checked that labels remained correct after hiding and reappearing. I distinguished a useful diagnostic result from a general fix.”

Next: test direct visibility updates while keeping the target initially hidden; see tab 24.

<a id="docs-t-tcee1jwa6vj6"></a>

## 24 Explicit visibility diagnostic

<a id="docs-h-tm14b5tbx8sw"></a>

### **Outcome and purpose**

Commit 765d912 tests direct visibility updates as an alternative to USD animation. Both initially-hidden direct-update repeats still failed object-ID labeling. The initially-visible control passed. This is a rejected fix candidate, not a repaired lidar pipeline.

<a id="docs-h-5f4h2c6jlt01"></a>

### **Technology and design**

**Time-sampled visibility** stores a schedule in USD. **Explicit visibility** writes the current default value directly when the live timeline crosses a transition. The diagnostic polls before every app update, writes only changes, and saves the exact times in visibility-events.json.

**StableIdMap** connects full **128-bit** return IDs to scene paths. The same sensor and analytic box reference are used in every run. No nearest-object relabeling, ID masking, discarded unknown points, preflight, runtime upgrade or policy changes are introduced. The normal viewer retains its original behavior.

<a id="docs-h-12s083nchiqc"></a>

### **Protocol and results**

One fresh time-sampled lifecycle baseline, two explicit lifecycle repeats, and one explicit initially-visible control were run serially. All four had 48 scans, no callback errors, verified artifact hashes and exactly reproduced offline labels and summaries.

Baseline: 14,395 checked visible returns; 0% ID agreement. Explicit repeats: 14,418 and 14,402 returns; both 0%. Explicit initially-visible control: 18,411 returns; 100%. All hidden phases had zero hits. Maximum checked range error across the four captures was below 1.86 cm.

Acceptance thresholds were unchanged: 99% ID agreement, at least 100 hits and four scans per visible phase, p95 range error at most 10 cm, maximum 20 cm, zero hidden-phase hits, and the existing cadence check. All 75 automated tests passed.

Explicit lifecycle writes occurred at 0, 1.0000, 2.0167 and 3.0167 seconds. Floating-point timeline accumulation delayed two transitions by one update. This remains inside the predeclared 0.2-second guards; these tests do not establish exact transition latency.

Run IDs: baseline 20260928T052103Z-lifecycle-sensor-65ca49; explicit 20260928T052206Z-lifecycle-sensor-3e0c5a; repeat 20260928T052237Z-lifecycle-sensor-295f1f; control 20260928T052325Z-initially-visible-sensor-4ebb94.

<a id="docs-h-q0qed667fonk"></a>

### **Portable reproduction**

Activate the traffic Python environment at the repository root and configure isaac\_root in hivemind.local.json as in tab 21. Close other Isaac applications. Paste this entire command as one line:

```text
python -c "import subprocess,sys; from hivemind.launcher import ROOT,load_config,isaac_runtime; sys.exit(subprocess.call([str(isaac_runtime(ROOT,load_config(ROOT))),*sys.argv[1:]],cwd=ROOT))" scripts/validate_lidar_lifecycle.py --variant lifecycle --visibility-mode explicit
```

Repeat once. Omit --visibility-mode explicit for the baseline. Use --variant initially-visible with explicit mode for the control. These are headless diagnostics, not GUI demonstrations; a failed validation deliberately exits nonzero.

```text
python scripts/analyze_lidar_lifecycle.py outputs/lidar_lifecycle/YOUR_RUN_ID
```

Replace YOUR\_RUN\_ID with the printed folder. Analysis success verifies evidence consistency; inspect summary.passed for sensor acceptance. Raw scans remain local, not on GitHub.

```text
python -m json.tool documentation/lidar-explicit-visibility-results.json
```

```text
python -m pytest -q
```

<a id="docs-h-st6qeurr4e2g"></a>

### **Interpretation and next test**

The result weakens an animation-only change-notification explanation. Initial registration or map refresh remains a leading hypothesis, not a proven NVIDIA defect. Direct updates successfully hide and restore a target that was mapped at startup, but do not register the initially-hidden target in these runs.

Next: attach a fresh map annotator after the target appears without changing its geometry or visibility. Require two successful repeats and zero hidden returns before promoting any candidate. Full curved-replay geometry and identity validation remains outstanding; lidar-based policy training has not begun.

<a id="docs-h-13iobowlu5mp"></a>

### **Learning and evidence**

**Negative control**: a condition that should produce no signal. Keeping the hidden-target check prevents a labeling workaround from passing by accidentally leaving invisible traffic detectable.

“I compared animation and direct updates under the same acceptance rules. Both failed for initially-hidden objects while an initially-visible control passed. This ruled out a plausible workaround and narrowed the next experiment.”

Evidence and source: documentation/lidar-explicit-visibility-study.md, documentation/lidar-explicit-visibility-results.json, scripts/validate\_lidar\_lifecycle.py, scripts/lidar\_lifecycle\_analysis.py and tests/test\_lidar\_lifecycle.py. Associated commit: 765d912.

<a id="docs-t-yt6tixhcaw4x"></a>

## 25 Hybrid roadmap and GitHub workflow

<a id="docs-h-xe6kgop5tp8w"></a>

### **Outcome and purpose**

The canonical checkout is C:\\Users\\hudso\\Documents\\highwaysim. The GitHub repository remains hudsonkaneko/hivemind-traffic. The documentation index, change log, development workflow and roadmap now separate working demonstrations from planned physics migration.

<a id="docs-h-l3r9tyt9mb32"></a>

### **Current versus planned**

CURRENT: the existing cooperative LiDAR demo still uses SUMO to move the cars. The Leatherback waypoint experiment separately demonstrates an RC-scale articulated vehicle and PPO. Its policy and dynamics are not validated for a full-size highway car.

IN PROGRESS: a separate low-speed one-car PhysX foundation. PLANNED: hybrid traffic with explicit drivers, LiDAR on physical cars, a two-way SUMO bridge and later Isaac Lab learning. Existing experiments and results stay intact.

<a id="docs-h-pdf4jji8dxzo"></a>

### **Why hybrid simulation**

Motion authority means the one system allowed to determine a vehicle’s actual pose. Isaac/PhysX will own the evaluated cars; SUMO will own inexpensive background traffic. Driver type, motion backend and LiDAR equipment are separate settings.

A SUMO background vehicle needs a sensor-visible proxy in Isaac. A physical Isaac vehicle needs a shadow in SUMO so background drivers react to its actual motion. Neither side may overwrite the other's owned car. Cross-boundary contact is a failed trial, not a validated realistic crash.

<a id="docs-h-9rqmgfdz3h5q"></a>

### **Behavior → planning → control**

Behavior chooses a tactic such as staying in a lane or yielding. A planner turns that tactic into a path and speed target. Low-level control produces steering, propulsion and braking. PhysX resolves tire forces and movement. LiDAR measures the resulting scene; it does not move the car.

<a id="docs-h-lq7anuz5q49b"></a>

### **The next 20 steps**

1\. Preserve and identify the current baseline.

2\. Extend identity, state, frame, time and motion-authority contracts.

3\. Build a simple full-size physical vehicle with explicit dynamics.

4\. Add the bounded steering, throttle and brake command interface.

5\. Validate low-speed acceleration, braking, turning, stability and repeated resets.

6\. Define a shared lane and route map independent of visual road assets.

7\. Build scripted behavior, path planning, lane keeping and lane changes.

8\. Attach LiDAR to the physical chassis and validate timing and fallback.

9\. Synchronize physics, controls, sensors and SUMO without double stepping.

10\. Build the two-way bridge with SUMO shadows and Isaac background proxies.

11\. Test one physical car interacting with one SUMO-controlled car.

12\. Add a second physical LiDAR car and a small SUMO-free validation mode.

13\. Compare no communication, ideal V2V and delayed or dropped messages.

14\. Configure human drivers, independent AVs and communicating AVs at 0%, 25%, 50%, 75% and 100% AV ratios.

15\. Add seeded demand, routes, safe entry queues and explicit removal.

16\. Measure efficiency, safety, fairness, communication and compute cost.

17\. Run matched-seed scripted comparisons, including unfinished trips.

18\. Measure 4-, 10- and 20-car tiers with a fixed small sensed fleet.

19\. Validate Isaac Lab integration; introduce learned control only after scripted baselines.

20\. Integrate detailed assets, teammate environment, overview/follow cameras, paths and ovrtx; revalidate performance.

<a id="docs-h-n2fpldlth1ll"></a>

### **Experimental safeguards**

Keep demand, seeds, driver assignments and physics/sensor assignments matched when changing the AV ratio or communication mode. Four total cars are the smallest fleet that represents all five requested ratios exactly. Do not relabel native SUMO human driving as an AV policy.

The previous capacity study measured simple SUMO and RTX workloads separately, not a full physical fleet. Two LiDARs plus inexpensive background cars remain the starting performance strategy. Highway-speed stability, long GUI sessions and maximum physical fleet size remain open tests.

<a id="docs-h-d2w8bfgbx2qt"></a>

### **GitHub and documentation routine**

Work in one canonical checkout on a feature branch. Commit meaningful code, tests and documentation together; push the branch as a checkpoint and use a pull request before merging into main. Git stores history, so do not create competing project-v1/v2 folders.

Raw LiDAR scans, checkpoints, logs and unique run packages remain local under ignored outputs/ and logs/. GitHub receives source, configurations and compact evidence summaries. Pushing does not back up ignored files.

This notebook is the learning companion. The repository is the implementation source of truth. Historical tabs preserve the findings made at that time; consult the active roadmap for current priorities.

<a id="docs-h-jdxqyyq2iz1v"></a>

### **Portable checks**

Run from the repository root with the traffic Python environment activated. Each command below is one line. None installs dependencies or changes the old demo.

```text
git status --short --branch
```

```text
python -m pytest -q
```

```text
python scripts/demo_cooperative_lidar.py --check
```

<a id="docs-h-dihzlwolh4om"></a>

### **Repository references**

[Active hybrid roadmap](<https://github.com/hudsonkaneko/hivemind-traffic/blob/955e47da7cb613b2c64284e50cf378f858bfb7aa/documentation/hybrid-roadmap.md>)

[Motion-authority decision](<https://github.com/hudsonkaneko/hivemind-traffic/blob/955e47da7cb613b2c64284e50cf378f858bfb7aa/documentation/decisions/001-hybrid-motion-authority.md>)

[Development and documentation workflow](<https://github.com/hudsonkaneko/hivemind-traffic/blob/955e47da7cb613b2c64284e50cf378f858bfb7aa/documentation/workflow.md>)

[Change log](<https://github.com/hudsonkaneko/hivemind-traffic/blob/955e47da7cb613b2c64284e50cf378f858bfb7aa/documentation/CHANGELOG.md>)

<a id="docs-h-n6ckarl2z1rf"></a>

### **Learning takeaway**

Separating the driver, physics engine and visualization makes it possible to test whether coordination helps traffic without confusing prettier graphics or different vehicle dynamics with a better policy.

<a id="docs-t-n7c34hqtzn8v"></a>

## 26 Physics-driven car foundation

<a id="docs-h-tbof0cj9870h"></a>

### **Outcome and purpose**

**VERIFIED**: a separate physical car now accelerates, brakes and turns without SUMO. An explicit scripted driver sends actuator commands; PhysX determines movement. The existing cooperative LiDAR demonstration remains unchanged and still uses SUMO movement.

This is a low-speed compatibility test, not PPO training or the completed hybrid simulation. The broader one-car foundation is still in progress because highway dynamics, repeated-reset memory health and Isaac Lab integration have not passed their gates.

<a id="docs-h-nr92klpw3ezb"></a>

### **What the code does**

At 60 Hz the scripted driver compares measured speed with a 3 m/s target. It produces normalized throttle, brake and steering requests. At 120 Hz the command gate checks identity, order, validity and limits; the vehicle adapter applies wheel torques and **Ackermann steering**. PhysX then advances the car by one fixed simulation step.

**Motion authority**: only PhysX changes the physical car’s motion. The script never teleports the chassis or sets its velocity each step. Direct physics readback supplies the test driver’s speed; this is explicitly **simulator-state control**, not LiDAR-derived perception.

**Command expiry**: the runner gives commands a 12-tick, 100 ms lifetime. Missing deliveries hold the last valid command only until expiry. Invalid or expired commands remove propulsion, apply the configured brake and return steering toward zero at a limited rate.

**Ackermann steering**: the inside and outside front wheels turn by different angles so their geometric paths share a turn center. This is steering geometry, not a replacement for tire physics.

<a id="docs-h-a6gufg151r3k"></a>

### **Vehicle model and dependencies**

The installed NVIDIA PhysX vehicle sample supplies a 1,800 kg box car, 3.2 m wheelbase, 1.6 m track, 0.35 m wheel radius, tire friction and raycast suspension. Front-wheel drive is capped at 700 N·m per wheel; brakes at 1,500 N·m per wheel. Actual braking depends on tire contact, not just the requested torque.

World coordinates are X forward, Y left and Z up, measured in meters. Steering is in radians, positive left. Position is the chassis origin, not SUMO’s front bumper. Readback quaternions use xyzw. The old state contract was preserved; a versioned cross-backend conversion is still required.

The installed Isaac Sim 6.0.1 release candidate runs this probe with its own Python. No dependencies were installed and no runtime files were modified. The RC-scale Leatherback controller and PPO policy were not scaled up or claimed to transfer.

<a id="docs-h-9bhuba3zw4mw"></a>

### **Measured results**

The published run 20261003T092703Z-7806d7c0 contains two freshly built 28-second episodes. Both passed every declared smoke gate: positive forward progress, speed, drift, braking, hold, left turn, upright stability, wheel contact and command-expiry stopping.

Straight speed was 2.99127 m/s. Braking began at that speed and reached the stop threshold within 0.67187 m. Left-turn yaw change was 1.33487 radians. All four wheels remained grounded throughout the post-settling measurements.

Stationary hold drift was about 0.000157 m. Command-dropout final speed was about 0.000226 m/s. Recorded positions and speeds matched exactly between the two episodes on this installation; the acceptance tolerance was 0.001 m and 0.001 m/s.

All 331 automated tests passed at this checkpoint, including 77 command-contract tests and 10 acceptance-gate tests. Unit tests check software behavior; the physical run separately checks movement.

<a id="docs-h-g5etnv28o2fd"></a>

### **Failures, warnings and limits**

The initial run 20261003T092506Z-aa86d952 also passed two episodes. It exposed a USD stage reference-count warning. Releasing Python references before rebuilding did not eliminate that warning; both attempts are retained. The next reset/lifetime test must investigate it rather than claiming long-run memory stability.

The published run sampled a whole-GPU peak of 4,070 MiB, including desktop applications, and a process peak working set of about 5.08 GiB. This physics-only, headless test does not measure rendered fleet performance or prove a maximum car count.

No LiDAR, V2V, road-following planner, hybrid bridge or Isaac Lab task is attached to this new car yet. A successful left turn is not lane keeping or highway-speed calibration. The sample helper is a version-sensitive dependency that needs a pinned or project-owned replacement before production use.

<a id="docs-h-72adlu43l4a"></a>

### **Portable demonstration and checks**

Start at the repository root with the traffic environment activated and isaac\_root configured in hivemind.local.json, or ISAAC\_SIM\_PATH set. Paste each command as one line. Close competing Isaac jobs first.

```text
python -m experiments.verify_physics_vehicle --check
```

The check resolves the local runtime and configuration; it does not launch or validate physics.

```text
python -m experiments.verify_physics_vehicle
```

This runs headlessly, prints a unique local evidence folder, and exits nonzero on failure. The supervisor imposes a 180-second deadline and a sampled 90% GPU-memory stop. This is a numerical demonstration, not a new GUI.

```text
python -m json.tool documentation/physics-vehicle-results.json
```

This displays the committed compact evidence. Raw trajectories, initial USD stages, logs and source snapshots stay under outputs/physics\_vehicle and are not included in a GitHub clone.

```text
python scripts/demo_cooperative_lidar.py
```

This launches the existing visible two-car LiDAR demo. It is still SUMO-controlled movement, not the new physical-car experiment.

<a id="docs-h-pukqkb19ne57"></a>

### **Next implementation gate**

Validate ten resets and stage lifetime, right turns and turn radius, 1 m/s and 6 m/s behavior, and a one-environment Isaac Lab action/reset adapter. Then add shared lanes and a scripted path-following controller before attaching LiDAR and the SUMO bridge.

<a id="docs-h-7vzulakps2g9"></a>

### **Learning and source**

[Driver-command contract](<https://github.com/hudsonkaneko/hivemind-traffic/blob/955e47da7cb613b2c64284e50cf378f858bfb7aa/traffic/driver_control.py>)

[Physics foundation and reproduction](<https://github.com/hudsonkaneko/hivemind-traffic/blob/955e47da7cb613b2c64284e50cf378f858bfb7aa/documentation/physics-vehicle-foundation.md>)

[Recorded physics results](<https://github.com/hudsonkaneko/hivemind-traffic/blob/955e47da7cb613b2c64284e50cf378f858bfb7aa/documentation/physics-vehicle-results.json>)

Engineering explanation: I separated driving decisions from vehicle dynamics, added an expiring actuator contract, and tested the physical response independently of sensing. I preserved the existing traffic baseline and kept the unresolved reset warning visible instead of treating one successful run as a finished autonomy stack.

[GitHub review — physical-car foundation (PR \#1)](<https://github.com/hudsonkaneko/hivemind-traffic/pull/1>)

Source checkpoint: 955e47d. The development branch is pushed; main remains unchanged pending review.

<a id="docs-t-egh5rmb516m4"></a>

## 27 Dynamics, resets and Isaac Lab

<a id="docs-h-64p93be1hmxh"></a>

### **Outcome and purpose**

**VERIFIED SUBSET:** the physical car now has broader braking/turning evidence and a working one-environment Isaac Lab control/reset interface. All 434 automated CPU tests passed. The existing SUMO-driven LiDAR demonstration and earlier results remain unchanged.

**OPEN GATE:** standalone scene rebuilding still emits one USD reference-count warning. Its reset and dynamics subchecks passed, but their supervisors deliberately report **failed overall**. The separate Lab probe passed without that warning. This is not yet lane following, physical-car LiDAR, a hybrid fleet or policy training.

<a id="docs-h-nw26lm97u19r"></a>

### **How the components interact**

An explicit scripted controller sends steering, throttle and brake at 60 Hz. The command gate checks identity, timing and limits; the wheel adapter applies torques and Ackermann steering; PhysX advances at 120 Hz. No runtime script teleports the moving car. Pose and velocity writes are permitted only at an episode reset.

Isaac Lab wraps that same physical car in **DirectRLEnv**. A **RigidObject** provides chassis reset access, while native vehicle rest-state calls reset wheels and suspension. It is not a robot Articulation. The task currently supports one CPU-physics environment, not cloned or multi-agent environments.

The action interface has three values: steering in radians, positive left, limited to ±0.5; throttle and brake in \[0,1\]. Braking suppresses propulsion. Commands expire after 12 physics ticks, or 100 ms, with the existing braking fallback.

The observation interface has eleven simulator-state values: position XYZ, yaw, velocity XYZ, upright-Z, applied steering, throttle and brake. This is **privileged simulator state**, not LiDAR perception. Zero reward is intentional: this is an interface probe, not a learning objective.

<a id="docs-h-ay3k2875bb4"></a>

### **What the dynamics tests measured**

Six scenarios ran twice: braking at 1, 3 and 6 m/s targets; left and right circles at a 3 m/s target; and four seconds of unpowered coasting. All twelve cases passed their physical checks over 288 simulated seconds.

Stopping distances were 0.07536 m from 0.99709 m/s, 0.67187 m from 2.99127 m/s and 2.67475 m from 5.98243 m/s. Stop means speed ≤0.05 m/s. The declared limits were 1, 4 and 8 m respectively. These are this sample model’s responses, not real-car braking guarantees.

At ±0.2 rad steering, both rear-axle turn radii were about 15.97414 m—1.1912% from the low-slip bicycle reference, within the 10% gate. Five-second hold checks passed; combined tilt remained below five degrees; all post-settling samples had four grounded wheels.

Recorded position, speed and yaw matched exactly between equivalent repeats on this installation, within 0.001-unit tolerances. Coasting reduced speed by about 0.05235 m/s over four seconds. This does not establish real aerodynamic drag or universal cross-machine determinism.

<a id="docs-h-xeu2172x5tn9"></a>

### **Isaac Lab result**

Three eight-second Lab episodes drove and braked successfully. Each drove about 9.46239 m during its driving phase and ended near 0.00000481 m/s. Initial plus three automatic resets returned the checked chassis and wheel speeds to zero. Recorded position and speed traces matched.

The probe checked finite \[1,11\] observations, \[1,3\] actions, episode timing, reset counts and wheel-state reset. Installed Lab source commit `0603cb1710dcda13087665f63abf9ac483b63c05` was clean. Terminal-step physical state is pre-reset while the returned observation is post-reset; future training needs explicit terminal-observation handling. Angular velocity and full suspension state were reset by API but not independently recorded. No dependencies were installed, no runtime files were edited, and no SUMO or LiDAR was used.

<a id="docs-h-tfoqp01ji7yr"></a>

### **The reset investigation—and what did not work**

The tools-enabled control reproduced one warning. Disabling NVIDIA’s vehicle-authoring tools did not remove it. Processing a stopped application update after opening each scene also did not remove it. The exact remaining **reference owner is unproven**; we did not patch private vendor state or suppress warnings.

Both ten-reset attempts produced identical final positions and speeds, with an empty stage cache after every close. Post-warmup peak process-memory growth was about 19 MiB and 0 MiB against a provisional 256 MiB limit. Zero means no sampled increase above the chosen baseline—not no allocations or proven leak freedom.

The warning appeared on the first research stage’s close, not on the next nine closes. Disabling authoring also produced a plugin-interface-pointer warning, so it is not described as a fix. The dynamics supervisor likewise fails its separate zero-warning gate despite twelve successful physical cases. All failed attempts remain in the evidence.

<a id="docs-h-gsloirz10i5i"></a>

### **Performance and scope**

The Lab run took about 17.63 seconds including startup and shutdown; its process peak was about 2.37 GiB RAM. The dynamics suite took about 167.26 seconds and peaked near 5.10 GiB RAM. Sampled whole-GPU peaks were 4,676 and 5,696 MiB; reset attempts reached 7,316 MiB. Desktop usage is included.

These unpaced, headless tests do not establish real-time fleet capacity, GUI endurance or sensor cost. Logs retain a solver velocity-noise warning and two property/material UI startup errors about OmniPlaybackAPI schema handlers. They did not prevent the measured Lab reset/step checks; the pass does not certify an error-free runtime. One environment does not validate vectorized training.

<a id="docs-h-hbba7peex9sh"></a>

### **Portable demonstrations**

From your clone’s repository root, activate the traffic environment and configure isaac\_root in hivemind.local.json or ISAAC\_SIM\_PATH. The launcher selects Isaac’s separate Python. Paste each command as one line and run only one Isaac job at a time.

```text
python -m experiments.verify_vehicle_stage --stage lab --check
```

This checks configuration and runtime discovery without starting the simulation.

```text
python -m experiments.verify_vehicle_stage --stage lab
```

This runs the verified headless Lab drive/brake/reset demonstration. Look for passed: true and its unique local evidence folder.

```text
python -m experiments.verify_vehicle_stage --stage dynamics
```

This runs braking, turns, holds and coasting. Physical cases passed in the recorded run; a nonzero exit currently reports the separate retained scene-warning gate.

```text
python -m experiments.verify_vehicle_stage --stage resets
```

This performs ten fresh-stage episodes. Inspect individual subchecks and runtime.log; the retained warning currently makes overall status failed even when reset behavior passes.

```text
python -m json.tool documentation/physics-validation-results.json
```

This displays compact evidence for all five retained attempts. Raw traces, logs and source snapshots live under local outputs/ and are not shipped in a GitHub clone. These commands are numerical demonstrations, not a new GUI.

<a id="docs-h-opx3j9trru15"></a>

### **Next development step**

Build shared straight/curved lanes and a scripted path-following controller. Keep behavior decisions, planned path/speed and low-level control separate. Then attach physical-car LiDAR and verify sensor timestamps and frames before using perception for control. Keep the scene-reference warning investigation open.

The SUMO shadow bridge, two physical cars, configurable mixed-driver ratios, V2V-failure experiments, learned controllers and polished visualization remain later gates. Simple car visuals are sufficient while these interfaces are validated.

<a id="docs-h-fnam8xnjha52"></a>

### **Source and learning reference**

[Validation note and reproduction](<https://github.com/hudsonkaneko/hivemind-traffic/blob/03e9b3e83a3ff2cfb615d6d9a6b83077f1ebe84b/documentation/physics-validation.md>)

[Hash-verified evidence, including failed attempts](<https://github.com/hudsonkaneko/hivemind-traffic/blob/03e9b3e83a3ff2cfb615d6d9a6b83077f1ebe84b/documentation/physics-validation-results.json>)

[Isaac Lab vehicle environment](<https://github.com/hudsonkaneko/hivemind-traffic/blob/03e9b3e83a3ff2cfb615d6d9a6b83077f1ebe84b/environments/physics_vehicle_lab.py>)

[NVIDIA vehicle authoring versus physics runtime](<https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/extensions/ux/source/omni.physx.vehicle/docs/index.html>)

[GitHub review — physical-car foundation (PR \#1)](<https://github.com/hudsonkaneko/hivemind-traffic/pull/1>)

Engineering lesson: a passing motion test, a reliable reset, a clean shutdown and a valid learning environment are different claims. I tested and reported them separately instead of treating one working demo as proof of the entire autonomy stack.

Source checkpoint: 03e9b3e. The development branch is pushed; main remains unchanged pending review.

<a id="docs-t-70inmvd9orgv"></a>

## 28 Scripted lane following

<a id="docs-h-zctkhcovivj8"></a>

### **Outcome and purpose**

**VERIFIED SUBSET:** one full-size sample car follows straight, left-curved and right-curved 100 m routes using Isaac physics. All six physical route cases and all 603 CPU tests passed. No SUMO movement, LiDAR input or policy training was used in this new fixture.

**OPEN GATE:** the overall supervisor still reports failed because the existing scene-reference warning occurred once. The passing driving checks do not close the lifecycle-reliability gate. Previous experiments and failed evidence remain preserved.

<a id="docs-h-vsbeika59cyt"></a>

### **What changed and why**

The driving map is now independent of road art: three analytic 100 m routes, 3.6 m lane width, and centerline/boundary samples no more than 0.5 m apart. Each curved route has 20 m straight, a 60 m arc with radius 60 m, then 20 m straight. The surface is an infinite collision plane, not a new highway mesh or GUI.

Metres and radians are explicit. X/Y form the ground plane, Z points up, and positive yaw/steering turns left. Progress refers to the chassis center; steering uses a rear axle 1.6 m behind it. SUMO’s existing front-bumper convention is preserved, not silently redefined.

<a id="docs-h-t9fdg4hoef8a"></a>

### **Three driver layers**

**Behavior** chooses the known-clear route and desired speed. **Planning** turns that intention into an identity-tagged, expiring route reference and endpoint. **Low-level control** turns tracking error into steering, propulsion and braking. These boundaries let us later replace one component without hiding responsibilities.

**Pure pursuit** is a geometric steering controller: choose a point ahead of the rear axle, then steer toward it. Lookahead is 3 + 0.6 × speed metres. A speed servo requests wheel torque or braking; approach speed reduces near the endpoint and braking latches at station 99.8 m. This is not obstacle avoidance or a general trajectory optimizer.

**PhysX—not the planner**—determines movement. Runtime commands change wheel steering/drive/brake settings; they do not copy the chassis pose along the lane. The controller currently reads **privileged simulator state** and a known map. It is not seeing the road through LiDAR.

<a id="docs-h-ny0bq7ps6jps"></a>

### **Timing, command safety and contact checks**

Behavior/planning run at 10 Hz, low-level control at 60 Hz, and physics/command gating at 120 Hz. The first two startup ticks only hold the brake until physical state is available. Every route then has 2 s settling plus 48 s driving/holding, producing 6,000 physical samples.

Commands expire within 0.10 s, route references within 0.20 s and behavior intentions within 0.30 s. Invalid identity, time, frame or state requests zero propulsion and full braking. Steering is bounded to ±0.5 rad and 0.5 rad/s. CPU tests cover these failures; this run does not yet inject physical sensor/communication faults.

A **positive control** deliberately approaches a static barrier before the route tests. The detector reported the correct chassis/barrier pair at tick 566 (4.7167 s). The first four points were within PhysX contact margins, with positive separation and zero impulse: reporting works, but this is not validation of realistic crash response.

<a id="docs-h-3qplj9nhy0wj"></a>

### **Measured results**

Each route ran twice at a 3 m/s target; achieved peak speed was about 2.9703 m/s. Both curves had **RMS lane error** about 0.01841 m and maximum error about 0.04866 m, within the predeclared 0.20 m and 0.50 m limits. Straight-route errors were below 0.000006 m.

All six route cases had no reported rigid contacts, no lane departure and no driver/command fallback. A conservative envelope for the whole chassis—not only its center—stayed inside the lane. The largest envelope was 1.07085 m against a 1.8 m half-width.

Final five-second holds stayed within 0.12262 m of the 100 m endpoint, below 0.000288 m/s, with no more than 0.00000416 m drift. All post-settling samples had four supported wheels. Matched position, speed and yaw traces were identical across repeats on this installation; cross-machine determinism is not established.

Independent review recomputed line/arc projections, checked the metrics and verified 63 artifact hashes plus 15 captured-source hashes. The frozen attempt is 20261004T194416Z-42992541. Its failed overall status is retained because of the separate reference-warning gate.

<a id="docs-h-yniy4ue8jysj"></a>

### **Performance and limits**

Startup took 41.23 s and the whole process 205.11 s, with about 5.23 GiB peak process RAM and 3,765 MiB sampled whole-GPU usage, including the desktop. This unpaced headless one-car test includes recording but no LiDAR or GUI. It does not prove real-time fleet capacity.

All seven stages closed with an empty cache/context, but one reference warning remained. There were no runtime \[Error\] lines in this attempt; other warnings remain in the log. SUMO map correspondence, lane changes, displaced-start robustness, highway speeds, hybrid traffic and perception-driven control are still open.

<a id="docs-h-vq9hrlxtahsi"></a>

### **Portable numerical demonstration**

From your clone’s root, activate the configured traffic Python environment and set isaac\_root in ignored hivemind.local.json or ISAAC\_SIM\_PATH. The launcher chooses Isaac’s separate Python. Paste each command as one line; run only one GPU job at a time.

```text
python -m experiments.verify_vehicle_stage --stage lane-following --check
```

This validates configuration/runtime discovery only; it does not start Isaac.

```text
python -m experiments.verify_vehicle_stage --stage lane-following
```

This runs the detector check and six headless route episodes. No window or training opens. Inspect the printed evidence folder, case-reports.json and summary.json. The recorded overall result is false because of the retained scene warning even though every lane case passed.

```text
python -m json.tool documentation/lane-following-results.json
```

This displays the compact committed evidence. Large raw traces, initial scenes, contacts and logs stay in local outputs/ and are not included in a GitHub clone.

<a id="docs-h-6ky2bsa8vqud"></a>

### **Next step—and your role in training**

Next: mount LiDAR on this physics-driven chassis and verify sensor mounting, coordinate frames, acquisition/delivery timestamps, stale-scan fallback and obstacle stopping. Keep simulator-state inputs distinct from sensor-derived observations. Preserve the lifecycle investigation and shared SUMO-map gate alongside that work.

**YOUR PARTICIPATION:** before Isaac Lab policy training starts, we will work through and agree on observations, actions, rewards, algorithm, training budget and held-out evaluation together. This preference is saved in project instructions and the active roadmap. No unattended training is authorized by this scripted-validation milestone.

<a id="docs-h-vs4q11iezfb1"></a>

### **Source and learning references**

[Implementation and reproduction note](<https://github.com/hudsonkaneko/hivemind-traffic/blob/e4404d39a6f1d79be4d87fb1d1cc0480564035eb/documentation/lane-following.md>)

[Hash-verified lane evidence](<https://github.com/hudsonkaneko/hivemind-traffic/blob/e4404d39a6f1d79be4d87fb1d1cc0480564035eb/documentation/lane-following-results.json>)

[Scripted behavior, planning and control](<https://github.com/hudsonkaneko/hivemind-traffic/blob/e4404d39a6f1d79be4d87fb1d1cc0480564035eb/traffic/path_following.py>)

[Coulter’s geometric pure-pursuit report](<https://publications.ri.cmu.edu/implementation-of-the-pure-pursuit-path-tracking-algorithm>)

[GitHub review — physical-car foundation (PR \#1)](<https://github.com/hudsonkaneko/hivemind-traffic/pull/1>)

Engineering lesson: distinguish choosing a route, tracking it with commands, and physically moving the car. A car can pass its driving checks while a separate runtime reliability check still fails. Report both instead of reducing every result to one success label.

Source checkpoint: e4404d3. The development branch is pushed; main remains unchanged pending review.

<a id="docs-t-rb5nks6ogqck"></a>

## 29 Physics car and LiDAR view

<a id="docs-h-hujmp5s6lq0z"></a>

### **Outcome and purpose**

**VERIFIED SUBSET:** one simple full-size car now drives along a visibly marked curved road using Isaac/**PhysX** movement. The window includes overview/follow cameras, a cyan planned path, a pink steering target, optional green LiDAR returns and pause controls. The car brakes before an orange barrier.

This is a **live scripted feedback loop**, not a replay or trained policy. No SUMO connection is required for this separate fixture. Existing SUMO traffic demonstrations, experiments and results are preserved. Detailed vehicle assets and realistic environment art are not part of this change.

<a id="docs-h-vg09gdxv150z"></a>

### **What the view means**

Cyan is the upcoming reference from a known road map—not a prediction learned from LiDAR. Pink marks the **pure-pursuit** target. Green dots are sensor returns, including the car’s own surfaces and ground, not semantic object labels. The orange block is a static obstacle used to evaluate stopping.

Overview frames the road; follow mode moves the camera with the observed car pose. Neither camera changes the car’s physics. Road artwork and changing overlays occupy separate OpenUSD layers. These ground-level overlays may enter raw LiDAR returns; they are below this braking fixture’s height filter, not guaranteed sensor-invisible.

<a id="docs-h-zcfls9kncbr4"></a>

### **Who controls what**

Behavior chooses the route and a 3 m/s target. Planning produces an expiring reference. The pure-pursuit and speed controller reads the known map and **privileged simulator odometry**, then commands steering, wheel propulsion and braking. PhysX integrates those commands into vehicle motion; the viewer never writes the car’s pose.

The **LiDAR emergency brake** receives time-checked XYZ points, not object labels or the barrier’s ground-truth pose. It checks a forward corridor and overrides propulsion with braking when a return is inside the stopping range. Braking distance includes speed, scan age and a margin. Obstacle braking latches until a new episode.

The sensor sends **WORLD-frame** returns. The adapter converts them into the current chassis frame using simulator odometry and preserves acquisition bounds, delivery time, identity and coordinate-reference time. This static-obstacle assumption is not moving-object tracking, lane perception or general obstacle avoidance.

<a id="docs-h-ul6a3rh51b5c"></a>

### **One clock and fail-safe data**

Physics runs at 120 Hz, control at 60 Hz, planning at 10 Hz, rendering at 30 Hz and rotary scans at 20 Hz. Both the sensor tick rate and scan rate are set and read back. The first two native initialization steps are recorded before episode tick zero.

Rendering must leave the native physics counter and clock unchanged. Pause withholds physics steps while the view updates. Missing, malformed, future, wrong-identity or stale scans command zero propulsion and full braking. The **oldest sample**—not the newest return or delivery timestamp—must remain within 0.20 simulated seconds.

<a id="docs-h-51m7ga1wo8h3"></a>

### **What testing found and fixed**

The first 10 Hz attempt repeatedly braked because acquisition, delivery and held-scan time exceeded the freshness limit. Shorter 20 Hz scans solved this without relaxing the 0.20-s limit. The same firing rate now gives fewer angular samples per revolution. A NumPy boolean in the result report also needed conversion to a native JSON boolean.

The first GUI point-cloud attempt failed before movement: the debug-writer extension was enabled after sensor construction. The installed sensor copies its writer aliases during construction, so the extension now loads first. A regression test covers registration order and off/on toggles.

Another GUI run passed driving checks but failed visual inspection: already-WORLD points were transformed again for display. The writer now explicitly uses **doTransform=False**, matching NVIDIA’s installed WORLD-frame handling. Raw braking data was separate and correct. A numerical pass is not automatically a visual pass.

<a id="docs-h-hc1et4w7xrsb"></a>

### **Measured results**

Each full obstacle/dropout case ran for 40 simulated seconds and 4,800 physics steps. In the obstacle case, lane-center RMS error was 0.02120 m and maximum error 0.02717 m. Final progress was 62.22481 m; final barrier clearance was 4.24599 m and minimum clearance 4.23066 m. No rigid contacts or lane departure were reported.

The final five-second hold stayed below 1.09e-7 m/s. Maximum sensor mount-position error was 0.001016 m, with every recorded post-settle frame matched. Obstacle-run accepted scan age stayed at or below 0.16667 s. In the frozen-delivery test, full braking began exactly at the expected control tick 1,452 (12.10 s).

Independent review recomputed raw scans: the GUI latch cloud contained 81 forward-corridor returns, nearest about 4.86418 m, inside a 4.88082 m stopping threshold. Ground returns did not enter the brake corridor. All 4,800 recorded physical states and applied controls matched between headless and GUI obstacle runs; sensor delivery itself is not claimed bit-identical.

784 CPU tests passed. Isaac’s Python separately passed all 23 view tests, including two USD checks skipped in the traffic environment. Nine runtime attempts are retained: two numerical failures and one further numerical GUI pass rejected for visual misalignment. Accepted final GUI and overview captures were visually inspected. Source and artifact hashes verified.

<a id="docs-h-aoioc5wjtmhp"></a>

### **Performance and remaining limits**

**NOT REAL-TIME READY:** the unpaced headless obstacle loop took 59.28 wall seconds for 40 simulated seconds, an RTF of 0.675. The final interactive GUI attempt took 91.95 s end to end, including 10.91 s startup and additional pause renders. Its sampled whole-GPU peak was 5,328 MiB, including the desktop; peak process RAM was about 6.17 GiB.

Passing new runs had no runtime Error lines or scene-reference warning. Other startup warnings remain in their logs. This does not retroactively close the older reset/lifecycle investigation. One short low-speed car test does not prove highway stability, fleet capacity, real-time control, general perception or memory endurance.

<a id="docs-h-a86iy7qe2qdh"></a>

### **Portable demonstrations**

From the clone root, activate the configured traffic environment and set isaac\_root in ignored hivemind.local.json or ISAAC\_SIM\_PATH. The launcher selects Isaac’s separate Python. Paste each command as one line and run one GPU job at a time. A compatible installed Isaac runtime and RTX hardware are still required; portability is not a promise that every machine supports rendering.

```text
python scripts/demo_physics_lidar.py --check
```

Checks configuration and runtime discovery without launching a simulation.

```text
python scripts/demo_physics_lidar.py --points
```

Opens the bounded 40-s visible demonstration. In the Highway Sim panel, use Overview, Follow car, Pause/resume, Toggle planned path and Toggle LiDAR points. Use these controls instead of the Kit timeline. The app closes after recording results; wall time may exceed simulated time.

```text
python scripts/demo_physics_lidar.py --headless --mode dropout --unpaced
```

Runs the sensor-loss test without a window. It freezes controller delivery at 12 s while the sensor keeps recording, so the original scan’s expiry can be audited.

```text
python -m json.tool documentation/physics-lidar-view-results.json
```

Displays committed compact evidence. The additional overview evidence is documentation/physics-lidar-overview-results.json. Raw trajectories, point snapshots, scenes, previews and logs remain in local outputs/vehicle\_visual\_lidar and are not included in a GitHub clone.

<a id="docs-h-l3a5fmu6wdtz"></a>

### **Next gates and your role**

This advances bounded parts of roadmap steps 8 and 20. Next: complete the longer physical-sensor/reset checks, measure and improve the control/render workload, then tackle the shared multi-rate scheduler and hybrid bridge before multi-car tests. The 60-s sensing gate, 120-s integrated scheduler and endurance gates remain open.

**YOUR PARTICIPATION:** before Isaac Lab policy training, we will agree together on observations, actions, rewards, algorithm, training budget and held-out evaluation. No training, dependency installation or runtime modification was performed for this milestone.

<a id="docs-h-e3u0omr5ezg0"></a>

### **Source and learning references**

[Implementation and reproduction note](<https://github.com/hudsonkaneko/hivemind-traffic/blob/531f748bef1c2e636515824e7edb5b6462779399/documentation/physics-lidar-view.md>)

[Hash-verified runtime attempts](<https://github.com/hudsonkaneko/hivemind-traffic/blob/531f748bef1c2e636515824e7edb5b6462779399/documentation/physics-lidar-view-results.json>)

[LiDAR safety controller](<https://github.com/hudsonkaneko/hivemind-traffic/blob/531f748bef1c2e636515824e7edb5b6462779399/traffic/lidar_braking.py>)

[NVIDIA sensor timing and multi-tick rendering](<https://docs.isaacsim.omniverse.nvidia.com/6.0.1/sensors/isaacsim_sensors_multitick_rendering.html>)

[GitHub review — physical-car foundation (PR \#1)](<https://github.com/hudsonkaneko/hivemind-traffic/pull/1>)

Source checkpoint: 531f748. Changes are pushed on the development branch; main remains unchanged pending review.

<a id="docs-t-ivzd9eyrxgjj"></a>

## 30 Wheel orientation fix

<a id="docs-h-uxliijl4vzo0"></a>

### **What changed**

The simple physical-car wheels now roll around sideways axles instead of tumbling like horizontal discs. This fixes the separate Isaac/PhysX demonstration; it does not change the SUMO demos, the driver controller, torque limits, suspension tuning or training.

<a id="docs-h-7pzv9q8hshfk"></a>

### **Why the wheels looked wrong**

Our vehicle frame is X forward, Y left and Z up. The original Y-axis cylinders were correct before simulation. Once physics started, the USD wheel attachment acquired an extra 90-degree Z rotation relative to the native wheel pose. That turned the visible axle into the direction of travel and made the wheel tumble.

Measured axle error was about 90 degrees. This supports a cylinder-basis/writeback mismatch in the installed runtime. We have not proved the exact defect in NVIDIA’s internal implementation.

<a id="docs-h-5455jewevf46"></a>

### **The correction**

The wheel is now authored as an X-axis cylinder with an explicit local +90-degree Z rotation. Initial world geometry, radius, width, center and collision APIs remain equivalent; its local extent is recomputed. The runtime folds this rotation into the attachment, so we verify the composed world-space shape rather than assuming the child transform stays unchanged.

PhysX remains the only motion authority. There is no per-frame wheel-pose override. The tire is visible geometry and a collision shape, so this is not just a camera trick; arbitrary tire/obstacle collision accuracy is still outside this test.

<a id="docs-h-ngnvvsfwy130"></a>

### **Evidence and a useful debugging lesson**

The corrected 8-second smoke exactly matched all 960 physical/control states of the original diagnostic. The final 40-second turn-and-brake run exactly matched all 4,800 physical/control states of the previous obstacle baseline. Final barrier clearance remained 4.24599 m, lane RMS error 0.0212006 m, and rigid contacts zero.

The new gate checks all four wheels at every 30-Hz render boundary. The 4,800 wheel samples had a maximum axle-direction error of 0.070414 degrees and composed-orientation error of 0.027559 degrees, both under the predeclared 0.1-degree limit. The orientation check includes wheel spin as well as axle direction.

One intermediate run is retained as failed: my first new test compared raw attachment and native rotations in different frames. Correcting the comparison to combine chassis orientation, native wheel-local pose and geometry basis resolved that false failure without changing the driving behavior or relaxing the threshold.

786 CPU tests passed and 4 were skipped. Isaac’s Python separately passed all 4 wheel tests, including the two new USD checks skipped by the traffic environment. Corrected viewport captures were visually inspected. The original numerical pass is not retroactively a visual pass.

The full run had no runtime Error lines or USD reference warnings. All 760 recorded post-settle sensor frames matched their poses. Its 40 simulated seconds took 62.91 loop wall seconds: this remains a low-speed one-car test, not a real-time or highway validation.

<a id="docs-h-rnerkzw34htt"></a>

### **Show the corrected wheels**

From the clone root, activate the configured traffic environment. Runtime discovery still uses local configuration. Close and restart any already-running demo to load this fix, then use Follow car to watch the wheels.

```text
python scripts/demo_physics_lidar.py --points
```

Run the bounded 40-second turn/brake check and save viewport captures:

```text
python scripts/demo_physics_lidar.py --headless --capture --unpaced
```

Run the small regression test; USD authoring cases require Isaac’s Python and are skipped when USD is unavailable:

```text
python -m pytest tests/test_wheel_geometry.py -q
```

<a id="docs-h-3b9i50sdj204"></a>

### **Key terms to explain**

**Axle**: the line a wheel spins around. **Coordinate frame**: the reference directions used to express a position or rotation. **Transform composition**: combining parent and child placement to obtain world placement. **Regression test**: a repeatable check that a previously fixed problem stays fixed.

Engineering lesson: put both quantities in the same coordinate frame before comparing them, and validate appearance as well as motion. Keep failed attempts so the reasoning can be audited.

<a id="docs-h-pjabmbc65mz6"></a>

### **Sources and scope**

[Implementation, explanation and portable commands](<https://github.com/hudsonkaneko/hivemind-traffic/blob/252c6de1f902da7b8209b7344305aa47876656ff/documentation/wheel-orientation.md>)

[Four hash-verified runtime attempts](<https://github.com/hudsonkaneko/hivemind-traffic/blob/252c6de1f902da7b8209b7344305aa47876656ff/documentation/wheel-geometry-results.json>)

[GitHub review — physical-car foundation (PR \#1)](<https://github.com/hudsonkaneko/hivemind-traffic/pull/1>)

Source checkpoint: 252c6de. Changes are on the development branch; main remains unchanged pending review. Raw pictures, scans, scenes and trajectories stay in local outputs/vehicle\_visual\_lidar and are not included in a clone. No dependency installation, external-runtime edit or training occurred.

<a id="docs-t-cbgq2dwnrach"></a>

## 31 Adaptive obstacle bypass

<a id="docs-h-25lmxr959sub"></a>

### **What you will see**

A separate physical-car demo now detects a static orange obstacle, moves left, passes it, returns to its lane and stops. The cyan line changes from the nominal route to the actual detour commanded by the driver. Pink marks its steering target; optional green points show RTX LiDAR returns. The earlier stop-before-obstacle demo is preserved.

<a id="docs-h-pkldfsbshagl"></a>

### **How the pieces work together**

RTX LiDAR supplies fresh XYZ returns. A scripted planner confirms the obstacle over two distinct scans, combines those returns with known road geometry and an explicit maximum obstacle-size prior, then builds a detour. Pure pursuit follows that path through steering and speed/brake commands. PhysX determines the resulting vehicle movement; the overlay only draws the selected route.

Physics runs at 120 Hz, wheel control at 60 Hz, planning at 10 Hz, rendering at 30 Hz and the configured RTX sensor at 20 Hz. Records retain vehicle and episode identity, acquisition intervals, delivery time and coordinate frames. Odometry is explicitly privileged simulator state; obstacle ground truth is used only to construct and evaluate the scene, not to plan or command the car.

<a id="docs-h-4b4s9am4arcf"></a>

### **What adaptive means here**

The obstacle location is inferred from LiDAR returns, rather than supplied as a hidden barrier coordinate. The confirmed route shifts left to Y=3.8 m and returns to Y=0. The visible preview moves along the next 35 m of that route. After confirmation, the route stays fixed for this static-obstacle episode: this is not continuous replanning around moving traffic.

Assumptions are a known straight two-lane road, an empty adjacent corridor, privileged ego odometry and an obstacle no larger than the declared 4 m by 3 m prior. Sparse returns do not prove unseen space clear. No SUMO connection or training is required, and no pose teleporting replaces wheel physics.

<a id="docs-h-54zxmgpl8v1l"></a>

### **Fail-safe behavior**

An infeasible detour or invalid/stale scan commands braking. Fresh points in the immediate corridor or predicted body envelope latch a stop. The blocked-road test deliberately exceeds the size prior: the planner may initially choose a detour, but the point-envelope brake must stop before the full-width barrier. The dropout test freezes the original packet at 12 simulated seconds, preserving its acquisition time.

<a id="docs-h-g1p695nov3p4"></a>

### **Run this version**

From the clone root, activate the configured traffic Python environment. The launcher discovers the external Isaac installation through local configuration. Commands below are portable and single-line. Close and restart an older demo window to load this version. Runs last 50 simulated seconds, not necessarily 50 wall-clock seconds.

```text
python scripts/demo_obstacle_bypass.py
```

Use Overview or Follow car, Pause/resume, and the path/point toggles. For an overview with the real point cloud:

```text
python scripts/demo_obstacle_bypass.py --camera overview --points
```

Capture a bounded headless pass, or demonstrate the two fault cases:

```text
python scripts/demo_obstacle_bypass.py --headless --capture --unpaced
```

```text
python scripts/demo_obstacle_bypass.py --mode blocked
```

```text
python scripts/demo_obstacle_bypass.py --mode dropout
```

<a id="docs-h-ttuuiko6xzl7"></a>

### **Measured evidence and limits**

The accepted headless pass completed 6,000 physics steps with zero contacts or road departures, 2.14471 m minimum body clearance, 0.15458 m maximum path error and final position X=89.87754 m / Y=0.00212 m. All 959 post-settle sensor frames matched historical poses within 0.000848 m. Correct native wheel-axis/pose errors were below 0.000036 degrees.

Blocked-road and frozen-scan checks also passed, retaining 4.46184 m and 13.43378 m minimum clearance respectively. The dropout case applied full braking at control tick 1452, recorded after the step as row 1453. Each case completed its five-second stopped hold. The suite also passed 823 traffic-environment CPU tests; 10 unavailable-runtime cases were skipped. Isaac’s Python separately passed all 37 view/wheel tests.

An independent replay of retained raw scans reproduced the planner decisions and adopted path exactly without obstacle ground truth. Across two complete runs, 6,000 motion samples matched exactly; only the wheel measurement module changed. This is evidence for those runs, not a universal bitwise-repeatability claim.

The passing loop took 99.77 wall seconds for 50 simulated seconds: about 0.50 times real time. Whole-GPU peak was 6,719 MiB including the desktop. One GUI launch crashed inside Isaac before scene creation; that failure is preserved. Startup reliability, sustained real-time performance, moving obstacles and coordinated physical fleets remain open. The unchanged-code visible-window retry subsequently passed all checks and reproduced the same motion, but ran at about 0.36 times real time. The original 40-second curved braking demo also passed its regression.

<a id="docs-h-uxwzwr1k6cu4"></a>

### **What debugging taught us**

The first attempt stopped safely when sparse returns could not support the original passing clearance. Moving the route slightly farther left and checking the complete reference footprint solved this without shrinking the obstacle prior. A later attempt passed the obstacle but exposed a short-mesh array conversion bug near the finish; explicit USD array types fixed the display.

Two complete runs then failed an inherited wheel check that compared the tilted 3D axle with a direction projected onto the ground. Comparing against the native body-and-wheel axle corrected the measurement while retaining the 0.1-degree limit. The old contact-direction difference remains a separate diagnostic. Failed attempts and their original conclusions are not overwritten.

<a id="docs-h-12zbj9z2tvo9"></a>

### **Key terms to explain**

**Path planning**: choosing the geometric route the driver should follow. **Pure pursuit**: steering toward a look-ahead point on that route. **Motion authority**: the component that actually determines a vehicle’s movement, here PhysX.

**Privileged odometry**: vehicle pose/speed read directly from the simulator rather than estimated from sensors. **Extent prior**: an explicit assumption about the maximum unseen obstacle size. **Fail-safe**: stopping when required information is stale, invalid or insufficient instead of continuing blindly.

<a id="docs-h-c4yyw0d5jn"></a>

### **Sources and next boundary**

[Implementation and portable commands](<https://github.com/hudsonkaneko/hivemind-traffic/blob/07a373bd630e6ea403092d3393b5015f3d9cbedf/documentation/obstacle-bypass.md>)

[GitHub review and validation updates](<https://github.com/hudsonkaneko/hivemind-traffic/pull/1>)

Source implementation checkpoint: 07a373b on the development branch; main remains unchanged pending review. Raw scans, scenes, screenshots and trajectories stay in local outputs/vehicle\_obstacle\_bypass, not in a fresh clone. Next: moving-traffic occupancy checks, two physical cars and communication fallback. The user will participate before Isaac Lab training choices or training runs.

<a id="docs-t-9wkfs52ky6df"></a>

## 32 Demo readiness and profiling

<a id="docs-h-6txwhoqnzvyl"></a>

### **What this milestone delivers**

Three separate demonstrations are now rehearsed: SUMO traffic with scripted or frozen PPO decisions; a full-size physical car performing a scripted LiDAR-triggered bypass; and the existing RC-scale Leatherback waypoint PPO in Isaac Lab. These are not one combined simulator. Existing experiments, trained weights and historical results remain preserved.

<a id="docs-h-ugsi3y3vdgom"></a>

### **The SUMO demonstration**

SUMO determines movement. TraCI connects the Python controller to SUMO; PettingZoo supplies the multi-agent interface. The scripted controller and frozen shared-PPO checkpoint use the same traffic scenario and state-based safety shield. This PPO does not perceive through LiDAR.

```text
python -m hivemind demo sumo --seed 2001 --horizon 400
```

```text
python -m hivemind demo trained --seed 2001 --horizon 400
```

The matched readiness study used seeds 2001, 2002 and 2003 plus a repeat of 2001 for each policy. Both cars finished all eight episodes with zero recorded collisions; repeated metric fingerprints matched exactly. The trained policy required 29, 39 and 43 safety overrides on the three seeds. This is a readiness check, not proof that PPO is better.

```text
python -m experiments.demo_sumo_readiness --headless
```

<a id="docs-h-7epomjc69kbw"></a>

### **The physical LiDAR demonstration**

The full-size simple car follows steering, torque and braking commands; PhysX determines its movement. RTX LiDAR triggers the static-obstacle detour. Cyan shows the actual selected path; Follow and Overview change only the view. The car still uses known road geometry, privileged simulator odometry and an explicit maximum obstacle-size prior. This is scripted, not learned, and does not require SUMO.

```text
python scripts/demo_obstacle_bypass.py
```

```text
python scripts/demo_obstacle_bypass.py --camera overview --capture
```

Three fresh optimized pass-mode starts passed the existing gates. Minimum body clearance remained about 2.14471 m. The blocked-road and frozen-scan cases also passed, retaining approximately 4.46184 m and 13.43378 m clearance. Sensor timing, wheel alignment, route tracking and source-immutability checks stayed enabled.

<a id="docs-h-g3cqww2nxanw"></a>

### **Why it became faster**

Profiling separates the time spent on physics, rendering, safety calculations and logging. The original run spent 26.52 seconds repeatedly rewriting growing logs and 22.11 seconds checking point-cloud safety. Immutable one-second evidence chunks now save only new records during the loop; full final JSON remains available. NumPy evaluates the same safety predicates in batches.

For the same 50 simulated seconds, the headless unpaced loop fell from 92.58 to 45.49 wall seconds, or roughly 0.54× to 1.10× simulation rate. Startup was separate, about 10 seconds. Visible loops took 64.65–67.19 seconds, roughly 0.74–0.77×: the GUI is not yet real time. The follow-camera timing overlapped CPU tests; the isolated overview run is documented separately. No physics/control/sensor rates, driving limits or safety thresholds were reduced.

All 6,000 trajectory ticks and 500 planning decisions were compared. Discrete route/safety decisions matched, and position differences stayed below 0.000008 m. Small LiDAR sampling differences remained; floating telemetry is not universally bitwise identical. Saved scan tests independently matched the scalar safety reference.

<a id="docs-h-akscp9uw1nga"></a>

### **The genuine learned-controller demonstration**

```text
python scripts/demo_waypoint.py --gui --capture
```

This evaluates the existing Leatherback PPO checkpoint, not a newly trained highway controller. Its observations are simulator-state waypoint inputs; its outputs command speed and steering targets for articulated wheels. It does not use LiDAR or V2V. The RC-scale vehicle appears small in the overview, so zoom in during the live demonstration.

Three starts, each with seed 43, one environment and 1,800 control steps, completed 20 successful waypoint episodes with no failures or timeouts. All 180 sampled trace rows matched exactly across headless, visible and repeated evaluation. The checkpoint stayed unchanged. The final launcher records interrupted runs as failures, uses bounded process cleanup, and checks saved source snapshots as well as live source files.

<a id="docs-h-3jet8dcjrz98"></a>

### **Setup, evidence and recovery**

Run each command from the clone root with the configured traffic Python active. The launchers discover the separate SUMO, Isaac and Isaac Lab runtimes. A compatible installed runtime, RTX hardware for Isaac, and trusted local checkpoints are still required. Set ISAACLAB\_PATH if Lab is not in your Documents/IsaacLab folder. A Git clone does not include the ignored weights or raw run packages.

```text
python scripts/demo_waypoint.py --check
```

```text
python scripts/demo_obstacle_bypass.py --check
```

Run one simulator at a time and let it finish before starting the next. Each attempt uses a new evidence folder; inspect its manifest, summary and available runtime log after a failure. Do not replace historical folders. New PNG captures are available as still-image fallbacks; no continuous backup movie was produced in this milestone.

The final CPU suite passed 870 tests; 10 runtime-dependent tests were skipped. Evidence packages are under outputs/demo\_sumo\_readiness, outputs/vehicle\_obstacle\_bypass and outputs/waypoint\_demo. Private local configuration is not copied; resolved commands and runtime paths are recorded. The remote Leatherback asset URI is hashed, not its USD contents.

<a id="docs-h-kbru60r7y36l"></a>

### **Key terms to explain**

**Inference**: using fixed trained weights to choose an action, without updating those weights. **Profiling**: measuring where execution time is spent before choosing an optimization. **Vectorization**: evaluating many points together with array operations while preserving the same predicate.

**Matched seeds**: reusing specified randomness across comparisons, with repeat checks rather than assuming determinism. **Real-time factor**: simulated seconds divided by wall-clock seconds, excluding startup when explicitly stated. **Provenance**: the source, settings, runtime, checkpoint and evidence needed to trace a result.

<a id="docs-h-jikemr7xscs"></a>

### **Source checkpoint and the next boundary**

Implementation checkpoint: 703961b. The feature branch is backed up to GitHub and remains under review; main is unchanged. The full-size physical car, RC-scale PPO and SUMO baseline are distinct. The new highway USD asset has not been integrated into these driving demos. Shared-loop driving, cloned physical environments and new policy training remain open. The user will participate in observations, actions, rewards, training budget and evaluation choices before new Isaac Lab training.

[Demo handoff and portable commands](<https://github.com/hudsonkaneko/hivemind-traffic/blob/703961b10d4219083a3f1bdbd95491ef530a26ec/documentation/demo-readiness.md>)

[Measured performance and equivalence evidence](<https://github.com/hudsonkaneko/hivemind-traffic/blob/703961b10d4219083a3f1bdbd95491ef530a26ec/documentation/demo-physics-performance.md>)

[GitHub review — demo readiness (PR \#3)](<https://github.com/hudsonkaneko/hivemind-traffic/pull/3>)

<a id="docs-t-e9ctk5gfkhg"></a>

## 33 Live preview timing

<a id="docs-h-a6jolht4ml55"></a>

### **Purpose and current status**

The existing one-car obstacle-bypass preview is easier to launch and now has an optional lighter 1x-target mode. Consistent real-time playback in the visible window is **NOT verified**. Every completed GUI trial preserved driving/sensor correctness but failed the declared timing gate. The flag is a target, not a guarantee.

This is a live PhysX vehicle using a scripted controller, RTX LiDAR XYZ, known road geometry, privileged odometry and an explicit maximum-obstacle-size prior. It is not a recording, SUMO-driven movement, a newly trained policy, a vehicle fleet or a highway-speed validation. The car remains limited to 3 m/s.

<a id="docs-h-p2t6ui4zh7nu"></a>

### **Portable preview commands**

Run these complete commands from the repository root after activating your configured traffic Python environment. A compatible Isaac installation and RTX GPU are still required. The launcher discovers the runtime; it does not install it. Run one simulator at a time.

```text
python scripts/demo_obstacle_bypass.py --real-time
```

```text
python scripts/demo_obstacle_bypass.py --real-time --camera overview
```

```text
python scripts/demo_obstacle_bypass.py --real-time --points
```

```text
python scripts/demo_obstacle_bypass.py --real-time --capture
```

```text
python scripts/demo_obstacle_bypass.py --real-time --check
```

```text
python scripts/demo_obstacle_bypass.py
```

The last command preserves the original graphics/render-rate profile. The check command validates settings without launching. Capture saves seven viewport PNGs and one GUI-window PNG, not a movie. Do not combine --real-time with --unpaced. A timing failure returns a nonzero status even if driving succeeded; inspect summary.json.

<a id="docs-h-pyr8tj6g62m7"></a>

### **How to demonstrate it**

Allow Isaac to initialize. Use Overview / Follow car, Pause / resume, Toggle planned path and Toggle LiDAR points in the demo panel. Use the demo pause button rather than the Kit timeline. Cyan shows the adopted route; pink marks the steering target. The bounded run lasts 50 simulated seconds and then saves evidence and closes. Startup/final saving add wall-clock time.

Watch the Playback line: it shows the measured simulation/wall-clock ratio and accumulated lag. In lightweight mode, the full adopted cyan route stays visible and changes when the obstacle planner adopts a detour. Rendering is less smooth at 20 Hz; the vehicle is not artificially sped up.

<a id="docs-h-wxvdir58z2jd"></a>

### **Technology and unchanged boundaries**

The process-local lightweight profile uses MinimalRendering, textured-diffuse shading, a 960 x 600 viewport, FXAA, eight CPU workers and VSync disabled. Overview framing is cached; full-route geometry changes only with the route; the pink target uses a USD transform. The original mode retains its rolling path preview. No installed runtime, driver preferences or dependencies were changed.

Physics remains 120 Hz, vehicle control 60 Hz, planning 10 Hz and LiDAR acquisition 20 Hz. Rendering changes from 30 to 20 Hz only in the optional mode. No physics ticks are skipped. Render calls must advance zero physics ticks. Asynchronous rendering remains disabled. Ground-level debug meshes may enter raw scans and are not claimed sensor-invisible.

<a id="docs-h-2uaq7y26a3hh"></a>

### **Definitions worth explaining**

**Real-time factor (RTF)**: simulated elapsed seconds divided by active wall-clock seconds. 1.0x means one simulated second per real second; it does not mean the vehicle drives faster.

**Accumulated lag**: how far the current simulation time trails its wall-clock deadline. A good whole-run average can hide slow driving followed by catch-up while stopped.

**Fixed timestep**: physics advances by the same simulated time increment regardless of computer speed. Pacing waits when ahead and reports lag when behind; it does not reset the clock to hide stalls.

**Soft-real-time gate**: the entire expected episode must complete, with RTF 0.99–1.01, p95 accumulated lag at most 0.10 seconds and maximum lag at most 0.50 seconds. Startup and explicit pauses are excluded, but settling, captures and checkpoint work are included. This is not a hard-deadline guarantee.

<a id="docs-h-e1m2ccqwi2gs"></a>

### **Evidence and what happened**

A headless pass-mode run completed 50 simulated seconds in 50.013 active wall seconds, with p95/max lag 0.015/0.293 seconds: it passed the timing gate. A comparable visible-window run took 50.416 seconds overall but accumulated p95/max lag 4.897/5.062 seconds: it failed. A later GUI run without screenshots took 57.350 seconds and also failed. No visible run has demonstrated consistent 1x yet.

Exploratory trials compared quality, thread counts, display waits, UI visibility and path-geometry caching. Hiding UI removed demo controls without solving timing; extra display-wait overrides were not retained. One native startup crash was preserved. Desktop load varied, so these are measured exploratory outcomes, not clean causal benchmarks.

The final 20-Hz blocked-road and frozen-scan tests preserved every physical/sensor safety gate: stop before the obstruction and brake on stale data. Their timing gates still failed, despite near-1x averages. Automated checks passed: 880 traffic-environment tests (11 skips) and 31 USD road-view tests in Isaac Python. An aborted-prefix timing-report edge case found in review was fixed and tested.

<a id="docs-h-5odfbm8et26q"></a>

### **What remains and where to look**

The unresolved work is native window/render profiling and matched-load repeats to make the visible preview consistently meet the same timing gate. This does not change the mixed-traffic research roadmap or start new Isaac Lab training. New training still requires your direct involvement.

Each run keeps a unique outputs/vehicle\_obstacle\_bypass folder with resolved settings, source hashes, timing samples, physical/sensor evidence and logs. Large outputs remain local, not in a GitHub clone. The repository report lists every attempt and its limitations; the previous guide tabs retain their historical results.

[Preview implementation and evidence](<https://github.com/hudsonkaneko/hivemind-traffic/blob/e74b574/documentation/realtime-preview.md>)

<a id="docs-t-9ah0315nta8b"></a>

## 34 Loop traffic showcase

<a id="docs-h-4in2rt2s4p2t"></a>

### **Loop traffic showcase — v04 and v05**

Updated October 10, 2026. A scripted presentation branch, not an Isaac Lab training result. Earlier tabs and historical experiments remain unchanged.

<a id="docs-h-86k3b3vov30w"></a>

### **What changed and why**

v04 established a working prepared Americano v07 main car with 12 slower block cars, synchronized moving visuals, an adaptive cyan path, and follow/traffic/overview cameras. v05 increases the background fleet to 80 cars: 20 per lane, arranged in staggered pairs around the full loop. Alternating gaps provide repeated passing opportunities without reserving a permanently empty lane.

The main car targets 35 mph; background cars travel approximately 15–15.33 mph. The planner changes lanes only when its gap checks permit it, otherwise following or braking. Existing v04 commands remain the lighter default.

<a id="docs-h-evacna74pxjz"></a>

### **How the pieces interact**

**Dynamic PhysX**: the main car moves through simulated physics in response to steering and wheel drive/brake commands. Its running position is not teleported to the planned path.

**Kinematic background**: block cars follow scripted circular targets with real collision shapes. They do not represent learned or calibrated human drivers. Shared angular speed preserves their spacing over successive laps.

**Privileged observations**: the scripted controller reads timestamped simulator object tracks and a known road map. This showcase does NOT drive from LiDAR, use PPO, or demonstrate hivemind communication.

Timing: physics 120 Hz, low-level control 60 Hz, planning 10 Hz. v05 displays at 10 Hz versus v04 at 20 Hz. Native physics observations stay independent of display updates. Display rate is not simulation speed.

**OpenUSD composition**: reusable vehicle geometry and collision assets are referenced under named branches. Render-only background proxies mirror measured physics poses. Live display edits stay in a disposable layer; supplied assets remain unchanged and local-only.

<a id="docs-h-z60t66jrumb3"></a>

### **Portable demonstration commands**

Run from the repository root in the configured traffic Python environment. Isaac Sim and the separately supplied Americano v07 asset must already be available; a Git clone alone does not provide that asset or the GPU runtime.

```text
python scripts/demo_loop_showcase.py --version v05
```

Denser four-minute drive, followed by a bounded stop. Use the live camera controls, or start with the closer overhead view:

```text
python scripts/demo_loop_showcase.py --version v05 --camera traffic
```

Shorter dense preview and lighter real-time-qualified comparison:

```text
python scripts/demo_loop_showcase.py --version v05 --drive-seconds 90
```

```text
python scripts/demo_loop_showcase.py --version v04
```

An independent check resolves configured paths without starting the simulation:

```text
python scripts/demo_loop_showcase.py --version v05 --check
```

<a id="docs-h-4h7f7gpvkow"></a>

### **Evidence and limitations**

v04’s qualified GUI run completed 252 simulated seconds in 252.0003424 active wall seconds, with 12 passes, five lane changes, no contacts and safe braking. Two fresh repeat runs matched on recorded physical state. This older result does not establish v05 performance.

The v05 90-second-drive test (20261010T210628Z-ad519ed9) completed 16 passes and eight lane changes, with no contact or overlap, 1.5693 m minimum clearance and successful braking. Complete physical/visual checks and unchanged sources passed. It simulated 102 seconds including settling/stopping in 130.012 active wall seconds: 0.785× real time, NOT real-time qualified.

**Real-time factor (RTF)** means simulated seconds divided by active wall-clock seconds. An RTF of 0.785 means the live demo runs slower than real time; pacing cannot make a slow machine faster. Physical success, visual synchronization and real-time performance are separate claims.

<a id="docs-h-j8fhlhx8mid8"></a>

### **Problems found and engineering decisions**

Dense rendering exposed a native first-frame crash in the installed prerelease Isaac runtime. A documented, process-local legacy renderer switch avoids that failure on this workstation; the exact vendor root cause remains unproven. No installed runtime or persistent preference was changed.

One exploratory layout trapped the main car beside a neighbor; reversing the outer-pair stagger restored passing opportunities. Another run passed traffic safely but collided during shutdown when a nonreactive car reached the parked main car. v05 now explicitly decelerates the entire background fleet at the demonstration endpoint and verifies a stationary hold. This is demo rundown, not autonomous cooperation.

Conservative geometry pruning reduces expensive clearance calculations without ignoring any vehicle. Removing duplicate hidden meshes and batching display edits reduces overhead without removing colliders. These changes did not make the measured 80-car preset real-time.

Detailed source, retained failures and qualification records: pull request \#10 and documentation/dense-loop-showcase.md in the repository. Isaac Lab training remains postponed and requires the user’s involvement.

[`https://github.com/hudsonkaneko/hivemind-traffic/pull/10`](<https://github.com/hudsonkaneko/hivemind-traffic/pull/10>)

<a id="docs-h-tazfct1og9g6"></a>

### **Final qualification — October 10**

The full GUI test (20261010T211050Z-5d35f8f5) completed 46 overtakes and 23 lane changes over 3,537.17 m, without contact or overlap. Minimum clearance was 1.5693 m, followed by a safe stop and hold. It simulated 252 seconds in 335.418 active wall seconds (0.7513×), so the denser version remains NOT real-time qualified.

That long test preceded a shutdown-only workaround. A later repeat and command-loss trial completed their driving checks but crashed while an unused sensor-icon GUI helper released its USDRT scene handle. They remain failed trials. v05 now disables only that optional icon helper for the demo process; actual physics, rendering and sensor computation are not disabled. A fresh GUI smoke then closed cleanly.

Two final 90-second runs (20261010T212511Z-6495f5ba and 20261010T212738Z-aa5d3a93) each passed 16 vehicles, completed eight lane changes and shut down cleanly. The evidence comparator verified identical inputs and zero position/speed difference across 12,240 aligned physics samples, plus 1,020 complete visual/native checks for all 80 cars. This establishes same-machine repeatability, not universal determinism or guaranteed freedom from future native crashes.

Final command-loss, blocked-lane and deliberate-contact checks also passed with clean shutdown. The deliberate collision is a positive control proving contact detection works, not an avoidance success. The finished regression suite passed 1,384 general tests (119 optional skips, 18 subtests) and 100 USD tests.

Use the v05 commands above for denser traffic; keep v04 for the lighter real-time-qualified demonstration. No training was started, no vehicle assets were changed or published, and the main research roadmap remains intact. Full run IDs, failed attempts and exact commands are retained in documentation/dense-loop-showcase.md.

<a id="docs-t-dwsxfv73kedy"></a>

## 35 Showcase performance

<a id="docs-h-f9n5x8iuc002"></a>

### **Showcase performance — v06**

Updated October 10, 2026. A lighter presentation of the same 80-car showcase; v04/v05, source assets and historical results are preserved. This is not a training milestone.

<a id="docs-h-85z7g8boxqjk"></a>

### **What changed and why**

We first measured the live loop, then removed redundant work: repeated reads of the same completed physics state, duplicate camera/path updates, unnecessary distant decorative wheel spinning, and repeated road-projection calculations. All 80 background collision bodies and displayed body poses remain present. OpenUSD references, named hierarchies and disposable live layers are preserved.

Physics and safety sampling remain at 120 Hz, low-level control at 60 Hz and planning at 10 Hz. The main Americano still targets 35 mph. Only v06 opts into the optimized controller and display path; older versions remain available.

<a id="docs-h-hpgw5wwbjw6e"></a>

### **Definitions and engineering tradeoffs**

**Profiling:** measuring where execution time goes before choosing an optimization. In the instrumented v05 baseline, render/app-update used 34% of loop time, physics 17%, controller 10% and safety/metrics 9%. Render/app-update includes Kit/UI/scene synchronization; it is not exclusively GPU drawing.

**Shared measured state:** consumers reuse one immutable native-body snapshot for a completed physics tick. The next commanded step invalidates it. Requested target poses are never substituted for measured physical observations.

**Presentation versus full evidence:** presentation keeps complete 120 Hz ego/safety rows in memory, but samples full object records and optional native/display agreement audits at 1 Hz, plus first-audit samples. A native crash may lose buffered evidence. Full mode retains 120 Hz native audits, every-display visual checks, 10 Hz object records and durable per-second chunks.

**Full-rate safety:** contacts, measured vehicle state, clearance, road containment, wheel support, attitude and command expiry are not reduced. Sampling diagnostic agreement checks is not equivalent to disabling collision detection.

**Real-time factor (RTF):** simulated seconds divided by active wall seconds. Finishing at 1x does not prove smooth playback if the simulation falls behind and later catches up. Display rate is separate: 10 Hz is not 60 FPS.

<a id="docs-h-xrzn2cwk7ike"></a>

### **Measured results**

The first matched 90-second-drive comparison reduced active loop time from 125.768 to 101.528 seconds (about 19%). Both included 102 simulated seconds with settling/stopping, 16 overtakes, eight lane changes and 1.5693 m minimum clearance. Recorded positions, speeds and controls matched exactly across 12,240 physics samples. This isolated first-iteration comparison is not a universal speedup guarantee.

Two final four-minute GUI drives each completed 46 passes and 23 lane changes over 3,537.17 m, with zero contact/overlap, 1.5693 m minimum clearance, a 10.85 m final stop and clean shutdown. Their 30,240 physics samples matched exactly; both trajectory hashes are identical. Visual/native agreement was checked at all 253 scheduled audit frames per run, not every rendered frame.

**Strict smooth real-time qualification remains OPEN.** Each final run finished 252 simulated seconds in about 252 wall seconds, but p95 accumulated lag was 1.652 s and 0.194 s against a 0.1 s limit. Maximum frame intervals were 305/311 ms against a 200 ms limit. No threshold was loosened. The 20/30 Hz display trials also failed to sustain the requested timing; 10 Hz remains the v06 default.

Full-evidence mode separately passed a 90-second drive with every-tick/every-display audits and 102 durable chunks. A fresh follow-camera screenshot was inspected and showed the prepared car, moving traffic and cyan planned path correctly. Command-loss braking, a 90-second blocked-lane fixture and deliberate-contact detection passed. The deliberate collision proves detection, not avoidance.

Retained failures matter: a 15-second blocked fixture was safe but still approaching traffic during its final measurement window, so it failed the following-speed criterion; the 90-second trial passed. Another physical pass had a competing GPU application and is excluded from isolated timing claims. After the user closed it, measured idle GPU usage dropped from roughly 92% to 3%. No unrelated app was closed by the agent.

<a id="docs-h-sioagaq68o7f"></a>

### **Portable commands**

Run from the repository root in the configured traffic Python environment. Compatible Isaac Sim and the separately supplied Americano v07 asset must already be available. Neither is included by simply cloning GitHub.

```text
python scripts/demo_loop_showcase.py --version v06
```

Four-minute paced presentation with follow, traffic-overhead, whole-loop, pause and planned-path visibility controls. It is usable but not strict smooth-playback qualified.

```text
python scripts/demo_loop_showcase.py --version v06 --evidence full --drive-seconds 90
```

Full audit/recording mode; expect additional overhead.

```text
python scripts/demo_loop_showcase.py --version v06 --camera traffic
```

```text
python scripts/demo_loop_showcase.py --version v06 --check
```

The original no-version launcher still selects the lighter twelve-car v04. Experimental --render-hz 20 or 30 overrides are not qualified defaults.

<a id="docs-h-i5a0gu89s2cg"></a>

### **Evidence, scope and next step**

Final regression suite: 1,483 passed, 140 optional skips, 18 subtests; separate fleet/view USD suite: 76 passed. Same-source repeat report: 20261011T005532Z-1d7514f6. Long runs: 20261011T004030Z-5e517202 and 20261011T004825Z-591ff392. Full-evidence capture: 20261011T004609Z-827df6f2. Raw results and screenshots remain local and ignored by Git.

The showcase still uses a scripted controller and privileged simulator object tracks, not LiDAR-derived driving, RL or hivemind coordination. Backgrounds are nonreactive kinematic obstacles, not human-driver models. No training, dependency installation, vendor-file changes or vehicle publication occurred.

Next: trace the remaining render/app-update sub-stages and test scene/UI synchronization or immutable visual instancing separately. Preserve physics, safety, references and wheel access. Do not turn a timing improvement into a claim of sensor or traffic-policy effectiveness.

Repository source and complete evidence: documentation/showcase-performance.md and showcases/loop\_traffic/v06/README.md. Pull request \#10 remains a review branch; main and the research roadmap are unchanged.

[Review implementation and evidence (PR \#10)](<https://github.com/hudsonkaneko/hivemind-traffic/pull/10>)

<a id="docs-t-jndashuyzqn1"></a>

## 36 Responsive showcase

<a id="docs-h-akzh32dss3fc"></a>

### **Responsive showcase — v07**

October 10, 2026. An opt-in presentation milestone for the existing 80-car loop. Earlier versions, source assets and experimental evidence are preserved. This is not an Isaac Lab training or LiDAR-driving milestone.

<a id="docs-h-az1bz2ax3msm"></a>

### **What changed and why**

**OpenUSD instancing:** 80 background cars now share immutable body and wheel geometry. Four body-color prototypes plus one wheel prototype serve 400 instance roots, preserving all 1,280 visible geometric primitives. Car transforms and wheel-spin pivots remain individually editable. The 80 real background collision bodies and the dynamic Americano ego are unchanged.

**Regular editor UI:** at the user's request, v07 now defaults to the normal Isaac Sim editor, including Stage/Properties, menus and toolbar, alongside the existing showcase controls. Shared visual geometry, cached camera/path metadata, skipped redundant writes and 5 Hz statistics remain enabled. The minimal app is optional with --ui-profile showcase; performance is measured separately for each interface.

**Separate rates:** physics and safety still run at 120 Hz, low-level controls at 60 Hz and planning at 10 Hz. V07 requests 30 display updates per simulated second, targets 35 mph for the ego and retains the slower background traffic. No extra hidden physics steps, reduced collision checks or DLSS frame generation are used.

<a id="docs-h-m83hf8t72h51"></a>

### **What the measurements mean**

**Frame budget:** 60 FPS allows about 16.7 milliseconds per displayed frame; 30 FPS allows about 33.3 milliseconds. Low overall GPU or CPU utilization does not show which serial task is holding up the application.

**Native frame delivery:** we count renderer completion events separately from application updates and simulated time. GPU-completion notifications may arrive zero, one or several times per app update; unique frame identities and monitor scanout are not independently verified. This does not measure actual mouse-to-photon latency. Event-service intervals show opportunities for the application to handle input, not a verified click response time.

**Real-time factor (RTF):** simulated seconds divided by wall-clock seconds. A fast-looking frame counter is not enough if the simulation itself falls behind. The 60-Hz trial delivered about 50 native FPS but only 0.83x real-time speed; it remains experimental.

**Profiling:** a bounded optional diagnostic captures installed native CPU/GPU zones and restores its settings. Zones are inclusive and may refer to different completed frames, so nested times must not be added together. One diagnostic sample showed a 12.95 ms CPU-side Hydra render bracket and a 0.88 ms GPU RTX root zone, supporting scene synchronization/work as a bottleneck. Profiling and user-paused runs cannot qualify smooth display.

<a id="docs-h-qib72rn5p455"></a>

### **Validation and current limits**

Historical minimal-app 30-FPS smoke: run 20261011T012749Z-655ffebd completed 27 simulated seconds in 27.000 wall seconds, with 30.04 native delivered FPS, 34.65 ms p95 delivery interval and 97.66 ms maximum. Physical, scene, clean-process and strict display gates passed. This short test is not a long-run guarantee.

Historical minimal-app long-run result: final-source GUI runs 20261011T013722Z-f3410868 and 20261011T014156Z-5131a70f each completed 252 simulated seconds at 1.000x average real-time speed with 46 passes, 23 lane changes and zero contacts. All 30,240 position/speed samples repeated exactly. P95 renderer-notification intervals were 34.72 and 34.56 ms; maximum intervals were 126.00 and 127.97 ms, so the strict 100 ms hitch-free gate remains OPEN. This is an improved 30-Hz preview, not qualified 60 FPS. Full-evidence run 20261011T014828Z-168a9ad4 passed 12,240 native and 3,060 display audits; its fresh 55-second capture was visually inspected. Driver-command expiry (20261011T015050Z-d4f69476), blocked traffic (20261011T015120Z-31a91cc9) and intentional-contact detection (20261011T015204Z-f9ac4d2a) also passed. These headless failure controls do not qualify GUI performance; driver-command expiry is not a V2V-network test.

Editor UI follow-up (October 10): the regular editor is now the v07 default without removing shared geometry, cached updates, 5 Hz statistics or the physics/control settings. All 247 targeted regression tests passed. Fresh default-editor paced run 20261011T032144Z-ca68b556 passed physical, scene and process checks with Stage, Property and Toolbar extensions enabled: 16 passes, eight lane changes, zero contacts and safe stopping. Its 102 simulated seconds took 107.225 wall seconds (0.9513x), with 28.54 renderer notifications/s and a 39.68 ms p95 notification interval. Strict smoothness remains open; this single check does not establish long-run repeatability or click latency. The regular UI is retained as requested despite the measured overhead.

Regression checkpoint: the final general suite passed 1,575 tests, with 179 optional skips and 18 subtests; 115 USD fleet/view tests passed. The read-only review also led to bounded profiling during pause and explicit exclusion of headless runs from GUI qualification. All earlier runtime failures remain local evidence. The 32-worker trial was slower than eight workers; a four-worker 60-Hz trial still missed real time, so the default remains eight.

<a id="docs-h-zd8ctbt7f08e"></a>

### **Run it yourself**

Run these one-line commands from the highwaysim repository root with the configured traffic Python environment active. The supervisor locates Isaac Sim. The separately supplied Americano v07 remains a local prerequisite; cloning GitHub does not include that vehicle asset.

```text
python scripts/demo_loop_showcase.py --version v07
```

Optional minimal showcase interface (the regular editor is now the default):

```text
python scripts/demo_loop_showcase.py --version v07 --ui-profile showcase
```

For every-tick/every-display diagnostic audits and durable evidence chunks:

```text
python scripts/demo_loop_showcase.py --version v07 --evidence full --drive-seconds 90
```

Experimental 60-Hz test; do not describe this as validated 60 FPS:

```text
python scripts/demo_loop_showcase.py --version v07 --render-hz 60 --drive-seconds 15
```

Optional bounded native diagnosis (adds overhead and cannot qualify FPS):

```text
python scripts/demo_loop_showcase.py --version v07 --profile-render --drive-seconds 15
```

<a id="docs-h-6ofulbxk213i"></a>

### **Evidence, provenance and next boundary**

Repository notes: documentation/showcase-responsiveness.md and showcases/loop\_traffic/v07/README.md. Development branch: codex/loop-traffic-showcase. Review: [`https://github.com/hudsonkaneko/hivemind-traffic/pull/10`](<https://github.com/hudsonkaneko/hivemind-traffic/pull/10>). Raw runs, screenshots and vehicle assets stay local under outputs/ and vehicles/; GitHub records code, tests and compact evidence notes. No runtime installation or persistent settings changes were made.

Presentation buffers full-rate ego/safety records but samples optional native/display agreement audits. Full evidence is more expensive and has durable checkpoints. Neither mode changes driving safety cadence. A future 60-FPS improvement must reduce remaining scene/physics synchronization work without silently changing the experiment. Sensor-enabled performance, real input latency and longer memory-growth studies remain separate gates.

<a id="docs-t-q3vcdoqqkcjq"></a>

## 37 Live showcase controls

<a id="docs-h-zabs1775qpms"></a>

### **Live showcase controls — v08/v09**

October 12, 2026. Purpose: explore traffic density and driving settings without repeatedly restarting Isaac Sim. This is a scripted presentation milestone, not LiDAR driving, hivemind intelligence or Isaac Lab training.

<a id="docs-h-11xu8gsp535o"></a>

### **What changed and why**

**v08 foundation:** a native, docked Highway Sim Controls panel in the regular Isaac editor. Camera and overlay controls work live; traffic and rendering choices use a supervised safe-stop/restart. v08 remains available as a preserved milestone.

**v09 live requests:** density, main-car target speed, background speed and following gap now change within the same episode. The sidebar distinguishes requested density from actual on-road population. Existing cars are not redistributed, and the episode clock does not reset.

<a id="docs-h-8n37yuaty9u0"></a>

### **What you can adjust now**

**Background cars:** 4–80, plus one physics-driven main car. Light, Medium and Dense presets request 12, 40 or 80 background cars. Cars enter/leave gradually, at up to four per simulated second, outside a protected 160 m arc around the main car. Low targets can take longer while nearby cars remain protected.

**Driving targets:** main car 5–35 mph, background inner lane 5–25 mph, preferred following gap 1.8–3 seconds. Outer-lane speeds scale to maintain common angular spacing. Lower background speed first when reducing both speeds; it cannot exceed the main-car target in this nonreactive showcase.

**Ramping:** background speed changes by at most 0.5 m/s²; main-car target by 1 m/s²; preferred following gap by 0.25 seconds per second. Actual ego acceleration depends on the driver and physics. Changes wait through committed lane changes; new lane changes wait while background speed or population is transitioning. A protected population can remain above the requested count until safe removal is possible; lane following continues. Paused driving changes apply on resume.

**Viewing controls:** follow distance/height, path-preview length, statistics refresh, decorative-wheel animation distance, camera choice, path visibility and pause. These do not reduce physical collision checking.

**Restart-only controls:** rendering cadence and resolution. The explicit Restart rendering button brakes and holds, closes Isaac cleanly, then launches a fresh process with a new evidence folder. Unsafe shutdowns do not auto-retry.

**Future controls:** 26 disabled sliders are labeled DISPLAY ONLY / Not implemented. AV share, communicating AVs, driver variability, LiDAR, V2V and advanced dynamics are not secretly active parameters.

<a id="docs-h-j5p2jyf2bvsg"></a>

### **How the implementation works**

**Object pool:** 80 existing USD/native-physics slots stay bound for the whole run. Membership changes instead of deleting or spawning new prims or rebuilding physics handles. Dormant bodies move to a remote parking area and their visible proxies are hidden; they are not invisible obstacles left on the road.

**Identity generations:** re-entering traffic gets a new logical vehicle ID while reusing the same USD slot. Timestamped lifecycle events explicitly tell the controller which vehicles entered or left. Unannounced membership changes remain errors.

**Bounded mailbox:** slider callbacks store the latest requested values; the simulation consumes them at step boundaries. This avoids native physics or USD work on every slider pixel. Statistics and event storage are bounded.

**OpenUSD ownership:** reusable geometry stays referenced/instanced, static asset files are immutable, and live transform/visibility opinions stay in the runtime layer. The hierarchy does not change when density changes. Physics/control/planning remain 120/60/10 Hz.

<a id="docs-h-gkwtv3ktqcgo"></a>

### **Run and demonstrate**

From the repository root in the configured traffic Python environment:

```text
python scripts/demo_loop_showcase.py --version v09
```

On this workstation, open a terminal in C:\\Users\\hudso\\Documents\\highwaysim, then run:

```text
.venv-traffic\Scripts\python.exe scripts\demo_loop_showcase.py --version v09
```

Try Light → Medium → Dense. Watch actual density catch up without restarting, then adjust speed or following gap. The default bounded demo closes after four minutes of driving plus settling/braking. Prepared Americano v07 and Isaac remain local prerequisites; cloning GitHub does not supply the vehicle asset.

Native callback repeat test, hidden from desktop input:

```text
python scripts/demo_loop_showcase.py --version v09 --drive-seconds 90 --control-smoke --headless --panel-hidden --unpaced
```

<a id="docs-h-7sm3yqwya7vf"></a>

### **Validation and important limits**

**Verified live-control sequence:** two isolated 102-second native runs changed 80 → 12 → 40 → 12 cars without restarting. Both passed contact, clearance, road containment, braking/hold, visible/native-pose, source-preservation and shutdown checks. All 12,240 physics samples and lifecycle/control events repeated exactly with matching source hashes. Minimum clearance was 1.607 m; braking distance 1.524 m. The car followed safely in this tuning sequence (zero overtakes/lane changes), so this is not an overtaking qualification. Final regression: 1,593 passed, 180 optional skips, 18 subtests; separate USD suites: 122 passed. Evidence: 20261013T002704Z-70c5c8fc and 20261013T002856Z-6b410449.

**Performance boundary:** pooling avoids repeated loading/rebinding but retains the 80-body memory/physics allocation even at low density. Lower density is not a promise of proportionally lower cost. Interactive runs and hidden tests do not qualify visible smoothness, click latency or 60 FPS.

**Experimental boundary:** the main car uses PhysX steering/wheel torques and privileged simulator-state object tracks. Background cars follow scripted kinematic targets and do not react like human drivers. Live tuning changes demand during an episode, so it is exploratory rather than a matched fixed-configuration experiment. No new training or sensor pipeline was started.

<a id="docs-h-i3o2xpk7s2sd"></a>

### **Code and deeper study**

**Repository note:** documentation/showcase-live-controls.md. Settings/mailbox: experiments/showcase\_controls.py. Native widgets: visualization/showcase\_sidebar.py. Membership and ramps: traffic/live\_showcase.py. Native pool adapter: usd/live\_showcase\_fleet.py. Version folders: showcases/loop\_traffic/v08 and v09.

[Draft review and source history](<https://github.com/hudsonkaneko/hivemind-traffic/pull/10>)

[**NVIDIA PhysX:**](<https://nvidia-omniverse.github.io/PhysX/physx/5.4.0/docs/RigidBodyDynamics.html#kinematic-actors>) [kinematic actor motion and teleportation](<https://nvidia-omniverse.github.io/PhysX/physx/5.4.0/docs/RigidBodyDynamics.html#kinematic-actors>)

<a id="docs-t-124coz3q4xks"></a>

## 38 Project skills in GitHub

<a id="docs-h-ccq4xkg6fjew"></a>

### **Project skills in GitHub**

October 12, 2026. Purpose: keep the project's reusable agent workflows versioned with its implementation, so another checkout can use the same reviewed guidance.

<a id="docs-h-65bgq96cp2d7"></a>

### **What changed**

Eight domain skills now live under .agents/skills: sumo-traci-traffic, pettingzoo-multiagent-traffic, train-hivemind-marl, stream-traffic-openusd, build-omniverse-traffic-viewer, validate-av-isaac-lab, run-reproducible-av-experiments and prepare-simready-vehicle.

Each has instructions and selector metadata. Vehicle preparation includes two reusable references. The source manifest tracks 18 files and original hashes; installed personal copies remain unchanged.

<a id="docs-h-uehgfqn4ojnl"></a>

### **Key concepts and boundaries**

**Skill:** reusable instructions for a coding agent, not an executable controller, a trained policy or a runtime installation.

**Repository scope:** older SUMO-first guidance applies to SUMO-backed experiments. PhysX motion ownership, current project rules, local-only asset restrictions and the user-participation gate before training still apply.

**Provenance:** the manifest records where the selected text came from and what was adapted. It does not grant licenses for vehicle models, software or referenced documentation.

**Excluded:** the private vehicle case study and machine paths, assets, checkpoints, credentials, unrelated personal skills and bundled provider/plugin files. Cloning these skills does not install SUMO or Isaac dependencies.

<a id="docs-h-xhukuehva1t1"></a>

### **Use and check**

Open the repository in Codex and select the repo-local skill. This example is a Codex prompt, not a terminal command:

```text
Use $sumo-traci-traffic to review our existing SUMO baseline without changing it.
```

From the repository root, in the existing Python test environment:

```text
python -m unittest discover -s tests -p test_project_skills.py -v
```

Same-name installed and repo-local skills are not merged; both may appear in selectors. Select the repository copy. Restart Codex if a new skill is not visible. Future updates are reviewed Git changes, not an automatic sync of personal settings.

<a id="docs-h-zctlwzldn3nh"></a>

### **Validation and limitations**

Six repository checks passed: inventory/provenance, skill metadata, selector metadata, local references, path/credential screening and private-case-study exclusion. All eight skills also passed the installed skill-creator validator using existing read-only PyYAML; no dependency was installed.

This validates packaging, not future agent behavior or simulation performance. No simulator, sensor or training run was required, and no simulation code or results changed.

<a id="docs-h-ec5y94k2rd89"></a>

### **Source and deeper study**

Repository inventory and usage: documentation/project-skills.md. Original-source manifest: documentation/project-skills-manifest.json. Checks: tests/test\_project\_skills.py.

[Draft review and source history](<https://github.com/hudsonkaneko/hivemind-traffic/pull/10>)

[Official Codex skill discovery guidance](<https://learn.chatgpt.com/docs/build-skills?product=breeze>)

<a id="docs-h-1bye2clpc51w"></a>

### **October 14 update — merged into main**

All ten project pull requests are now merged into main through PR 10, preserving the original commits and feature branches. Main commit: `da05a6a42ddc8872ef678c6584e77828ec8d9f67`. The eight skills and all committed simulation milestones are now available from the default branch; local main matches GitHub.

Fresh integration checks: 1,599 general tests passed (180 optional/runtime skips, 34 subtests), 134 USD checks and four route checks passed, and the v09 launcher preflight passed. The two merge conflicts were documentation-only; no simulation source or asset changed. This is not a new physics, GUI-performance or training qualification.

Private vehicle assets, credentials, raw outputs and uncommitted local-only vehicle notes remain excluded. Historical failure/acceptance limits remain valid; merging does not complete the pending roadmap gates.

[Merged review and integration evidence](<https://github.com/hudsonkaneko/hivemind-traffic/pull/10>)

<a id="docs-t-mriyksl2h83o"></a>

## 39 Clean-clone readiness

<a id="docs-h-fnw1dp7egwxc"></a>

### **Clean-clone readiness**

October 15, 2026. Purpose: let another developer install the published project and demonstrate it without borrowing the maintainer’s virtual environments, private car assets or saved policies.

<a id="docs-h-z6rzz0gm3c"></a>

### **What changed and why**

A Windows / Python 3.11 clean-install profile now includes SUMO 1.27.1 and the existing pinned traffic stack. GitHub Actions installs it on a fresh hosted runner and executes real SUMO integration tests plus the CPU suite. This catches missing dependencies and checkout assumptions that a working development machine can conceal.

The loop showcase now accepts --vehicle public: a simple PhysX block car generated from the installed NVIDIA Factory sample through our existing layered USD builder. The main car remains physics-driven; background cars are scripted kinematic objects. No private Americano assets or vendor source were published. Existing commands still select the prepared car by default.

Missing required showcase inputs now fail with status 2 and useful setup instructions. SUMO can be discovered beside the traffic Python without activating the environment. Tests no longer share a fixed temporary directory, and Git preserves the exact bytes of the manifest-hashed highway source.

<a id="docs-h-bgxbqy775mbt"></a>

### **Key ideas**

**Clean clone:** tracked repository files plus a new dependency environment, without relying on local-only assets, configuration or outputs.

**Continuous integration (CI):** automated installation and tests on every applicable push or pull request. A green CPU job is not a GPU, sensor or performance certification.

**Preflight:** inexpensive checks for required files before launch. Passing --check does not prove a compatible driver, valid USD composition or correct vehicle dynamics.

**Explicit vehicle profile:** a visible model choice recorded in configuration and evidence. Different car models are not interchangeable treatments in a matched experiment.

<a id="docs-h-9pmohtm1aywq"></a>

### **Install and demonstrate SUMO**

Windows x64; install Git and Python 3.11 first. PR 11 was merged into main on October 15, 2026 (edb7125). A fresh default-branch clone was verified to contain the exact tested source tree. Run each command on its own line:

```text
git clone https://github.com/hudsonkaneko/hivemind-traffic.git highwaysim
```

```text
cd highwaysim
```

```text
py -3.11 -m venv .venv
```

```text
.venv\Scripts\python.exe -m pip install -r requirements-ci.txt
```

```text
.venv\Scripts\python.exe -m pip check
```

```text
.venv\Scripts\python.exe -m pytest -q
```

```text
.venv\Scripts\python.exe -m hivemind demo sumo
```

<a id="docs-h-f1zb1yp86l0f"></a>

### **Demonstrate the public Isaac car**

Install Isaac separately and set isaac\_root in ignored hivemind.local.json, using the provided example file. The tested standalone runtime is Isaac Sim 6.0.1-rc.7 on Windows with an RTX 5070; other runtime versions need qualification. Do not install traffic requirements into Isaac’s bundled Python. These commands use the traffic interpreter to launch Isaac in a separate process:

```text
.venv\Scripts\python.exe scripts/demo_loop_showcase.py --version v09 --vehicle public --traffic-count 12 --check
```

```text
.venv\Scripts\python.exe scripts/demo_loop_showcase.py --version v09 --vehicle public --traffic-count 12 --drive-seconds 90
```

For the scripted sidebar validation used in this milestone, run the following. It automatically requests 80 → 12 → 40 → 12 background cars with speed/gap changes. This tests real native control models, not physical mouse-click latency:

```text
.venv\Scripts\python.exe scripts/demo_loop_showcase.py --version v09 --vehicle public --drive-seconds 90 --control-smoke --capture
```

<a id="docs-h-4c9og8mb9s0n"></a>

### **Verified outcomes and retained failures**

Published source: `8b01a93585dc5ad318466e1a68fb446ffbcf616b`. A fresh clone from GitHub with a new traffic environment passed 1,605 tests, with 180 explicit optional/runtime skips and 34 subtests. The independent hosted Windows CI run also passed those totals and four separately invoked real SUMO checks. Dependency consistency passed; the clone contained no vehicle package or local configuration.

The public car passed a 27-second headless physics smoke test and two 102-second GUI runs. Both GUI runs passed motion, braking, road containment, wheel support, clearance, live population controls, immutable scene inputs and clean shutdown. Their complete 12,240-row trajectories and traffic lifecycle records matched exactly. Minimum measured vehicle clearance was 1.811 m. Neither GUI run demonstrated overtaking or lane changes; those are separate qualification scenarios.

A relocated generated scene reopened with all 19 layer dependencies inside the copied package and no unresolved assets; its structure checks passed. Another 134 existing USD tests passed. The short smoke test’s first attempt failed because the safety guard needed a real inspection of the procedural vehicle source; that inspection was added, not bypassed. The initial CPU run also exposed a test that wrongly assumed its temporary directory was inside the repository; its fixture is now explicit. Failed evidence remains retained.

Performance is not newly certified. GUI active-time ratios were 0.9789× and 1.0000× real time, but both failed the strict lag/smoothness criterion. The first overlapped CPU validation; captures also add overhead. No 60 FPS, large-fleet, sensor-driven autonomy, real-world calibration or training claim follows from these runs. The GPU tests reused the installed runtime/hardware, not a second clean machine. No Isaac Lab training or external runtime changes occurred.

<a id="docs-h-qf3y2omi69mv"></a>

### **Source, evidence and next boundary**

The repository guide contains evidence IDs, exact commands and support limits. Raw local results, images and checkpoints are not included in Git. CPU CI does not make trained/replay/ovrtx/Lab demos turnkey; those still need explicitly documented external inputs. The prepared vehicle is still local-only. Repository vehicle/viewer workflows kept scene composition and motion ownership intact; reproducibility checks supplied isolated-install and repeat-run evidence.

[Review the implementation and validation](<https://github.com/hudsonkaneko/hivemind-traffic/pull/11>)

[Independent Windows CI result](<https://github.com/hudsonkaneko/hivemind-traffic/actions/runs/38765009932>)

[Repository setup and detailed evidence](<https://github.com/hudsonkaneko/hivemind-traffic/blob/main/documentation/clean-clone.md>)

<a id="docs-h-qn81753kl192"></a>

### **Windows Temp installation test — October 15, 2026**

A second clean checkout was installed directly under Windows Temp at published commit `e31b34a66db38d98f118108518bbf36f6a3231e3` (the PR 11 branch at test time; subsequently merged into main as edb7125). Its new Python 3.11 environment passed dependency consistency and 63 targeted setup/launcher/configuration tests. Python, TraCI, sumolib and SUMO resolved inside that environment. No private car package or existing project environment was copied. The full CPU suite was not rerun in this additional copy.

The first attempt, 20261015T180043Z-2210d3c9, failed with a 120×0 render-window/swapchain error and undocked sidebar. Driving and shutdown completed, but it was not accepted as a working viewer. The failed evidence remains preserved; its root cause is unconfirmed. No source, driver or runtime changes were made before the retry.

The unchanged-source retry, 20261015T220110Z-c47ce2fb, passed all 31 simulation gates and exited cleanly without runtime errors or unresolved-asset warnings. With 80 scripted background cars, the public PhysX car completed 16 passes and 8 lane changes; minimum clearance was 1.780 m. Five viewport images were saved and a capture was visually inspected. Braking, road containment, wheel support, camera, sidebar and scene/source immutability checks passed.

Performance remains separate: 102 simulated seconds took 116.396 active wall seconds (0.876× real time); the strict real-time criterion failed. This does not establish 60 FPS. The test reused this PC’s Isaac Sim 6.0.1-rc.7 runtime, RTX 5070 and caches, so it is not a fresh-machine or universal-compatibility certification. No LiDAR, learning, runtime installation or private Americano asset was involved. The canonical project and its pre-existing edits remain untouched.

Reproduce from the configured Temp checkout (or another configured clone); run-preview.cmd in the test folder uses the same bounded command. Local evidence and TEST-RESULTS.txt remain in the Temp test folder and may eventually be removed by Windows cleanup:

```text
.venv\Scripts\python.exe scripts/demo_loop_showcase.py --version v09 --vehicle public --drive-seconds 90 --capture
```

<a id="docs-t-lz1t1b5s17js"></a>

## 40 Showcase validation

<a id="docs-h-7ye2nbnmkn2d"></a>

### **Showcase validation — startup, controls, statistics and timing**

October 16, 2026. Purpose: make the existing public-car showcase easier to launch, test its live controls, expose defensible descriptive measurements and measure presentation costs without changing the driving experiment. This chapter distinguishes implemented safeguards from verified native behavior and performance.

<a id="docs-h-pms4vdefiszn"></a>

### **Scope and the four changes**

**Ground-truth scripted showcase:** one PhysX-owned public procedural main car, a scripted steering/propulsion/braking controller and up to 80 scripted nonreactive kinematic backgrounds. The controller and statistics see native simulator state and a known circular map. This is not LiDAR driving, SUMO human traffic, a learned policy or an autonomous research fleet.

The work covers bounded startup readiness, opt-in endpoint/pause validation, descriptive traffic statistics and a predeclared display-cost matrix. The regular Isaac editor remains the default. Existing vehicle dynamics, physics/control/planning rates, contact/clearance/road checks, source integrity and clean-shutdown requirements remain in force. Public and prepared cars are different physical models and must not be substituted within matched comparisons.

<a id="docs-h-elif8wfyg95"></a>

### **1. Startup readiness: what is checked**

**Observed readiness:** after render warm-up, require three consecutive samples with positive native-window dimensions, a visible viewport with positive computed dimensions and actual sidebar docking when the panel is visible. Requested render resolution alone is not proof that a usable window exists.

The bounded helper pumps render-only lifecycle frames, with the existing session guard verifying no physics advancement. It permits one restore/resize attempt after three invalid native extents, retries docking at most once every four lifecycle frames and stops after 120 updates or 15 wall seconds. A resize request is not itself success. Headless and hidden-panel checks have explicitly narrower requirements.

**Cause still unconfirmed:** the retained 120-by-0 swapchain failure occurred during SimulationApp construction, before the project sidebar and this readiness gate. The source-identical retry succeeded, but that does not establish the cause. Existing 1440-by-900 window defaults are recorded explicitly; process-local saveSizeOnExit=false prevents a repair from persisting window size. No runtime installation, display/DPI choice or saved user layout is modified.

Every attempt retains startup-readiness.json, including failure reasons, sampled dimensions and actions. Earlier renderer errors still reject the run even if later dimensions become valid. Native repeated launches and readable captures remain required; a unit-test pass cannot qualify the viewer.

Declared execution order: three fresh captured 15-second driving smokes, inspect each viewport and stop on failure; then the 180-second endpoint run and a same-configuration repeat; finally the eleven-case performance matrix. Capture work is separate from the baseline matrix. These runs are a protocol, not completed evidence.

<a id="docs-h-ilpc119ymmx4"></a>

### **2. Live controls: endpoints, pause and recovery**

**Endpoint scenario:** start with 12 backgrounds, 10 mph main target, 5 mph inner-lane traffic and a 2.4 s preferred gap. At simulation time 5 s request 4 cars / 5 mph / 5 mph / 3 s by 14 s. At 15 s, while paused, request 80 / 35 / 25 / 1.8 by 50 s. At 55 s request 12 / 35 / 5 / 3 by 100 s. At 105 s request 80 / 35 / 15 / 1.8 by 135 s. Times include the two-second settle.

Four UI-only pause frames exercise 64 rapid population edits and nine speed/gap edits through the actual native widget models. The latest-value mailbox must apply one coalesced event only after resume; native time/step counters, vehicle pose/speed, traffic state and applied settings must remain unchanged while paused. A traffic target above the main target must be rejected and its model restored.

Each entry or removal must retain measured separation of at least 160 m and spacing of at least 30 physics ticks. Between 135 and 180 s, the settled 80-car tail must complete an additional pass and lane change after the final transition. Successful callbacks or earlier passes do not satisfy these gates. This changes settings deliberately and cannot also serve as a fixed-demand timing comparison.

**Protected minimum population:** the 4-car endpoint is tested from 12, while dense removal tests 80 to 12. More than four cars can remain inside the protected zone at high density; if their relative geometry does not change, an 80-to-4 request can stay pending. A negative CPU fixture verifies that protection is retained. The panel shows requested and actual counts separately; no protected car is deleted to force a deadline.

<a id="docs-h-21s43ks1go6a"></a>

### **3. Measurements: definitions and limits**

**Native state at 120 Hz:** geometry uses each actual collision box, including the main car's model-specific offset. Ego speed is native chassis planar velocity magnitude; background speed is native velocity projected onto body-forward direction. Only active generation-aware vehicle IDs are included. Drive intervals exclude settling, forced final braking, missing intervals and dormant slots. Sidebar refresh frequency does not change collection frequency.

**Measured headway:** geometric circular bumper gap divided by measured ego speed, with actual lane radius and footprint overlap. Front-to-front headway is also reported. A stationary ego or absent eligible leader gives null time headway. A leader is considered within 150 m forward center-arc distance; these ego-only observations are not the requested following-gap setting.

**Acceleration and jerk:** finite differences of scalar speed, with jerk based on the separation of acceleration-interval midpoints. These are tangential approximations, not signed body-frame acceleration or lateral ride-quality measurements. Noise can be amplified. Hard braking uses the declared project threshold of at most -3.0 m/s².

**Near-miss proxy:** one ego-level episode when nearest exact clearance is below 1.0 m or closing TTC is below 1.5 s; report both events and exposure duration. Nonclosing TTC is null. The TTC model assumes current circular motion and does not predict future lane changes. Close stationary pairs can trigger the clearance proxy, so these are not proven avoided crashes or a replacement for contact checks.

**Time loss and fairness:** integrate max(0, 1 - measured speed / applied unconstrained reference speed) over observed drive time. Deliberate target ramps are respected; overspeed earns no credit. Per-lifetime time-loss fractions, their range and Jain's index of achieved fractions are reported overall and by ego/background role. Lifetimes below one second are excluded from fairness but retained in the ledger. Exits and run-ended incomplete observations remain visible; re-entry gets a new generation.

This descriptive time loss is not origin-to-destination trip delay or proof of traffic-caused delay. Across-role fairness mixes a physical main car with scripted backgrounds and is not evidence of a cooperative benefit. Laps, detector passages, overtakes and lane changes are not completed trips. The collector uses bounded running moments rather than full 120 Hz history; summaries cannot reconstruct every raw background measurement.

<a id="docs-h-rqxl4lrzpw7d"></a>

### **4. Matched performance: qualify 30 Hz and real time**

**Fixed driving experiment:** public car, seed 101, 120 Hz physics, 60 Hz control, 10 Hz planning, eight CPU workers, 90 s drive plus 2 s settle and 10 s braking, paced GUI and statistics enabled. Retain all physical, scene and shutdown gates. No control changes, pauses, CPU-test overlap, competing simulator jobs or native profiling are allowed in qualified timing runs; desktop load is recorded.

The 11-case matrix compares 4- and 80-car v09 editor runs with visible/hidden panels; dense hidden 30 Hz versus 20 Hz; dense hidden presentation versus captures versus full recording; a separate v07 editor/minimal-app pair; and two additional dense-visible repeats. Low-density v09 still carries the fixed 80-slot native pool. Density tiers are separate scenarios, not a matched intervention. Desktop GPU, Kit-process and CPU/RAM observations are retained, but these snapshots cannot exclude every transient background job. One sample per changed variant supports descriptive differences rather than statistical or causal improvement claims.

**Matching and acceptance:** paired runs must have matching source hashes, runtime, assets, physical settings and per-tick controls, with position/speed differences at most 0.02 m / 0.02 m/s and all simulation gates passing. Strict timing requires real-time factor 0.99–1.01, p95 lag at most 0.10 s, max lag at most 0.50 s, native-delivery p95 at most 1.1/requested-Hz and max 100 ms, plus event-service p95 50 ms and max 100 ms. These native notifications do not prove monitor scanouts or mouse-to-photon latency. No 60 FPS claim follows from this study.

<a id="docs-h-ww7phef8xe5k"></a>

### **Portable checks and run commands**

Run from the repository root in the configured traffic Python environment. The public car avoids private vehicle assets. The preflight command does not launch Isaac. The remaining commands start bounded native checks and should run only when competing GPU work is clear.

```text
python scripts/demo_loop_showcase.py --version v09 --vehicle public --check
```

```text
python scripts/demo_loop_showcase.py --version v09 --vehicle public --drive-seconds 15 --capture
```

```text
python scripts/demo_loop_showcase.py --version v09 --vehicle public --control-validation endpoints --drive-seconds 180
```

```text
python -m experiments.showcase_benchmark --plan
```

```text
python -m experiments.showcase_benchmark --run
```

```text
python -m experiments.showcase_benchmark --compare outputs/showcase_benchmark/<run-id>/index.json
```

Use --no-statistics to disable descriptive collection in v09, or --statistics to enable it in supported earlier comparison profiles. Keep this choice identical across timing pairs. The benchmark's default and --plan modes only print the eleven-case matrix. --run executes it serially and refuses to launch when other Kit processes are present or their inventory is unavailable; it never closes another application. Physical, source, resource or cleanup failures stop later cases; a strict timing failure alone does not. Replace &lt;run-id&gt; with the new benchmark folder when comparing.

**Valid comparison versus timing pass:** the evaluator verifies the benchmark index hash, per-run source/output hashes, declared settings, physical gates, control/trajectory equivalence and raw timing samples. Its overall passed flag requires all eight comparisons to be valid. Check both\_strict\_display\_qualified separately for timing success. Missing, stopped or altered runs cannot make an incomplete matrix pass.

<a id="docs-h-898rbjxhn340"></a>

### **Verified evidence and open gates**

Final frozen-source CPU regression: 1,712 tests passed, 180 optional tests skipped and 98 subtests passed in 98.70 seconds. Independent review found no remaining actionable defects after integration fixes and was tied to the committed snapshot across all 62 recorded source hashes. The 50 benchmark-evidence tests are included. Filesystem-only endpoint preflight and whitespace checks passed. This verifies CPU behavior and setup, not native Isaac imports, viewer behavior or performance.

Native status: blocked by four other-project Kit sessions in the recorded prerequisite snapshot. No new Highway Sim native simulation or training was launched for this milestone. The three startup smokes, two endpoint runs and eleven timing cases remain unexecuted; no new startup-fix, performance improvement, 30 Hz or 60 FPS qualification is claimed.

Published checkpoint: commit `3a8cea4f29be4129469dfcd20ad5c5b89c8d0a86` on codex/showcase-validation, with [draft PR 12](<https://github.com/hudsonkaneko/hivemind-traffic/pull/12>) open for review. For that exact implementation commit, [GitHub clean-clone push run 38945979178](<https://github.com/hudsonkaneko/hivemind-traffic/actions/runs/38945979178>) passed: 4 real-SUMO tests in 3.75 seconds and the full CPU suite with 1,712 passed, 180 optional skips and 98 subtests in 139.77 seconds. This immutable run does not assert current-head CI status. The implementation is not merged. CPU reports, source hashes, benchmark plan and the read-only native-prerequisite snapshot are retained locally under outputs/showcase\_validation/cpu-final-20261016T181237Z/; these ignored raw reports are not GitHub assets.

Failed attempts remain evidence. Native acceptance, reset/endurance/memory qualification, reactive SUMO integration and sensor-derived control are separate gates. Before new Isaac Lab training, agree with the user on observations, actions, rewards, algorithm, budget and evaluation. No new training is authorized by this presentation milestone.

Source notes: [showcase validation milestone](<https://github.com/hudsonkaneko/hivemind-traffic/blob/3a8cea4f29be4129469dfcd20ad5c5b89c8d0a86/documentation/showcase-validation.md>); [startup readiness](<https://github.com/hudsonkaneko/hivemind-traffic/blob/3a8cea4f29be4129469dfcd20ad5c5b89c8d0a86/documentation/showcase-startup.md>); [live-control validation](<https://github.com/hudsonkaneko/hivemind-traffic/blob/3a8cea4f29be4129469dfcd20ad5c5b89c8d0a86/documentation/showcase-control-validation.md>); [measured statistics](<https://github.com/hudsonkaneko/hivemind-traffic/blob/3a8cea4f29be4129469dfcd20ad5c5b89c8d0a86/documentation/showcase-statistics.md>).

<a id="docs-h-qxm9lx454ga3"></a>

### October 19, 2026 — native validation and bounded control recovery

Measured checkpoint: This cycle completed 20 native runs: nine functional validation trials and 11 display-matrix trials. Startup, admission recovery, the bounded gap endpoint, its exact recorded-behavior repeat and legacy compatibility passed within their declared scopes. Both earlier failed endpoints remain preserved. All nine functional trials failed strict display timing; in the matrix, the two four-background cases passed and all nine dense cases failed. A separate corrected reassessment accepts all eight matrix comparisons without any native rerun.

The native-blocked paragraph above records the October 16 checkpoint. It remains historical evidence. This dated addition preserves both failed endpoints and reports later outcomes separately; it does not rewrite the earlier failures.

<a id="docs-h-e8m9ijyat6fb"></a>

#### Scope, source and CPU evidence

The native trials use one public procedural PhysX car, scripted nonreactive kinematic backgrounds, privileged simulator state and a known circular map. They do not establish sensor-driven autonomy, SUMO integration, learned control or general traffic safety. No training was launched.

Runtime: Isaac Sim 6.0.1-rc.7+release.42383.32955d8d.gl; supervisor Python 3.11.9; Windows build 26200; GeForce RTX 5070, 12,227 MiB, driver 610.88. The functional trials retained seed 101, physics 120 Hz, control 60 Hz, ordinary planning 10 Hz, requested display 30 Hz, eight CPU workers and presentation evidence. Each trial adds two settling and ten final-brake seconds to its driving duration.

Published checkpoint `7940a9757356fa2dcd6790cc8c1769e0d0ba1fbc` contains the final functional and benchmark documentation. The gap trials recorded admission commit 396f7ef plus frozen traffic/live\_showcase.py SHA256 7b3778e9635422f7773f63cde5ecee308d5fb4f48a49312363913d245d0d4042. The matrix used runtime checkpoint 3a3185a; its later evaluator correction was captured separately. Historical manifests retain their own source hashes and working-tree patches; a pristine checkout is not assumed.

CPU and review passed: the final corrected-evaluator local suite passed 1,767 tests, with 181 optional skips and 116 subtests, in 86.23 seconds. The earlier gap-controller checkpoint passed 1,750 tests before the native trials. Independent source, behavior and publication reviews found no remaining blocking finding. These results apply to their recorded source hashes. Both clean-clone Windows CPU/SUMO CI jobs for published commit `7940a9757356fa2dcd6790cc8c1769e0d0ba1fbc` passed: [CI run 39299088357](<https://github.com/hudsonkaneko/hivemind-traffic/actions/runs/39299088357>) and [CI run 39299079995](<https://github.com/hudsonkaneko/hivemind-traffic/actions/runs/39299079995>).

Published sources: [measured report](<https://github.com/hudsonkaneko/hivemind-traffic/blob/7940a9757356fa2dcd6790cc8c1769e0d0ba1fbc/documentation/showcase-validation-20261019.md>); [compact evidence JSON](<https://github.com/hudsonkaneko/hivemind-traffic/blob/7940a9757356fa2dcd6790cc8c1769e0d0ba1fbc/documentation/evidence/showcase-validation-20261019.json>); [gap-creation design](<https://github.com/hudsonkaneko/hivemind-traffic/blob/7940a9757356fa2dcd6790cc8c1769e0d0ba1fbc/documentation/showcase-gap-creation.md>); [draft pull request 12](<https://github.com/hudsonkaneko/hivemind-traffic/pull/12>).

<a id="docs-h-fixhrvly647j"></a>

#### Nine completed native trials

| **Trial** | **Driving s** | **Functional result** | **RTF** |
| --- | --- | --- | --- |
| S1: startup | 15 | Passed | 0.903791 |
| S2: startup | 15 | Passed | 0.931452 |
| S3: startup | 15 | Passed | 0.944900 |
| E1: original endpoint | 180 | Failed | 0.957291 |
| L1: admission legacy | 90 | Passed | 0.999995 |
| E2: admission endpoint | 180 | Failed | 0.952755 |
| G1: gap endpoint | 180 | Passed | 0.950207 |
| G2: gap endpoint repeat | 180 | Passed | 0.954552 |
| L2: gap legacy | 90 | Passed | 0.999998 |

All nine strict-display results are failed. Exit code zero and verified cleanup are separate from functional acceptance; E1 and E2 remain failed even though their scene, contact and process checks passed.

Run IDs: S1 = 20261019T161716Z-e514b86e; S2 = 20261019T161922Z-d44d31d2; S3 = 20261019T162041Z-7f6eec77.

Run IDs: E1 = 20261019T162155Z-fb9cb018; L1 = 20261019T164220Z-7fd95c16; E2 = 20261019T164433Z-563caac3.

Run IDs: G1 = 20261019T170934Z-049ddcb3; G2 = 20261019T171541Z-e2c26f1d; L2 = 20261019T171947Z-9f61a18c.

Startup passed in three fresh launches: each passed 32/32 run gates with four readiness updates, one dock request and three stable confirmations. No resize repair was needed. Native extent was 1440 × 900, viewport output 960 × 600 and viewport window 949 × 577. Corresponding scheduled viewport images and trajectories were byte-identical. The historical 120-by-0 swapchain cause remains unconfirmed; three short launches do not establish endurance reliability.

<a id="docs-h-pi2xx3b9dc2q"></a>

#### Failures, admission repair and bounded gap creation

E1 failed its unchanged deadlines: requested counts 4/80/12/80 yielded actual 4/75/12/77 at 14/50/100/135 seconds. The settled tail remained at 77 backgrounds with no new pass or lane change. The whole run had one pass and no lane change. Zero contacts and 1.844801 m minimum clearance do not complete the requested control behavior.

The admission repair searches alternate remote dormant entry phases when the existing phase cannot be admitted safely. It preserves active vehicle poses and retains safety vetoes. E2 then reached every 4/80/12/80 deadline and held 80 backgrounds in the tail, but still added no pass or lane change there. Its overall result remained failed despite three earlier passes and one lane change. L1 passed the simpler 12/40/12 legacy scenario, which does not require passing.

The separate gap supervisor preserves the ordinary planner, full-cruise candidate forecasts, emergency authority and existing clearance/tracking limits. After two seconds of settled blocked following, a bounded preflight may permit a target-speed cap and ordinary recovery. Limits are three attempts, 16 examined formations, 12 hypothetical queries, at most 11 seconds slowing and 21 seconds total, and at most 50 m added lag. Measured lead progress defines lag. A fresh rear reserve assumes no ego progress for the remaining window and checks full footprints under the declared background-motion envelope. Invalid tracking, reserve or recovery raises a retained finite failure certificate; no active car is moved to make an opening.

G1 and G2 each passed 33/33 native gates and all nine control subgates. Each completed 11 passes and five lane changes overall; the settled 135–180-second tail held 80/80 cars and added eight passes and four lane changes. Minimum clearance was 1.779726 m, with zero reported contacts and verified cleanup.

One attempt began at 123.8 s, released its cap at 134.3 s and handed off to a newly accepted ordinary maneuver at 138.2 s: 10.5 s slowing, 3.9 s recovery and 14.4 s total. Measured handoff lag was 34.725721 m. Maximum reconstructed lag was 38.373119 m, below the 50 m limit; this uses the 120 Hz ego trajectory and declared background angular motion, corroborated by saved lead diagnostics, not a saved 120 Hz lead track. Maximum radial/heading errors were 0.015890 m/0.003148 rad. The cap target floor was 2.5 m/s; actual speed undershot to 2.469845 m/s, so it is not a physical-speed floor.

<a id="docs-h-50ifrqa2c06o"></a>

#### Control settings, metrics and repeatability

| **Deadline s** | **Actual / requested backgrounds** | **Ego / traffic target m/s** | **Preferred gap s** |
| --- | --- | --- | --- |
| 14 | 4 / 4 | 2.2352 / 2.2352 | 3.0 |
| 50 | 80 / 80 | 15.6464 / 11.1760 | 1.8 |
| 100 | 12 / 12 | 15.6464 / 2.2352 | 3.0 |
| 135 | 80 / 80 | 15.6464 / 6.7056 | 1.8 |

Independent trajectory readback matched every deadline field exactly in both gap endpoints. Four saved paused frames preserved the simulation clock, state and counters; pause adds no metric exposure. Invalid target combinations remained rejected. The requested minimum of four is protected by pending transitions and geometry vetoes, not an unconditional guarantee for arbitrary control sequences. The tested endpoint sequence is the verified scope.

Statistics are drive-only, sampled at 120 Hz; settling and the final forced brake are excluded. Ego acceleration and jerk are derivatives of measured scalar planar speed, a tangential approximation, not body-frame signed acceleration or lateral ride quality. Measured geometric bumper gap/headway is separate from the preferred following setting. Headway is undefined for stationary ego or an absent leader; nonclosing TTC is undefined. These are not fabricated finite observations.

| **Drive-only metric** | **G1 and G2** | **L2** |
| --- | --- | --- |
| Ego exposure, s | 179.991667 | 89.991667 |
| Mean ego speed, m/s | 9.723022 | 7.409489 |
| Ego time loss, s | 53.357063 | 32.170187 |
| Time-loss fraction | 0.296442 | 0.357480 |
| Minimum observed headway, s | 1.943188 | 1.903976 |
| Headway observation exposure, s | 134.400000 | 89.958333 |
| Minimum closing TTC, s | 5.634473 | 7.096361 |
| Hard-braking / near-proxy episodes | 0 / 0 | 0 / 0 |

Time loss integrates max(0, 1 − measured speed / current declared free-flow reference) over observed drive exposure. The 14.4-second attempt contains 10.390451 seconds of total time loss against the unchanged reference; that is not incremental causal cost versus continued following. Near-miss proxy thresholds remain clearance below 1 m or closing TTC below 1.5 s; hard braking is scalar acceleration at or below −3 m/s². Zero proxy episodes do not establish general safety.

Each gap endpoint had 220 lifecycle events: 144 entries and 76 exits, 156 background lifetimes plus ego, 76 retired and 81 run-censored, maximum generation three. The seven alternate-entry reconstructions passed against decimated native snapshots; dormant slots add no exposure. L2 had 124 events and 109 total lifetimes, with 96 retired and 13 censored. These are observed lifetimes and pass events, not completed trips or detector flow. Scripted backgrounds dominate the aggregate fairness value, which is not evidence of a fair interactive traffic policy.

The original and repeat passed 94 independent consistency checks each; L2 passed 88. G1/G2 configuration, all 23,040 trajectory rows, statistics, 220 lifecycle events, four native control events and control-validation results were byte-identical. All 192 saved planner frames matched after per-run episode identity was omitted; the normalizer also excludes wall-named fields, and wall-clock equality is not claimed. The repeated result supports this source and scenario, not general determinism.

Legacy compatibility: L2 passed 33/33 gates with no recorded deliberate attempt. Its 12,240-row trajectory, configuration, 124 lifecycle events and three controls are byte-identical to L1. Its 102 planner frames match after per-run episode identity and the new gap\_creation diagnostic object are excluded. The only changed captured runtime source is traffic/live\_showcase.py. Zero passes and lane changes are expected for this smoke.

<a id="docs-h-e4g9umjp7ry1"></a>

#### Timing and the completed matched matrix

| **Run** | **Active wall s** | **Lag p95 / max s** | **Strict display** |
| --- | --- | --- | --- |
| G1 | 202.061221 | 10.782066 / 11.590994 | Failed |
| G2 | 201.141406 | 9.917673 / 10.665637 | Failed |
| L2 | 102.000223 | 1.405118 / 1.472237 | Failed |

Functional success is not timing success. The unchanged real-time limits require RTF 0.99–1.01, lag p95 at most 0.1 s and maximum lag at most 0.5 s, with preview interval p95 at most 1.5/requested-Hz seconds and maximum 200 ms, alongside native notification and service-interval checks. L2’s near-one RTF and about 30 notifications/s do not override its lag/frame failures. Kit frame notifications are not monitor scanouts or measured click-to-photon latency. Captured startups and interactive endpoints are diagnostic trials, not matched presentation-cost comparisons.

Completed matrix: 20261019T172621Z-63aa7c17 ran all 11 cases from frozen runtime source. Every individual physical/scene/process result passed: 32/32 gates for v09 and 31/31 for v07. Both light cases passed strict display timing; nine dense cases failed. All trials had empty display-qualification exclusions. The independent audit checked 1,063 recorded output hashes and 15 conditions per trial. These are observed native trials, not a 60 FPS or monitor-scanout claim.

| **Case** | **Hz** | **RTF** | **Lag p95, s** | **Strict display** |
| --- | --- | --- | --- | --- |
| light-visible | 30 | 0.999997 | 0.000506 | Passed |
| light-hidden | 30 | 0.999996 | 0.000511 | Passed |
| dense-visible | 30 | 0.892322 | 13.529411 | Failed |
| dense-hidden | 30 | 0.890838 | 13.740176 | Failed |
| dense-20hz | 20 | 0.999999 | 0.202518 | Failed |
| dense-captures | 30 | 0.892760 | 13.551578 | Failed |
| dense-full | 30 | 0.720950 | 39.196332 | Failed |
| v07 editor-cost | 30 | 0.913485 | 11.274404 | Failed |
| v07 minimal-cost | 30 | 0.995487 | 2.548458 | Failed |
| dense-repeat-1 | 30 | 0.890113 | 13.815437 | Failed |
| dense-repeat-2 | 30 | 0.879450 | 14.897049 | Failed |

The original evaluator rejected all eight pair comparisons because vehicle.scene\_composition.assembly\_directory differed between fresh output packages. That failed result remains unchanged. The correction validates each exact own-run public scene directory, then normalizes only that packaging field; assets, physics, references and other composition semantics remain checked. Reassessment 20261019T175418Z-a5a6a122 used evaluator SHA256 9a388b68ebbac79e9575f45bb4dd1b0d7fcfea3652b142bdc2d36b5e7e135f1d and accepted all eight pairs, with zero maximum position/scalar-speed difference and exact per-tick controls. No native run was repeated.

Observed active-wall changes below use 100 × (second / first − 1). Directions are explicit. These are individual observations under recorded local desktop load, not statistical or causal effects. Only the light-panel pair has both strict-display results passed.

| **Comparison** | **First → second** | **Wall change** | **Both strict** |
| --- | --- | --- | --- |
| Light panel | hidden → visible | −0.000102% | Yes |
| Dense panel | hidden → visible | −0.166316% | No |
| Display cadence | dense hidden 30 Hz → 20 Hz | −10.916080% | No |
| Capture | dense hidden off → on | −0.215307% | No |
| Evidence | presentation → full | +23.564560% | No |
| Editor | v07 minimal → editor | +8.976857% | No |
| Dense repeat 1 | baseline → repeat 1 | +0.248151% | No |
| Dense repeat 2 | baseline → repeat 2 | +1.463621% | No |

Dense panel and capture differences are smaller than both recorded baseline repeat changes; they do not show that a visible panel or capture improves performance. The larger observed recording/cadence/editor differences do not remove dense timing failures. Pre/post Kit inventories were quiet, but transient and non-Kit desktop work is not ruled out. No physics, control, safety or statistics cadence was reduced.

Late-stall evidence: five dense v09 30 Hz presentation cases have over-200 ms preview intervals at the same 28.4333 and 67.2667 simulated seconds. Their largest intervals are about 225–228 ms, while first frames are about 44–50 ms; driving-segment RTF is roughly 0.85–0.87. A startup-only explanation is inconsistent with the late spikes and growing lag. The v07 editor has a separate 2.2124-second preview outlier at 89.3 seconds. Near-one total RTF in the 20 Hz/minimal cases masks lag or later braking catch-up.

Next measurement: collect bounded, tick/subcall-indexed native/Sdf publication and CPU/allocation/GC evidence around 28.4333 and 67.2667 seconds, and inspect the separate editor render bracket. Aggregate publication maxima lack frame identity and cannot identify the cause. Preserve all cadences and thresholds; no speculative optimization is reported as verified.

Benchmark sources: [completed matrix report](<https://github.com/hudsonkaneko/hivemind-traffic/blob/7940a9757356fa2dcd6790cc8c1769e0d0ba1fbc/documentation/showcase-benchmark-20261019.md>); [compact benchmark evidence](<https://github.com/hudsonkaneko/hivemind-traffic/blob/7940a9757356fa2dcd6790cc8c1769e0d0ba1fbc/documentation/evidence/showcase-benchmark-20261019.json>). The original failed comparison and corrected reassessment remain distinct in both.

<a id="docs-h-27noqz4o9cms"></a>

#### Screenshots and reproduction

Original captures: [startup editor](<https://github.com/hudsonkaneko/hivemind-traffic/blob/7940a9757356fa2dcd6790cc8c1769e0d0ba1fbc/documentation/images/showcase-validation-20261019/showcase-startup-editor-20261019.jpg>); [public follow viewport](<https://github.com/hudsonkaneko/hivemind-traffic/blob/7940a9757356fa2dcd6790cc8c1769e0d0ba1fbc/documentation/images/showcase-validation-20261019/showcase-public-follow-20261019.png>); [first endpoint transition](<https://github.com/hudsonkaneko/hivemind-traffic/blob/7940a9757356fa2dcd6790cc8c1769e0d0ba1fbc/documentation/images/showcase-validation-20261019/showcase-live-transition-20261019.jpg>); [first endpoint stalled brake](<https://github.com/hudsonkaneko/hivemind-traffic/blob/7940a9757356fa2dcd6790cc8c1769e0d0ba1fbc/documentation/images/showcase-validation-20261019/showcase-endpoint-stalled-brake-20261019.jpg>); [admission endpoint dense state](<https://github.com/hudsonkaneko/hivemind-traffic/blob/7940a9757356fa2dcd6790cc8c1769e0d0ba1fbc/documentation/images/showcase-validation-20261019/showcase-endpoint-dense-in-progress-20261019.jpg>); [gap endpoint recovery](<https://github.com/hudsonkaneko/hivemind-traffic/blob/7940a9757356fa2dcd6790cc8c1769e0d0ba1fbc/documentation/images/showcase-validation-20261019/showcase-gap-recovered-20261019.jpg>). Captions distinguish in-progress images from completed results. The recovery image is the observed 156-second state: 80/80 cars, 34.9 mph, six passes and three lane changes, with current displayed clearance 2.42 m rather than the run minimum. [Image provenance](<https://github.com/hudsonkaneko/hivemind-traffic/blob/7940a9757356fa2dcd6790cc8c1769e0d0ba1fbc/documentation/images/showcase-validation-20261019/provenance.json>) retains all six original-byte hashes.

From the repository root with the documented traffic Python and external Isaac runtime, these commands create new evidence. Reproduction requires the recorded source/configuration/runtime; running changed source is a new experiment. Never install the traffic dependencies into Isaac Python.

```text
python scripts/demo_loop_showcase.py --version v09 --vehicle public --drive-seconds 15 --capture
```

```text
python scripts/demo_loop_showcase.py --version v09 --vehicle public --control-validation endpoints --drive-seconds 180
```

```text
python scripts/demo_loop_showcase.py --version v09 --vehicle public --control-smoke --drive-seconds 90
```

```text
python -m experiments.showcase_benchmark --plan
```

```text
python -m experiments.showcase_benchmark --run
```

```text
python -m experiments.showcase_benchmark --compare outputs/showcase_benchmark/20261019T172621Z-63aa7c17/index.json
```

The benchmark plan is read-only; run starts the declared serial native trials. Use the completed index path for comparison. Do not start competing workloads during measurement. Raw trajectories, logs and local review reports remain ignored under outputs; the committed compact JSON and hashes do not replace unavailable raw telemetry for independent numerical recomputation.

<a id="docs-t-k82ts3ro3qz9"></a>

## 41 Isaac Lab lane task

<a id="docs-h-2oavjxr7zcj0"></a>

### One-car Isaac Lab lane task — October 19, 2026

The separate one-car task and bounded native acceptance matrix are complete. All 12 packages passed, including 13 successful stops, the three expected contact/departure/timeout endings and three fresh-process repeat comparisons. Training is UNAPPROVED and has not launched. No policy, optimizer or trainer entrypoint was created for this milestone; executed optimizer steps remain zero.

The complete local CPU regression passed 2,037 tests and 116 subtests, with 187 optional skips, in 114.55 s. Skipped checks are not passes. Six separate OpenUSD composition tests also passed without a SimulationApp, physics or rendering. Exact native outcomes remain distinct from CPU tests and from later hosted CI.

Published implementation: [4166234](<https://github.com/hudsonkaneko/hivemind-traffic/tree/4166234197000459f9fa9b961c46117aa5281ddf>). [Draft pull request 13](<https://github.com/hudsonkaneko/hivemind-traffic/pull/13>) targets the showcase-validation branch. Hosted CI was still pending at synchronization; the completed local CPU and native results above remain distinct from that later hosted result.

The existing compatibility adapter, scripted lane controller, SUMO demonstrations and showcase remain available. This milestone adds a low-speed physical-control task and a reviewable training proposal; it does not qualify a learned policy or establish perception, traffic coordination or highway behavior.

<a id="docs-h-ihmeoxcp6aw3"></a>

### Task, scene and control authority

One native CPU PhysX car runs at 120 Hz with controls at 60 Hz. The existing roads are 100 m long; the task stops at station 60 m at a target cruise speed of 3 m/s. The 35 s cap includes 2 s of forced-brake settling and one continuous measured second stopped. Resets are bounded to ±0.15 m lateral offset and ±0.035 rad heading error, with native chassis/wheel/rest state and velocities reset. Pose writes are confined to resets; physics owns motion during each episode.

| **Component** | **Responsibility** |
| --- | --- |
| CPU task contract | Observations, bounded reset specifications, reward components, failure/success/timeout priority. |
| DirectRLEnv adapter | One CPU vehicle, two physics substeps per action, detached pre-reset transition, automatic reset. |
| Composed scene | Reusable referenced vehicle, ground, road and optional diagnostic barrier assets; named Environment, Vehicles, Lighting, Cameras, Physics and Debug branches. |
| Runtime namespace | One physics scene at /World/Physics/Scene. Empty /World/envs/env\_0 is Lab bookkeeping, not a cloned vehicle. |
| Supervisor, probe and report | Bounded native process, exact source inventory, original traces/captures, independent physical assessment and fresh-process comparison. |

Static file layers are authored before physics and remain unchanged during each run. Runtime wheel, camera and debug opinions stay in anonymous layers. The scene checks cover the composed hierarchy, reference/layer structure and reopening from another working directory/process. The diagnostic barrier is absent from ordinary driving scenes.

<a id="docs-h-ylxb2jtx7b2z"></a>

### Native matrix and retained history

The final matrix contains 12 unique packages and 16 declared episodes. Seeds 101 and 1101/1102 are validation-only. Native reset/action diagnostics additionally check same-seed repetition, signed pose resets, zeroed native wheel/velocity state, malformed-action rejection without advancing physics, finite clipping and brake priority.

| **Matrix cell** | **Native package** | **Episode result** | **Gate** |
| --- | --- | --- | --- |
| straight100-nominal | 20261019T185927Z-55ca0a2b | 1 successful stop | Passed |
| straight100-repeat | 20261019T190017Z-e121f7b3 | 1 successful stop | Passed |
| straight100-perturbed | 20261019T190105Z-212c63ee | 2 successful stops | Passed |
| left\_r60-nominal | 20261019T185944Z-6fe028fb | 1 successful stop | Passed |
| left\_r60-repeat | 20261019T190033Z-29dd0af5 | 1 successful stop | Passed |
| left\_r60-perturbed | 20261019T190230Z-06a430fb | 2 successful stops | Passed |
| right\_r60-nominal | 20261019T190001Z-69e5cfb2 | 1 successful stop | Passed |
| right\_r60-repeat | 20261019T190049Z-71be4337 | 1 successful stop | Passed |
| right\_r60-perturbed | 20261019T190251Z-d4ca90d2 | 2 successful stops | Passed |
| contact-control | 20261019T185655Z-38b5a659 | rigid\_contact | Passed |
| diagnostics | 20261019T190313Z-5713de32 | time\_limit, lane\_departure | Passed |
| visible | 20261019T185729Z-b54f88b6 | 1 successful stop | Passed |

Across the 13 successful stops, progress was 59.876984–59.878113 m. Worst driving lateral RMS was 0.043482 m and maximum absolute error was 0.150409 m; all had zero rigid contacts and post-settling four-wheel support fraction 1.0. The largest endpoint error was 0.123016 m and largest one-second hold drift was 0.006162 m. Each hold was checked over 121 native states. All three repeat pairs had zero measured position, speed and yaw differences across their complete recorded traces.

Qualification required ≥59.5 m net progress, station 60 ±0.5 m, speed ≤0.05 m/s over the one-second hold, drift ≤0.05 m, driving RMS ≤0.20 m and max ≤0.50 m, full conservative footprint containment, no undesired rigid contact, tilt ≤5°, post-settling four-wheel support ≥0.99 and no unsupported interval over 0.10 s. Expected contact/departure failures and timeout check ending behavior; they do not count as successful driving.

The accepted positive control recorded a chassis/barrier contact at tick 543, persistence at 544, eight contact points in total and termination at tick 544. It verified the newly composed scene and contact-reporting path. The contact-processing setting is enabled for the task, read back during use and restored on cleanup.

| **Earlier evidence** | **Observed outcome and interpretation** |
| --- | --- |
| 185046 — construction | FrozenInstanceError before any episode: Lab configuration processing tried to mutate a nested frozen object. The runtime task contract now separates configuration dictionary handling from the frozen contract. |
| 185241 nominal; 185305 diagnostics | Preliminary passes preserved, then superseded by the complete source-frozen matrix after the contact fix. |
| 185326 — contact control | The car hit the barrier physically but recorded zero contacts and timed out because Lab contact processing was disabled. This remains a failed positive control. |
| 190126 — left perturbations | Both physical episodes passed, but concurrent plain-USD kit.exe processes failed process-isolation acceptance. The package remains failed. |
| 190230 — left perturbation repeat | Same declared case reran without competing Kit processes and passed; this is the accepted matrix package. |

Every package is bounded to 300 s in the current protocol, with no more than 600 s permitted, and stops at 90% whole-GPU memory use. Source/scene immutability, own-process cleanup and contact-reporting cleanup are acceptance gates. The two exact compatibility-baseline OmniPlaybackAPI UI errors are retained and allowed at most once before the task-ready marker; duplicates, later errors, other errors and USD reference-count warnings fail qualification.

An evaluator-only repair reads a non-UTF-8 native log byte with UTF-8 replacement decoding, matching the supervisor. Raw bytes still require exact hashes; a regression confirms a benign byte is accepted while an actual error line still fails. This changed neither captured native evidence nor original pass/fail outcomes. Plain-USD portability checks created no SimulationApp, physics or renderer, but their runtime executable was kit.exe; their concurrency correctly triggered the earlier isolation failure.

<a id="docs-h-kn739tuwpl8"></a>

### Observations, actions and the automatic-reset boundary

| **Observation group** | **Privileged state and known-map fields** |
| --- | --- |
| Route-relative pose | Progress, remaining distance, lateral error, sine and cosine of heading error. |
| Motion and pose | Route-forward, route-lateral, planar and vertical speed; height and upright z. |
| Map and targets | Curvature now, 3 m ahead and 6 m ahead; cruise and endpoint-approach target speed. |
| Applied control and time | Steering, throttle, brake, elapsed fraction, settling flag and hold fraction. |

The actor and critic would share these 22 float32 fields. No sensor measurement or invented yaw-rate signal is used. Wheel/suspension dynamics and angular velocity are omitted, so the interface is not a fully observed vehicle-state claim. Three physical actions request steering in \[−0.5,+0.5\] rad and throttle/brake in \[0,1\]. Any positive brake suppresses throttle; steering slew is capped at 0.5 rad/s and command lifetime at 12 physics ticks. Finite values clip; malformed/nonfinite inputs abort before physics advances. Settling forces brake.

The installed Isaac Lab SB3 wrapper’s fast and slow paths both supplied the post-reset observation as terminal\_observation. The project-local correction uses detached pre-reset final observations, flags and masks, validates finite shape/lifecycle data and copies buffers. Ordinary returned observations still describe the new reset. Only timeout-only truncations bootstrap; true success/failure terminations do not. Failure takes precedence over success, then timeout.

The focused correction tests and ten installed-version synthetic checks passed without a SimulationApp, physics, rendering, model or optimizer. The native matrix then verified terminal-versus-reset snapshots and SB3 timeout flags for successful stops, contact/departure failures and timeout. The outer VecNormalize test applies terminal normalization exactly once and preserves frozen evaluation statistics. The inspected installed SB3 version is 2.9.0; upstream guidance is an API reference, not an installed-version upgrade.

<a id="docs-h-g7xno3yfz2xu"></a>

### Observed throughput and resource scope

Nine successful headless driving packages recorded 17,588 control transitions. Per-package step-call rates were 258.05–318.24 transitions/s; whole episode-loop rates were 203.41–247.94 transitions/s, pooled at 226.37 transitions/s. This set contains nominal, fresh-process repeat and two-corner perturbations on all three routes. Deliberate failure/timeout and visible capture cases are excluded from the headless estimate.

| **left\_r60 package** | **Step-call transitions/s** | **Episode-loop transitions/s** |
| --- | --- | --- |
| Nominal | 258.05 | 203.41 |
| Fresh-process repeat | 277.07 | 215.84 |
| Two corner episodes | 317.44 | 244.78 |

65,536 / 203.412981 gives 322.18 s, about 5.37 minutes of rollout-only reference time using the slowest observed whole-loop rate. This is conservative within the measured scripted sample, not a training-time guarantee. Process startup, PPO policy/value inference, optimizer work, checkpointing and evaluation are unmeasured by that estimate; learned failure/reset behavior may differ. The proposed 3,600 s training wall cap remains a stop, not a prediction.

The visible run measured 63.16 whole-loop transitions/s with rendering and capture; its process time additionally includes inspection. Whole-GPU peaks were 3,318–3,328 MiB for the nine headless driving packages and 5,150 MiB for the visible package, including desktop use. These are control-transition rates, not render FPS or established PPO throughput. No cloned-environment extrapolation is supported.

<a id="docs-h-truq7wdpv3yf"></a>

### Original visual evidence

Original viewport captures show the scripted left-curve drive with requests at [12 s](<https://github.com/hudsonkaneko/hivemind-traffic/blob/4166234197000459f9fa9b961c46117aa5281ddf/documentation/images/lab-lane-following-20261019/lane-following-12s.png>) and [20 s](<https://github.com/hudsonkaneko/hivemind-traffic/blob/4166234197000459f9fa9b961c46117aa5281ddf/documentation/images/lab-lane-following-20261019/lane-following-20s.png>) in accepted package 20261019T185729Z-b54f88b6. Their metadata records request ticks, not exact rendered-frame times. They are native captures, not generated illustrations.

The [scene overview](<https://github.com/hudsonkaneko/hivemind-traffic/blob/4166234197000459f9fa9b961c46117aa5281ddf/documentation/images/lab-lane-following-20261019/scene-overview.png>) from that package was requested after automatic reset, at episode 2, tick 0. It shows the next start pose, not the completed endpoint. The [hierarchy inspection](<https://github.com/hudsonkaneko/hivemind-traffic/blob/4166234197000459f9fa9b961c46117aa5281ddf/documentation/images/lab-lane-following-20261019/scene-hierarchy.jpg>) comes from the same accepted visible package; the separate [layer inspection](<https://github.com/hudsonkaneko/hivemind-traffic/blob/4166234197000459f9fa9b961c46117aa5281ddf/documentation/images/lab-lane-following-20261019/scene-layers.jpg>) comes from supplemental package 20261019T190802Z-167fa4e1, outside the 12-package matrix. Images document appearance and composition; numerical success and safety come from native state assessment.

<a id="docs-h-o95jsctzt21a"></a>

### PPO setup proposed for review — UNAPPROVED

The first proposed pilot trains only left\_r60 with reset offsets sampled within the validated bounds. Straight/right final outcomes would be route-transfer checks. The proposed actor/critic fields, physical actions and safety gate remain as described above. Independent Gaussian throttle/brake outputs are redundant under brake priority, so clipping and overrides must be reported. The two-second settling transitions remain explicit in rollout, with zero running reward.

| **Reward term** | **Proposed contribution, dt = 1/60 s** |
| --- | --- |
| Progress | Signed route-station change /3 once; zero for impossible jumps. |
| Lateral | −dt × (lateral\_error/0.5)² |
| Heading | −0.25 × dt × (heading\_error/0.25)² |
| Speed | −0.25 × dt × ((forward\_speed−target\_speed)/3)² |
| Applied steering rate | −0.01 × dt × (steering\_change/(dt×0.5))² |
| Time | −0.05 × dt |
| Terminal | +10 stopped success; −10 declared failure; zero timeout bonus. Settling running terms are zero, but an early safety failure still receives −10. |

Reward weights remain proposed even though the scripted task computes them to test transition semantics. Signed progress prevents oscillating displacement from earning net progress. Physical acceptance remains separate from reward.

| **Setting** | **Proposed value** |
| --- | --- |
| PPO and device | Installed SB3 2.9.0, MlpPolicy, CPU, four PyTorch threads, one native environment. |
| Networks | Separate actor/critic \[64,64\], Tanh, orthogonal initialization, log\_std\_init −1. |
| Updates | Learning rate 0.0003; rollout 2048; minibatch 128; ten epochs. |
| Discount and loss | Gamma 0.999; GAE 0.95; clip 0.2; entropy 0; value coefficient 0.5; gradient norm 0.5; target KL 0.02; no value clip or state-dependent exploration. |
| Normalization | Observation-only VecNormalize outside corrected wrapper; clip 10, epsilon 1e−8; training statistics update, evaluation freezes them. |
| Training budget | Seed 2101; 65,536 control transitions; 32 completed updates; at most 5,120 minibatch optimizer steps; one serial run. |
| Hard stops | 3,600 s training wall cap; 90% whole-GPU memory; nonfinite data/loss or contract failure; no automatic extension, seed replacement or resume. |

Checkpoint after completed updates 8/16/24/32 at 16,384/32,768/49,152/65,536 transitions. Save actor, critic, optimizer, normalizer, resolved configs, counters, RNG/reset provenance and source/runtime/asset hashes atomically. This does not promise exact native mid-episode continuation.

| **Phase** | **Seeds and exposure** | **Selection/interpretation** |
| --- | --- | --- |
| Validation already used | 101; 1101/1102 on three routes | Scripted native acceptance; separate from policy-training seeds. |
| Proposed training | 2101; 65,536 transitions | left\_r60 only; no training launched. |
| Proposed checkpoint selection | 3101–3103 on left\_r60; four checkpoints; max 25,200 transitions | Separate no-update jobs after training ends. Rank successes, failures, progress, lateral RMS; earlier checkpoint breaks ties. No reward/held-out selection. |
| Proposed held-out comparison | 4101–4105 × three routes × two controllers; max 63,000 transitions | 15 episodes/controller, matched resets, selected deterministic PPO versus unchanged scripted follower, frozen normalizer, no exploration/optimizer. Retain all failures. |

If every checkpoint fails to complete, report no learned completion rather than treating the relative best checkpoint as accepted. Publish per-route success denominators, failures/timeouts, progress, lateral error, contacts/footprint, endpoint/hold, support/tilt, clipping/overrides and measured resources. Five seeds per route are descriptive pilot evidence, not broad safety or generalization. Scalar-speed acceleration/jerk reporting must declare its sampling convention.

Before any training, the user must agree on observations, actions, rewards, PPO, budget, checkpoint selection and held-out evaluation after reviewing this native evidence and throughput scope. A separately reviewed trainer entrypoint must enforce those bounds and require an explicit launch. This milestone supplies no training command or training authorization.

<a id="docs-h-1cug8jbvojtw"></a>

### Reproduction and source evidence

Use the isolated traffic Python environment for the supervisor. The read-only check validates the resolved protocol; other commands below intentionally start bounded scripted native runs and install nothing. Do not run competing Kit processes.

```text
python -m experiments.verify_lane_following_lab --check
```

```text
python -m experiments.verify_lane_following_lab --route straight100 --profile nominal
```

```text
python -m experiments.verify_lane_following_lab --route left_r60 --profile visible
```

Source and evidence: [task report](<https://github.com/hudsonkaneko/hivemind-traffic/blob/4166234197000459f9fa9b961c46117aa5281ddf/documentation/lab-lane-following.md>), [compact evidence](<https://github.com/hudsonkaneko/hivemind-traffic/blob/4166234197000459f9fa9b961c46117aa5281ddf/documentation/evidence/lab-lane-following-20261019.json>), [training proposal](<https://github.com/hudsonkaneko/hivemind-traffic/blob/4166234197000459f9fa9b961c46117aa5281ddf/documentation/lab-lane-following-training-proposal.md>), [resolved proposal](<https://github.com/hudsonkaneko/hivemind-traffic/blob/4166234197000459f9fa9b961c46117aa5281ddf/experiments/lane-follow-ppo-proposal.json>) and [validation protocol](<https://github.com/hudsonkaneko/hivemind-traffic/blob/4166234197000459f9fa9b961c46117aa5281ddf/experiments/lab-lane-following-validation.json>). Raw trajectories/logs remain ignored under outputs; published compact hashes do not replace raw data for independent numerical reconstruction.

<a id="docs-h-vrbl26s62aqt"></a>

### CIRCLES controller smoke — October 22, 2026

Seven pinned public ONNX exports were tested without training on our existing one-car PhysX loop task against a stationary native obstacle. All seven supplied throttle/brake through a small adapter; our existing PathFollower supplied steering. This was an isolated physical-car test, not the full background-traffic showcase.

Six models moved 9.14–11.73 m without contact, with peak speeds of 0.59–0.77 m/s. They timed out well short of the task's required stopping window. The cra\_interp export reached 2.99 m/s and contacted the obstacle after 22.51 m. Strict stopping-task success: 0/7. Basic non-contact actuation smoke: 6/7. Low final speed alone is not a successful policy stop.

Inputs were measured native ego speed, stationary leader speed and chassis-frame bumper gap, in m/s, m/s and metres. ONNX inference ran at 20 Hz, pedal/steering commands at 60 Hz and physics at 120 Hz. The adapter used desired\_speed = clamp(speed + 0.6 \* output, 0, 3), throttle gain 0.7 and brake gain 0.35. Two seconds of reset settling used full brakes; no obstacle safety shield supplied the driving controls.

Only controllers/cra/README.md explicitly documents acceleration units and the Time-Headway Follower-Stopper contract. Other exports use an experimental acceleration interpretation consistent with their graphs and common wrapper; they are not established as the exact final field-trial policy. These inference exports are not resumable RSL-RL checkpoints.

Confirmed run: outputs/circles\_control/v01-20261023T021853Z-3dd776cb. All seven cases completed; cleanup, source identity and runtime checks passed. The earlier run is retained because separate log streams caused known startup errors to be misclassified. No training or external Isaac runtime changes occurred. Small ONNX dependencies and raw evidence remain in ignored project folders.

Next implementation need: calibrate acceleration tracking and the observation/output contract before evaluating a moving leader or full traffic. This smoke does not establish LiDAR perception, merging, sudden-object avoidance or highway-speed control.

Report and rerun instructions: [`https://github.com/hudsonkaneko/hivemind-traffic/blob/codex/circles-control-smoke/documentation/circles-control-smoke.md`](<https://github.com/hudsonkaneko/hivemind-traffic/blob/codex/circles-control-smoke/documentation/circles-control-smoke.md>)

<a id="docs-h-2prchjl12cla"></a>

### CIRCLES highway preparation — October 25, 2026

Prepared, with highway driving validation pending. The user chose to defer the native retry while another project uses Isaac. No background launch or training is scheduled.

The separate v01 probe places three physical cars in adjacent V02 highway lanes. CIRCLES supplies the main car's longitudinal command and the existing scripted PathFollower supplies steering. Two frozen RSL-RL PPO actors control the other cars with their trained 3 m/s target and deliberately empty object slots for nonreactive lane-only traffic.

The ceiling is 60 mph (26.8224 m/s), not a target-speed promise. The adapter clamps speed + 0.6 \* output to that limit, with throttle gain 0.7 and brake gain 0.35. Full braking is requested at the ceiling and measured overspeed terminates the case. With no same-lane leader, CIRCLES receives the experimental surrogate \[own speed, 26.8224, 1000\]; this is not an upstream-verified missing-leader contract or sensor measurement.

One scripted baseline and seven pinned CIRCLES exports are configured for 35 simulated seconds each, including 2 seconds of settling. Physics runs at 120 Hz, commands at 60 Hz and CIRCLES at 20 Hz; seed 51001, 600-second process bound and zero optimizer steps. Passing requires the full duration, more than 5 m progress per car and no contact, footprint departure, support/tilt failure, reverse motion, command fallback or overspeed.

The first native attempt, v01-20261025T211416Z-ee14a001, failed during scene export before driving: 0 physics steps, no completed cases and no screenshot. Cleanup passed, but the evidence gate failed. The repair exports a separate file-backed layer without recomposing the live scene. All 55 selected CPU/OpenUSD tests passed, including portable three-car composition and preservation of all 12 wheel handles. The repaired native run remains untested.

Run later from the configured checkout root after Isaac is free: python experiments/run\_highway\_circles.py. The existing policy remains outputs/rsl\_integration/v02-train-20261022T203726Z-61afe5d7/policy.pt; no new checkpoint is created. New evidence and logs use unique outputs/highway\_circles and logs/highway\_circles directories. Models, checkpoints and raw outputs remain local and are not included in a Git clone.

Protocol, exact checkpoint hash, launch prerequisites and retained failure: [`https://github.com/hudsonkaneko/hivemind-traffic/blob/codex/highway-60mph-ppo-traffic/documentation/highway-circles-60mph.md`](<https://github.com/hudsonkaneko/hivemind-traffic/blob/codex/highway-60mph-ppo-traffic/documentation/highway-circles-60mph.md>)

<a id="docs-h-f79cqbasl293"></a>

### October 25, 2026: headless SUMO MAPPO v01 setup

The SUMO environment, shared local actor, centralized training critic and quiet bounded runner are implemented. No driving-policy training has started, and no trained MAPPO policy exists. The task is four fixed-lane cars following ordinary traffic on a straight three-lane 5 km road. SUMO owns motion; merging, learned steering, jump-in obstacles, sensor perception, communication and physical/Omniverse controller transfer remain later work.

Each actor sees 27 simulator-derived local features: speed, acceleration, lane index, remaining route fraction, speed limit, previous requested acceleration, six front/rear neighbor slots across own/left/right lanes (gap, relative speed and presence), lane-existence flags and elapsed episode fraction. Neighbors are limited to 120 m. The critic alone receives the four local observations plus alive masks (112 values) and a four-value agent identity (116 effective inputs). No global critic state reaches the deployed actor.

The single normalized action u in \[-1,1\] maps to acceleration a = -4.5 + (u + 1) \* 3.25 m/s². Its range is -4.5 to +2.0 m/s²; normalized zero means -1.25 m/s². A local speed/headway filter and SUMO safety checks remain active, with proposed, filtered and realized actions logged. The speed ceiling is 26.8224 m/s (60 mph), control interval 0.1 s, episode horizon 600 steps (60 s), initial speed 12–22 m/s and background arrival interval 1.5–3 s per lane.

Proposed training configuration, awaiting approval: CPU with one Torch thread and four SUMO workers; shared actor \[128,128\] and critic \[256,256\] with Tanh; 256 rollout steps per worker; maximum 1,000,000 actual vehicle decisions and a 7,200-second run deadline including evaluation. Adam actor/critic learning rates are both 0.0003; gamma 0.99; GAE lambda 0.95; policy/value clips 0.2; entropy coefficient 0.01; value coefficient 0.5; gradient norm limit 0.5; four epochs; minibatch 256; target KL 0.02.

Shared reward averages local normalized progress (+1), headway deficit (-0.5), clipped jerk (-0.02), collision (-10) and intervention (-0.1). Runtime seed is 5101; training episode seeds are 100000–199999. Validation uses 300001–300010 on the first, every 25th and final updates; held-out comparison uses 400001–400020 for the selected actor, scripted controller and SUMO-native baseline. Selection first prefers passing policies, then collision/overspeed counts, nominal episodes, reward and fewer overrides. Acceptance requires zero colliding vehicles, zero overspeed decisions, at least 600 m mean progress per car, at most 5% overridden decisions and nominal completion throughout evaluation.

No-learning preflight preflight-20261026T002632Z-df0749c6 completed in 24.969 seconds with zero optimizer updates and unchanged weights. Two 60-second episodes each for native, scripted, random and untrained controllers completed without collisions or overspeed; a scripted repeat matched exactly. A four-worker rollout collected 512 vehicle decisions, passed finite-value and likelihood checks, and exported actor outputs matched their source. Workers closed without forced cleanup. Native/scripted mean progress was 1581.756/1580.709 m. Scripted overrides were 9.25%, failing the unchanged 5% limit; random/untrained progress was 122.264/109.862 m and also failed. Native external overrides are zero by definition. These are setup checks, not learned-policy results.

Subagents implemented and reviewed the setup. Broad regression: 2724 passed, 215 skipped and 116 subtests passed; optional runtime skips remain explicit. Review repaired checkpoint selection, save-before-validation ordering, temporary-directory scope, SUMO error logging and owned process cleanup. POSIX forced cleanup is code-reviewed but untested on this Windows host. Synthetic optimizer unit tests are distinct from actual driving-policy training.

Configuration inspection only: python experiments/run\_sumo\_mappo.py

Bounded no-learning check: python experiments/run\_sumo\_mappo.py --mode preflight

Actual training requires a later user approval and explicit --mode train with the displayed configuration SHA-256. Digest: 0c8e70ef98827a27ed5c77dc680ea35739811cdbbca045e8fb76af9fd2d3265c. The future best actor saves to outputs/sumo\_mappo/v01/train-&lt;UTC&gt;-&lt;id&gt;/policy.pt, with best/latest checkpoints and full evidence alongside it. Preflight's untrained actor is not a trained driving policy. Logs stay in logs/sumo\_mappo; checkpoints and raw evidence are local, not included in a clone.

The runner handles collection, learning, checkpointing, evaluation and shutdown itself, with only start/final status printed. After approval, wait for process completion without continuous chat analysis. No monitoring automation or automatic retraining loop was installed. The Google guide and GitHub Markdown mirror remain chat-managed.

Full configuration, component definitions, commands, evidence and transfer limits: [`https://github.com/hudsonkaneko/hivemind-traffic/blob/codex/sumo-mappo-setup/documentation/sumo-mappo-setup.md`](<https://github.com/hudsonkaneko/hivemind-traffic/blob/codex/sumo-mappo-setup/documentation/sumo-mappo-setup.md>)

<a id="docs-h-ynrpce2pp5qp"></a>

#### October 25 — Forward sensor milestone v01 (native validation pending)

The existing SUMO and physical-car policies still use privileged simulator inputs. A new audit identifies exact neighbor state, obstacle-derived braking targets, episode bookkeeping and world-frame conversion shortcuts; historical checkpoints and observation contracts are preserved.

The separate sensor prototype receives forward RGB, RTX LiDAR returns and noisy wheel-speed/yaw-rate measurements with an initial noisy localization fix. RGB lane evidence is a brightness heuristic, not a learned semantic detector. Depth is saved for evaluation only. Packet timestamps, calibration, episode identity and stale-data braking are checked. The WORLD LiDAR conversion uses simulator pose only inside measurement generation; noncompensated moving-object and high-speed accuracy remain unverified.

The actual reusable local vehicle source is now v08, with a camera and LiDAR mount authored under the moving chassis in vehicle-sensors.usda. These survive reference or payload loading. V07 and a byte-identical backup are preserved. The private vehicle package is not included in GitHub.

160 targeted CPU tests and 17 OpenUSD tests pass. A bounded 15-second, 2 m/s native test is prepared, including a blank-camera braking check and genuine front/overview image capture. Native driving and screenshots remain pending while ARACHNID occupies the GPU; the user requested waiting. No new reinforcement-learning training is launched.

Source, exact parameters and pending evidence: [`https://github.com/hudsonkaneko/hivemind-traffic/blob/codex/sensor-perception-v01/documentation/sensor-perception-v01.md`](<https://github.com/hudsonkaneko/hivemind-traffic/blob/codex/sensor-perception-v01/documentation/sensor-perception-v01.md>)

Input audit: [`https://github.com/hudsonkaneko/hivemind-traffic/blob/codex/sensor-perception-v01/documentation/sensor-input-audit.md`](<https://github.com/hudsonkaneko/hivemind-traffic/blob/codex/sensor-perception-v01/documentation/sensor-input-audit.md>)

Draft review: [`https://github.com/hudsonkaneko/hivemind-traffic/pull/21`](<https://github.com/hudsonkaneko/hivemind-traffic/pull/21>)

<a id="docs-h-vpbx465vyrh7"></a>

#### Forward sensor v01 — native validation result

The final native run 20261026T053004Z-02b4a01a passed the bounded scripted fixture: 15 simulated seconds, 8.5195 m travel, maximum speed 1.9579 m/s, final obstacle clearance 2.5715 m, no contacts and full-body lane containment. All 126 nominal control frames after warmup were valid, excluding the intentional blank-camera fault; that fault commanded full braking and recovered. RTX delivered 299 LiDAR packets. No reinforcement-learning training started.

Native failures were preserved. The source v08 camera now includes the native RTX schema, a 15-degree downward pitch and visual support stems. Startup waits for viewport readiness before constructing sensors. The RGB heuristic handles bright shoulders. Obstacle braking remains latched as speed falls, releasing only when fresh measured clearance increases. Before the final run, the stop-clearance ceiling changed from 2.5 m to 3 m to accept the safer latched stop; previous failures were not relabeled.

Final checks: 163 targeted CPU tests and 18 plain-USD tests pass. Actual, unenhanced front-camera and vehicle overview screenshots were inspected and published in the milestone report. Source files were unchanged during the final run. This is one seeded static-obstacle test, not a held-out evaluation, repeated native reset proof, highway-speed qualification or learned-policy integration. Old actor observation contracts remain unchanged. Surround cameras, Cosmos generation, moving-hazard validation and new policy training are future work.

Results and screenshots: [`https://github.com/hudsonkaneko/hivemind-traffic/blob/codex/sensor-perception-v01/documentation/sensor-perception-v01.md`](<https://github.com/hudsonkaneko/hivemind-traffic/blob/codex/sensor-perception-v01/documentation/sensor-perception-v01.md>)

<a id="docs-h-edbm8sqjk8k5"></a>

#### Seven-camera source rig v01 — October 26, 2026

The reusable local Americano v09 source now contains front wide, front telephoto, cross left/right, rear left/right and rear telephoto cameras under the moving chassis. v08 and the LiDAR path are preserved. All seven use 640×360 RGB at 10 Hz; horizontal fields of view are 120°, 30°, 120°, 120°, 70°, 70° and 30° respectively. Housings are visual only; vehicle mass and collision geometry are unchanged.

Bounded native run 20261026T190556Z-d6c892f5 passed: 59 valid frames per camera, all seven assigned target visibility checks, 0.6299 m travel, zero contacts, stable camera mounts and unchanged source hashes. Six simulated seconds took 20.82 wall seconds including startup; this is not a real-time throughput benchmark. 44 capture tests and 21 USD tests passed. Actual screenshots and three failed attempts are documented. RTX discarded matrix-only mounts; standard translation/orientation ops fixed the source, with pre/post initialization checks added.

This validates camera capture and mounting, not learned surround perception or driving. No new training, Cosmos, fisheye distortion, learned sensor fusion or active LiDAR concurrency test was performed. Existing RL actors still use their prior observation contracts. Review the perception encoder, actor inputs, rewards, training budget and evaluation before training.

Results and screenshots: [`https://github.com/hudsonkaneko/hivemind-traffic/blob/codex/seven-camera-rig/documentation/seven-camera-v01.md`](<https://github.com/hudsonkaneko/hivemind-traffic/blob/codex/seven-camera-rig/documentation/seven-camera-v01.md>)

<a id="docs-h-wb812z1miqpz"></a>

#### Roof LiDAR v01 and combined sensor test — October 26, 2026

Local reusable vehicle source v10 now names the central sensor LidarRoof, with runtime child LidarRoof/Scan. The roof mount was raised 20 cm to chassis XYZ (0, 0, 1.45 m), with source-authored housing/support changes and unchanged vehicle dynamics. Seven cameras remain. v08/v09 retain their historical paths; old actor contracts are unchanged.

Combined native run 20261026T192146Z-40b994ce passed: 119 complete LiDAR scans, 59 valid images per camera, returns on all six tall surrounding targets, 0.6299 m travel, stable camera mounts, zero contacts and unchanged source hashes. 94 CPU tests and 22 USD tests passed. Actual vehicle and point-cloud screenshots are documented.

Near-field limitation: zero returns on 30 cm cubes placed 2, 4 and 6 m to both sides. The retained 32-emitter profile spans -15 to +10 degrees vertically, with 20 Hz scans and configured 0–360 degree azimuth. Roof placement does not guarantee complete close-range coverage. Source hardware lies below the sensor origin, but 1,666 returns in a broad roof region mean downward-beam self-occlusion remains possible. Short-range/perimeter sensing or a separately validated wider vertical profile remains future work. No new training or learned fusion was started.

Results: [`https://github.com/hudsonkaneko/hivemind-traffic/blob/codex/roof-lidar-rig/documentation/roof-lidar-v01.md`](<https://github.com/hudsonkaneko/hivemind-traffic/blob/codex/roof-lidar-rig/documentation/roof-lidar-v01.md>)

<a id="docs-h-jc0n7z4ekux1"></a>

#### Body-mounted cameras v01 — October 26, 2026

The actual reusable local v11 vehicle source removes all seven camera roof stalks. Compact housings now sit on measured exterior body surfaces: front fascia/hood, front fenders, rear quarter panels and tailgate. The central roof LiDAR remains unchanged. Camera paths, optics and headings are preserved; body-camera mount calibration metadata is updated. Earlier sources remain preserved.

Native run 20261026T193608Z-4ea7afd5 passed combined sensing: 59 valid images from each camera, all camera targets visible, 119 complete LiDAR scans with all tall surrounding targets detected, stable mounts over 0.6299 m travel, zero contacts and unchanged source. Two exterior-intersection tests and actual source checks passed. Visual inspection of an earlier layout found excessive body occlusion in cross views; moving those cameras onto the outer fenders cleared the views. Actual screenshots are documented.

Roof-region LiDAR self returns fell to zero in this test; that does not prove universal clearance. Small side targets at 2–6 m remain undetected by the existing LiDAR profile. No training or learned fusion was started; old actor observation contracts are unchanged.

Results: [`https://github.com/hudsonkaneko/hivemind-traffic/blob/codex/body-camera-rig/documentation/body-camera-v01.md`](<https://github.com/hudsonkaneko/hivemind-traffic/blob/codex/body-camera-rig/documentation/body-camera-v01.md>)

<a id="docs-h-8auh37hm86ow"></a>

#### Near-field sensing and perception dataset preparation — October 26, 2026

The actual local v14 source preserves seven body-mounted cameras and adds four compact perimeter LiDARs alongside the roof scanner. The opt-in generic 64-beam profile spans -65 to +10 degrees at 20 Hz. Roof-only and side-only attempts exposed occlusion; all attempts are preserved. Native run 20261026T212758Z-d9729274 passed all 12 small near-field target checks, six tall surrounding targets, 119 complete scans per scanner and 59 images per camera, with stable mounts, unchanged source and zero contacts. This closes measured gaps, not all possible blind spots or highway-speed sensing. 141 focused CPU tests, 28 USD tests and four additional capture-config tests passed.

Six short scenes produce 60 labeled front-camera frames (40 training, 10 validation, 10 test), with RGB, semantic masks and separate raw LiDAR. Native labels are supervision/evaluation only. The initial sample passed file, hash, class and split validation; pedestrian examples are absent and crossing boxes are labeled obstacle. Review prompted camera/chassis transforms and LiDAR frame-end pose metadata in revised captures. Raw WORLD scans are noncompensated; moving-object deskew and deployed fusion remain unverified. The sample verifies plumbing, not a training-quality corpus.

Proposed training, NOT STARTED: front-wide 640x360 RGB, six-class DeepLabV3 MobileNetV3, future 20,000 frames split 16,000/2,000/2,000 by scenes/seeds, AdamW learning rate 0.0001, weight decay 0.0001, batch 8, mixed precision, at most 20 epochs and validation-mIoU early stopping after four epochs without improvement. Weights are not downloaded. Full data, realistic pedestrian examples, an implemented/tested isolated training entrypoint, a measured wall-time cap and user review remain prerequisites. No policy training or changes to old actor inputs were made.

Results, version history and actual screenshots: [`https://github.com/hudsonkaneko/hivemind-traffic/blob/codex/perception-preparation/documentation/perception-preparation-v01.md`](<https://github.com/hudsonkaneko/hivemind-traffic/blob/codex/perception-preparation/documentation/perception-preparation-v01.md>)

<a id="docs-h-53zjwdnf2yx3"></a>

#### October 26: MAPPO v03 communication and traffic-wave setup

Implemented explicit simulated V2V transport between four SUMO-controlled cars: timestamped JSON state and preceding action intent, 10 Hz, 150 m range, six masked receiver slots, 0–300 ms delay, 10% packet loss and 500 ms expiry. Duplicate, stale and out-of-order packets are rejected; background vehicles do not transmit. A shared message encoder extends the actor from 27 to 69 inputs. Global critic information is not supplied to the deployed actor.

A seeded background leader brakes at -2 m/s² for four seconds, starting between 10 and 15 seconds. Wave rewards and throughput measure the affected lane only. Following target stays 1.5 seconds; speed ceiling stays 60 mph and cruise target stays 25 m/s. Merging and closer following are future work.

124 targeted tests passed. Native preflight preflight-20261026T230647Z-60ff04c7 passed with zero optimizer steps, unchanged parameters, reproducible seeded episodes, 512 rollout decisions and matching actor export. Scripted and cooperative scripted controls both achieved zero collisions/overspeed and 3.4375% overrides. Their outcomes were identical: communication benefits are not yet demonstrated. Pooled follower speed variance includes initial transients and changing membership and is not a string-stability certificate.

Final proposed pilot: MAPPO, four CPU workers, one million agent decisions, two-hour total budget, 60-second episodes, ten validation seeds and fifty held-out seeds. A separate matched no-communication configuration is prepared. No training has started; review the final configuration before launching. Learned wave improvement requires the separate trained baseline comparison: at least 15% lower pooled speed variance and no more than 2% throughput loss, alongside the unchanged safety acceptance.

Source and detailed configuration: [`https://github.com/hudsonkaneko/hivemind-traffic/blob/codex/mappo-communication-waves/documentation/sumo-mappo-v3.md`](<https://github.com/hudsonkaneko/hivemind-traffic/blob/codex/mappo-communication-waves/documentation/sumo-mappo-v3.md>)

<a id="docs-h-7ku459q12ami"></a>

#### October 29 — MAPPO v04 passing and cooperative-following setup

Implemented a new SUMO task with four shared-policy cars, three lanes, longitudinal acceleration and discrete right/stay/left lane requests. Actor observations are 81 structured local/message features; the training critic uses 328 joint features plus four agent identity values. This is not camera/LiDAR control or physical steering. The first pilot uses seeded mixed traffic; no staged-curriculum scheduler is claimed.

<a id="docs-h-ydwcafnpcj38"></a>

#### Safety and following distances

Lane changes check front and rear gaps, TTC, cooldown and simultaneous conflicts. The shield rechecks destination-lane acceleration and uses ordinary headway on lane entry. Fresh messages from the actual sensed controlled leader can enable a 0.8-second target; ordinary or stale leaders use 1.5 seconds. A conservative braking-distance rule dominates both targets at highway speed: about 50.5 metres at 60 mph. Thus the setup does not claim an achieved 0.8-second highway gap. Applied commands and subsequent eligibility are logged separately.

<a id="docs-h-oje0ekwdrwuc"></a>

#### Measured setup verification

145 targeted tests passed, including legacy regression, native SUMO and PettingZoo checks. Preflight preflight-20261029T221040Z-a36f0ca0 completed in 39.453 seconds with zero optimizer updates, unchanged sources and weights, identical scripted repeat and actor-export parity. Across two scripted 60-second episodes: three passes, seven applied lane changes, zero collisions/overspeed/required-gap deficits, and one override in 4,800 decisions. Mean progress was 1,501.02 metres; minimum measured headway was 1.728 seconds. No-message, full, delayed and outage channel checks passed. These are setup/reference results, not learned cooperation gains; the untrained actor failed driving acceptance.

<a id="docs-h-mg2wyetpc4ir"></a>

#### Final proposed training configuration

MAPPO with four CPU workers, 128x128 actor and 256x256 critic, 256 rollout steps per worker, minibatch 256, four PPO epochs, learning rates 0.0003, gamma 0.99, GAE 0.95, clip 0.2 and entropy coefficient 0.01. Limit: one million agent decisions or two hours including evaluation. Speed ceiling 60 mph, cruise 26 m/s. Ten validation seeds and fifty separate held-out seeds; a matched no-communication configuration is prepared. Acceptance additionally requires at least one actual pass in every episode and no car exceeding six lane changes per minute. No v04 training has started; review precedes launch. Frozen v02 weights are not compatible with the new hybrid-action contract.

Configuration SHA256: cc1800e164b7eef4d75958d718826d656ab129a9d87d51952283ad8f68720107

Implementation and complete evidence: [`https://github.com/hudsonkaneko/hivemind-traffic/blob/codex/mappo-v04-cooperative-passing/documentation/sumo-mappo-v4.md`](<https://github.com/hudsonkaneko/hivemind-traffic/blob/codex/mappo-v04-cooperative-passing/documentation/sumo-mappo-v4.md>)

<a id="docs-h-h9w6a85k72k1"></a>

#### Earlier v02 replay and release updates

The frozen v02 trajectory was rendered in 3D, then replayed with the local v14 prepared main car and lightweight background vehicles. The 60-second sensor replay saved 840 RGB images from seven cameras and 600 complete scans from five LiDARs. Capture took 96.944 seconds plus startup; measured LiDAR frame-end lag was 66.667 ms. Source layers were unchanged. This is recorded kinematic sensing, not physical control or sensor-driven MAPPO. Local captures and metrics are retained; compact evidence is in documentation/mappo-v02-sensors.md.

The original v02 policy.pt and matching held-out evaluation.json were published byte-for-byte with verified SHA256 checksums: [`https://github.com/hudsonkaneko/hivemind-traffic/releases/tag/mappo-v02-train-20261026T190003Z-9d133a8c`](<https://github.com/hudsonkaneko/hivemind-traffic/releases/tag/mappo-v02-train-20261026T190003Z-9d133a8c>)

<a id="docs-h-gnx18pd18asn"></a>

#### October 29, 2026 — MAPPO v04 completed pilot: not accepted

User-approved run train-20261029T221921Z-3810ae99 completed without runtime errors in 1,284.421 seconds (21 min 24 s), consuming 1,000,000 agent transitions across 245 updates and 15,510 optimizer steps. The agent did not monitor training; review began after completion. No recovery or resume was needed and no extra training budget was used.

The selected exported actor (update 175, 716,800 transitions) completed all 50 held-out episodes with zero collisions and overspeed, but failed acceptance: 7.7275% safety interventions versus the 5% ceiling, zero lane changes and zero passes. Mean per-car progress was 1,099.03 m; minimum headway 1.601 s and mean cooperative-eligible headway 1.886 s. There were 377 required-gap-deficit steps. These results do not demonstrate 0.8 s following or safety without active filters.

Later checkpoints did learn passing on validation seeds: update 200 made 11 lane changes / 11 passes at 30.90% overrides; update 225 made 50 / 22 at 26.63%; latest update 245 made 106 / 12 at 21.40%, with up to 10 lane changes per car per minute against a limit of 6. Each validation used ten episodes. No evaluated checkpoint passed full acceptance. The configured reward-based ranking correctly selected the earlier following policy; this is an optimization/selection limitation, not evidence of a broken lane-action path.

The scripted reference passed all 50 held-out episodes: 75 passes, 167 applied lane changes, zero collisions/overspeed, 0.2992% overrides and mean progress 1,468.69 m. Independent read-only review found finite training metrics and no evident implementation/runtime error requiring automatic recovery. A revised reward/curriculum/selection configuration and further training would require a reviewed budget; the approved transition budget is exhausted.

Retained locally: outputs/sumo\_mappo/v04/train-20261029T221921Z-3810ae99/policy.pt, checkpoints/best.pt, checkpoints/latest.pt, exact config/runtime/source manifests, per-seed evaluations, training curves, episode logs, action journals and checksums. Exported actor SHA256: 96af779e9e93f0d6c7df643d753ac6f63516ac737e2cf5997ac6cb9cb0dd5f91. Source commit: `b0b63f76fe56db1cf13da6ef8f66a80fdf8ea3d0`. Runtime: Torch 2.14.0+cpu, NumPy 2.4.6, Gymnasium 1.3.0, PettingZoo 1.27.0 and SUMO/TraCI 1.27.1. Config SHA256: cc1800e164b7eef4d75958d718826d656ab129a9d87d51952283ad8f68720107.

Published review evidence is in documentation/evidence/sumo-mappo-v4-training/ (summary, exact learned evaluation and per-seed CSV); documentation/sumo-mappo-v4.md explains the comparison. Bulk model weights and journals remain local. Runner passed=true means successful execution; learned\_acceptance=false is the policy result. This v04 pilot does not supersede the accepted v02 actor. No separately trained no-communication baseline, communication-benefit claim or 3D/physical validation was added.

<a id="docs-h-uqby034c2wnh"></a>

#### October 29, 2026 — MAPPO v04r1 retrained: passing achieved, lane-switching gate remains

The user authorized fixing high interventions and missing passing, then retraining without agent monitoring. V04r1 keeps the same 81-feature local/message actor, acceleration/lane actions, safety limits, communication, rewards and acceptance thresholds. It adds training-only imitation from the conservative scripted reference: 20 episodes (48,000 transitions), 60 supervised epochs, followed by 952,000 MAPPO transitions with current-state auxiliary teacher labels. The frozen neural actor does not use a teacher or scripted fallback at inference. Actor learning rate is 0.00003, entropy coefficient 0.001, auxiliary coefficient 1 and lane-change label weight 10. Initial log standard deviation is -2. Total budget remains one million transitions and two hours including evaluation.

Checkpoint selection now ranks acceptance, collisions, overspeed, nominal endings, passing/frequency targets, intervention rate, then reward. Training seed is 7104; episode seeds 2000000–2099999, validation 2110001–2110010, new held-out tests 2210001–2210050. Old v04 configuration/results remain intact. These new test seeds avoid presenting already inspected test cases as untouched evidence. 167 targeted tests and native no-learning preflight passed. An earlier preflight was invalidated by an overlapping source correction to resume accounting, preserved, and repeated successfully before training.

Run train-20261029T230444Z-1a5f80b7 completed in 1,410.844 seconds (23 min 31 s) without runtime errors or agent monitoring. It consumed the full budget, with 5,640 supervised optimizer steps and 14,537 PPO optimizer steps across 233 PPO updates. Sources stayed unchanged. No resume or additional training was needed. Selected update 225 at 969,600 transitions includes genuine PPO training; the supervised-only candidate was not selected.

Held-out neural results across 50 episodes: 82 slower-car passes, at least one pass in every episode; 4.9583% aggregate safety interventions; zero collisions, overspeed or required-gap deficits; 331 lane changes and 1,449.63 m mean per-car progress. Minimum headway was 1.614 s and mean cooperative-eligible headway 2.665 s. This meets aggregate safety/progress and passing requirements, but full acceptance is still false: seed 2210006 has agent\_2 changing lanes eight times in one minute versus the limit of six. The other 49 episodes meet that frequency gate. The 5% intervention limit is aggregate, not a claim that every episode is below 5%.

The selected checkpoint's ten validation episodes had 6.6625% interventions and a maximum of five changes/car/minute; final update 233 had 8.2875% and a maximum of four. Neither passed all validation gates. Selection used validation only. The scripted reference passed the same 50 held-out seeds with 80 passes, 171 lane changes, 0.1117% interventions and 1,488.69 m mean progress. Original v04's 7.7275% and refined 4.9583% used different test seeds, so this is a descriptive comparison, not a paired causal estimate. No communication-benefit, achieved 0.8 s following, physical steering or 3D-transfer claim is established.

Policy, best/latest checkpoints, demonstrations, journals and all metrics are retained under outputs/sumo\_mappo/v04/train-20261029T230444Z-1a5f80b7/. Policy SHA256: 6695282f8128168316feec7e26692d1644e6815deaa458ad625555bbc025ce52. Source commit: `584dd9760d91560f019340b14f85a25acc7db4b8`. Config SHA256: b092e1ceb14b6f42d2f5a3b3ba8324d9a3cac2b4ca19562221158a2142d14018. Review files in documentation/evidence/sumo-mappo-v4-refinement/ include summary/provenance, exact learned evaluation, per-seed CSV and optimization curves. The detailed note is documentation/sumo-mappo-v4-refinement.md. Accepted v02 remains available; v04r1 is a passing-capable candidate, not fully accepted. Unnecessary lane switching is the remaining behavior to refine in a separately declared experiment.

<a id="docs-h-371pkgkus9ri"></a>

#### October 29, 2026 — MAPPO v04r2 accepted for SUMO passing and lane stability

The user authorized fixing unnecessary lane changes and retraining without agent monitoring. V04r2 increases applied lane-change penalty from -0.05 to -0.5 and auxiliary teacher coefficient from 1 to 3, retains initial rare-action imitation weight 10, uses weight 2 during PPO, lowers initial log standard deviation to -2.5 and entropy coefficient to 0.0003. This addresses policy drift seen in validation. The optional stage-specific training weight preserves older configs when absent. Safety rules, 81-feature actor/actions, five-second cooldown, six-changes/car/minute gate and intervention counting are unchanged; no inference hard cap or scripted fallback was added.

178 targeted tests and native no-learning preflight passed. Independent review verified initial-versus-PPO weighting and resume behavior. Budget stayed one million transitions (48,000 demonstrations, 952,000 on-policy) and two hours including evaluation. RNG seed is 8104, training episode seeds 3000000–3099999, validation 3110001–3110010 and fresh held-out tests 3210001–3210050. Original v04 and v04r1 evidence is preserved.

Run train-20261029T235954Z-cc89b964 completed without runtime errors in 1,291.031 seconds (21 min 31 s), with no agent monitoring or analysis during training. Sources stayed unchanged. There were 233 PPO updates, 13,878 PPO optimizer steps and 5,640 supervised optimizer steps. No update was skipped, no resume was needed and no extra training was started.

The selected frozen actor passes all declared SUMO acceptance checks: 74 actual slower-car passes, at least one in every one of 50 test episodes; 3.175% aggregate safety interventions (limit 5%); maximum two lane changes per car per minute (limit six); zero collisions, overspeed or required-gap deficits; all 50 episodes ended normally; mean per-car progress 1,446.91 m (minimum 600 m). Minimum measured headway was 1.638 s and mean cooperative-eligible headway 3.086 s. The intervention gate is aggregate, not a claim that each episode is under 5%.

Use policy.pt or checkpoints/best.pt: selected update 175 at 764,800 total transitions. Its ten validation episodes also passed, with 1.9708% interventions, 14 passes and maximum two changes/car/minute. Final update 233 regressed to 9.2708% validation interventions; checkpoints/latest.pt is retained for history/recovery and is not the accepted deployment actor. Independent audit verified the exact validation-only selection maximum, all declared test seeds without exclusions and exact export/best-checkpoint parity on 960 saved observations. Test results were not used to select the checkpoint.

The scripted reference also passed the same 50 tests with 84 passes, 176 lane changes, 0.2367% interventions and mean progress 1,487.60 m; the neural actor had 212 lane changes. The baseline still performs better on interventions and average progress. Test seeds differ across refinements, so improvements across versions are descriptive rather than paired causal estimates. This acceptance applies to the declared SUMO passing/following fixtures with active safety filters; it does not establish physical steering, sensor inputs, 3D transfer, achieved 0.8 s following or causal communication benefit. Existing v02 integrations are preserved because their contract differs.

Accepted policy SHA256: 4cf4898df997a444624552515ea987e7e994b1ca40bb002cf91f0647be8a7245. Source commit: `bd59d2f008c4c59615276c759377562fc339e826`. Config SHA256: 8d926dc86f53473ef64fcb5f7f36bfec5801554fe26e1773ecfb803c882485e9. Full checkpoints, demonstrations, action journals, source/runtime identity and metrics remain locally in outputs/sumo\_mappo/v04/train-20261029T235954Z-cc89b964/. Versioned documentation/evidence/sumo-mappo-v4-stability/ contains exact learned evaluation, all-controller per-seed CSV, optimization curves and summary/provenance. Detailed interpretation is in documentation/sumo-mappo-v4-stability.md. Bulk weights and journals are not in Git.

<a id="docs-h-1xx7ncv8h9t1"></a>

#### Semantic mask and LiDAR fusion v01 — October 30, 2026

Implemented calibrated front-camera semantic-mask projection onto returns from the prepared v14 vehicle’s five RTX LiDARs. Static captures run first, followed by scripted moving traffic. No training ran; the MAPPO actor and its observation contract are unchanged. Masks currently come from simulator ground truth, not a learned segmentation model. Object IDs and rendered depth are reserved for independent evaluation and never determine fused labels.

Verified run 20261030T223941Z-65f18896 passed all three fixtures. Static: 99.9545% assigned-point accuracy, 93.1699% in-image return coverage. Lead at 2 m/s: 99.9544% accuracy, 92.9647% coverage. Lead at 10 m/s: 99.9525% accuracy, 92.5769% coverage. Both moving actors have correctly labeled returns in all eight frames. The adjacent fixture moves oppositely at 0.8 or 4 m/s. These are scripted sensing fixtures, not policy-controlled traffic.

Captures align to the absolute sensor clock after warm-up and use a render-only synchronization barrier. All 24 captures retain native point timestamps; accepted returns are 0.128–50 ms old. The unchanged 50 ms freshness gate rejects every return in the +250 ms stale-image control. A 3 m calibration error lowers first-frame accuracy to 75–79%. Forty-four focused offline tests pass. Independent review verified 238 artifact hashes, 13 source hashes and 10 unchanged source-vehicle hashes.

Scope and limits: eight frames over 0.7 seconds per fixture, parked physical ego, front pinhole camera, no moving-object deskew, no learned perception and no highway-speed ego validation. Coverage is separate from accuracy: the fast lead car has 57% assignment coverage. Thin lane markings are mostly rejected and pedestrians are untested. The independent depth diagnostic flags about 1.7% of assigned points behind the rendered visible surface; this is not complete visibility or object-recall validation. Camera timestamps are render-barrier caller stamps, not independently measured acquisition timestamps.

Implementation checkpoint 749f4e2 plus exact source snapshots/hashes records the tested version; all five failed development attempts remain preserved. Runtime: Isaac 6.0.1-rc.7, experimental RTX 1.4.6, RTX 5070, driver 621.01. Full raw captures and metrics remain locally in outputs/semantic-lidar-fusion/v01/20261030T223941Z-65f18896/. GitHub contains compact metrics and actual sensor screenshots, not the bulk scans or source vehicle assets.

Results, screenshots, contracts and reproduction command: [`https://github.com/hudsonkaneko/hivemind-traffic/blob/codex/semantic-lidar-fusion/documentation/semantic-lidar-fusion-v01.md`](<https://github.com/hudsonkaneko/hivemind-traffic/blob/codex/semantic-lidar-fusion/documentation/semantic-lidar-fusion-v01.md>)

Review: [`https://github.com/hudsonkaneko/hivemind-traffic/pull/31`](<https://github.com/hudsonkaneko/hivemind-traffic/pull/31>)
