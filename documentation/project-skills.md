# Project-local agent skills

Snapshot: October 12, 2026, plus one skill written in the repository on
November 18, 2026. Nine project-domain skills are checked into
`.agents/skills/`. They are versioned **instructions for a coding agent**, not
simulation code, installed dependencies, trained policies or evidence that a
simulation has passed validation.

## Included workflows

| Skill | Use it for |
| --- | --- |
| `sumo-traci-traffic` | Deterministic SUMO scenarios, TraCI control and baseline telemetry |
| `pettingzoo-multiagent-traffic` | Simultaneous-action traffic environments, lifecycles and API tests |
| `train-hivemind-marl` | SUMO-backed PPO/MAPPO experiment design, communication and evaluation |
| `stream-traffic-openusd` | Timestamped traffic-to-USD state, coordinate conversion and replay |
| `build-omniverse-traffic-viewer` | Kit or ovstage/ovrtx viewing, controls, overlays and rendering |
| `validate-av-isaac-lab` | Selected policy-transfer/physics validation and motion-ownership checks |
| `run-reproducible-av-experiments` | Seeds, matched comparisons, manifests, metrics and evidence |
| `prepare-simready-vehicle` | Asset preparation, physical driving readiness and scoped validation |
| `tune-highway-training` | Diagnosing a failing training gate and choosing one reward, weight or training change |

Each folder includes `SKILL.md` and `agents/openai.yaml`. Vehicle preparation
also includes two reusable technical references, and training tuning includes
a round-by-round tuning history. The
[source manifest](project-skills-manifest.json) records all 21 packaged files,
original source hashes, adaptations and exclusions.

## Use from a checkout

Open the cloned repository in Codex and select the repository-local skill.
For example, this is a **Codex prompt**, not a terminal command:

```text
Use $sumo-traci-traffic to review our existing SUMO baseline without changing it.
```

Codex discovers repository skills under `.agents/skills`. Same-name user and
repository skills are not merged and may both appear in selectors; choose the
repository copy for this project. If newly added skills do not appear, restart
Codex. No global installation or overwrite of personal skills is needed.
See [official skill discovery guidance](https://learn.chatgpt.com/docs/build-skills?product=breeze).

These copies are regular files, not symlinks or a live sync to personal settings.
Later personal-skill edits do not update GitHub automatically. Cloning the repo
does not install SUMO, Isaac Sim, Isaac Lab, ovrtx, Python packages, connectors or
vehicle assets. Follow [portable demos](portable-demos.md) for runtime setup.

## Current project boundaries

Read [repository instructions](../AGENTS.md), the
[active roadmap](hybrid-roadmap.md) and [scene conventions](../usd/SCENE_STRUCTURE.md)
before applying a skill. The generic skills originated in the SUMO-first phase:

- SUMO motion-authority statements apply to SUMO-backed experiments. They do not
  transfer native PhysX cars back to SUMO or authorize two pose writers.
- The current loop showcase has a PhysX main car and scripted kinematic block
  traffic. The planned SUMO-background hybrid is a distinct integration milestone.
- The Isaac Lab skill's trained-SUMO-policy prerequisite applies to that policy
  transfer workflow, not to standalone scripted physics/asset readiness checks.
- Keep experiment outputs in the repository's configured `outputs/` locations;
  generic `runs/` examples do not supersede the established storage conventions.
- User participation is still required before new Isaac Lab policy training.
  A skill describing training does not authorize an unattended training run.

## Repository-authored skill: tune-highway-training

Written on November 18, 2026 at the user's request ("Write project skill"),
after a review of published RL skills found none that fit this project's gates,
filters, shared ledger and budget. It turns the lessons of highway flow rounds
3 to 17 into one procedure: name the failing gate and car, read the saved
journals, classify the failure layer, check reachability with a scripted probe,
then change one reward term, weight or training setting, sized to the ledger
and judged without tuning on held-out data. Its reference,
`references/tuning-history.md`, lists each round, its change, result and
lesson, with the plan documents it came from.

It has no external source. Ideas were taken, not text, from NVIDIA Isaac Lab's
`debug-rl-training` skill (classify the failure layer, change one variable per
run, choose checkpoints by task metrics) and the tachyon-beep `yzmir` packs
(reward-term review, reading PPO update statistics). Like the other skills it
authorizes nothing: no training, evaluation, held-out run or budget increase.

## Publication boundary and provenance

The snapshot comes from the eight user-managed domain-skill directories on the
maintainer's machine. Installed originals remain unchanged. Copies normalize
line endings and add a short repository-scope paragraph. Vehicle preparation
replaces its private case-study link with an explicit exclusion notice.

The private vehicle case study, machine-specific source paths, local-only assets,
raw sensor data, checkpoints, credentials and runtime installations are excluded.
Unrelated personal skills and bundled provider/plugin skills (including the
Google Docs, OpenAI Docs and skill-creator tooling used for this publication)
are not vendored. NVIDIA documentation links are references, not bundled NVIDIA
skills or software. Obtain third-party tools through their own distribution.

No LICENSE or NOTICE file was present in the selected source directories; none
was removed and no new license grant is invented here. Publishing these workflow
instructions does not grant redistribution rights to the assets or tools they
discuss. Historical examples in the reusable references are illustrative, not
newly verified results or portable vehicle specifications.

## Validate and update

From the repository root, in the existing test environment:

```text
python -m unittest discover -s tests -p test_project_skills.py -v
git diff --check
```

The tests check the inventory, metadata, local reference closure and accidental
machine-path/credential inclusion. They do not test future model behavior or
execute simulation/training. Initial packaging also passed the installed
skill-creator validator for all eight skills. The traffic/tooling Python lacked
PyYAML, so that check used the existing read-only PyYAML 6.0.3 package supplied
with Isaac, with bytecode writes disabled. Nothing was installed or reconfigured.

Google engineering guide tab **38 Project skills in GitHub** was added and
native-readback verified: exact text, headings, two shaded monospace examples,
source links, and all 38 older tab bodies/titles/order preserved. Rendered Google
pagination was not inspected. This is a tooling/documentation milestone only.

Update the repo-local files deliberately, record meaningful changes in the
changelog, run the checks, and follow the [Git workflow](workflow.md). Preserve
the initial source hashes as provenance; record later import snapshots separately
rather than treating those hashes as checksums of the edited repository copies.
