# Proposed segmentation refinement training v02

October 31, 2026. **PROPOSED_NOT_STARTED. No training or new capture is authorized
or launched.** This is a reviewable experiment specification, not an executable
training configuration: no trainer entrypoint, dataset manifest or verified CUDA
perception runtime exists for it yet. The current preview runtime is CPU-only.

The [proposed JSON](../configs/perception/segmentation_refinement_training_v02.proposed.json)
is separate from the earlier v01 DeepLab proposal and leaves it unchanged. It
proposes supervised image segmentation, not a MAPPO update. Existing driving
policies remain unchanged.

| Setting | Proposed value |
| --- | --- |
| Model | Pinned pretrained SegFormer B0 encoder/decoder; new six-class classifier |
| Labels | Background, road, lane markings, vehicle, pedestrian, obstacle |
| Input | RGB 640x360, padded to 640x384 without stretching |
| Data | 3,600 frames from 180 scene groups; 2,880 / 360 / 360 train / validation / test |
| Optimizer | AdamW; encoder LR 0.00003, decoder/head LR 0.0003; weight decay 0.01 |
| Loss | Train-frequency-weighted cross entropy + 0.5 soft Dice + 0.2 boundary Dice |
| Batch | Four images per microbatch, accumulate four times; effective batch 16 |
| Initial seed | 20261102; additional seeds are proposed follow-ups, not part of this budget |
| Budget | At most 20 epochs / 3,600 optimizer updates / two hours including validation, whichever comes first |
| Selection | Validation-only combination of six-class IoU and road/vehicle boundary F1 |
| Stop early | Five validation epochs without at least 0.001 score improvement |
| Outputs | Immutable run, best/last resumable checkpoints, metrics and comparison screenshots |

The two-hour cap is a budget, **not a measured completion-time estimate**. GPU
memory and throughput must be measured in an authorized preflight before the
resolved launch configuration is agreed. GPU work must run serially. A budget
limit saves the checkpoint; it does not authorize automatic extensions.

## Data and label prerequisites

The existing 24 correlated preview frames have already been inspected and are
reserved for qualitative regression. They are not a training corpus or an
independent test set. The earlier 60-frame pipeline smoke sample is also excluded
from this proposed study. No new data has been generated for this proposal.

The 180 planned groups contain 20 frames each and are split as 144/18/18 whole
groups. Road-layout families, source sequences, scenario seeds and asset pools
are assigned before capture; reused variants of an asset must not cross pools.
The final manifest must enumerate actual group IDs and seeds, preserve file
hashes, reject exact duplicates, and review near-duplicates across splits.
Unseen obstacle identities and road-layout families are reserved for test.

Every split must contain reviewed examples of every class, including real
pedestrian assets. Boxes are obstacles, never pedestrian substitutes. Required
variation includes empty-road negatives, different road edges, vehicle shapes,
lighting, occlusion, distances, cones, barriers, boxes and debris. The existing
road/drivable label ID 1 is retained, but a semantic road label does not prove
traversability. Rider/bicycle labels must be resolved before dataset authoring.

The pretrained model's original 19-class weights are loaded strictly, then only
its final classifier is replaced. That new six-class classifier is random and
**must be trained before its predictions are treated as meaningful**. This does
not alter the frozen 19-class model used for the current preview.

## Loss and evaluation contract

The JSON fixes class-weight calculation from training pixels only. Padding is
applied as exact floating-point zero after RGB normalization, avoiding rounded
uint8 mean-color padding. Target padding is ignored; logits are resized and
unpadded before loss/metrics. Soft Dice handles
foreground imbalance. Boundary Dice compares differentiable 3x3 morphological
gradients of predicted probabilities with label boundaries, excluding ignored
pixels and frame edges. These losses still need implementation and unit tests;
the proposal does not establish that they outperform cross entropy alone.

Checkpoint selection uses validation data only: half mean six-class IoU and half
mean road/vehicle boundary F1 at two-pixel tolerance. All class counts are pooled
before computing metrics. The reserved test set is used once after selection;
test results cannot choose checkpoints or tune thresholds. Report per-scene
metrics and scene-bootstrap intervals. A single training seed cannot establish
training-seed robustness.

This requires a future six-class evaluator accepting prediction IDs 0 through 5.
The current frozen-preview `metrics()` accepts only shared predicted classes
0, 1, 3 and 4 and excludes ground-truth lane/obstacle labels. It must not be
reused as six-class training acceptance evidence.

Provisional research targets are road precision >=95%, vehicle precision >=90%
with recall >=95%, mean road/vehicle boundary F1 >=85%, obstacle pixel recall
>=85%, and false-drivable rate on non-road GT <=2%. That last denominator is all
GT pixels except road, lane and ignored pixels; its numerator is the subset
incorrectly predicted as road. Obstacles, vehicles, pedestrians and background
are included. It is not a fraction of predicted road pixels. These targets are
**not achieved results or driving acceptance**.
They require review against the eventual dataset before training and must not
be relaxed after seeing held-out results. No real-time latency target is claimed
until an intended control rate and target hardware are agreed.

Obstacle evaluation includes small/distant instances and unseen assets, not just
aggregate pixels. The planned object matching rule requires instance annotations
and reports unmatched predictions; a semantic mask alone cannot identify object
instances. LiDAR remains independent geometry evidence. Compare raw RGB masks
and RGB-plus-LiDAR outputs separately; absent returns never imply free space.
Unknown predictions may be withheld using validation-calibrated confidence, but
uncalibrated softmax confidence is not a safety probability.

## Before launch

The next training decision requires a complete dataset manifest, reviewed class
coverage and split isolation, an implemented and tested trainer/resume path,
an isolated verified CUDA runtime, memory/throughput measurements, and explicit
user authorization for the resolved configuration. Any new simulator capture
also needs separate authorization. No training command is supplied because the
trainer and data prerequisites are not ready.

The proposed output location is
`outputs/perception-training/segmentation-refinement-v02/<unique-run-id>/` with
`best.pt` and `last.pt`. Neither file exists as a result of this proposal.
