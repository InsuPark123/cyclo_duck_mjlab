# Cyclo Duck flat walking task

Author: Insu Park. Implemented and checked on 2026-10-08.

Task ID: `Cyclo-Velocity-Flat-Duck-v0`.

This ports the MD **flat walking** recipe from
`pollen-robotics/microduck_rl` revision
`273afe0b31c4ab365b9ff806a927b63ac92b5ddd` into the existing Cyclo task layout.
It uses Cyclo masses/inertias/visuals, the existing MD walking collision proxies,
and the pinned BAM M6 GPU implementation. Rough terrain, recovery, rollers,
backlash and other MD tasks are outside this implementation.

## Running

From `/workspace/cyclo_duck_mjlab` inside the `cyclo_duck` container:

```bash
python scripts/reinforcement_learning/train.py Cyclo-Velocity-Flat-Duck-v0 \
  --env.scene.num-envs 32 --agent.max-iterations 10
```

The task is registered through `source.tasks`, like the existing K1 tasks.
Training uses `logs/rsl_rl/cyclo_duck_velocity/`. A short run checks the learning
pipeline; it does not produce a validated walking controller.

The registration includes MD PPO defaults: actor and critic MLPs with
512/256/128 units, ELU, observation normalization, initial policy std 1.0,
24 rollout steps, 5 learning epochs, 4 minibatches, learning rate 1e-3 with
adaptive scheduling, gamma 0.99, lambda 0.95, clip 0.2 and desired KL 0.01.
Symmetry is disabled in the reference, so the standard MJLab PPO config suffices.
Logging uses local TensorBoard instead of the MD W&B project. The iteration
limit is 50,000 and checkpoint interval is 250; no full training run was launched.

## Interface, commands and sensors

The [policy interface](cyclo_duck_policy_io.md) remains 61 actor inputs,
76 critic inputs, and 14 HOME-relative joint actions. Physics runs at 200 Hz;
policy actions and observations run at 50 Hz.

- `twist`: vx ±0.4 m/s, vy ±0.3 m/s, yaw rate ±1 rad/s, resampled every 3–8 s.
  Standing sampling starts at 2%. A 15% turn-in-place bucket overrides sampled
  translation and standing status, with yaw magnitude 0.4–1 rad/s. The inherited
  20% forward-only bucket is retained; heading sampling is disabled.
- `head_pose`: four HOME-relative head/neck commands, resampled every 2–5 s.
- `body_pose`: xyz ±5 mm and roll/pitch/yaw ±0.05 rad, resampled every 2–5 s.
  Its input slots are retained, while body tracking reward remains disabled.
- Foot contact sensor: left/right sole geoms against terrain, contact indicators,
  forces and air times. Terrain-height sensor: two rays on a 4 cm ring per foot.
- Self-collision sensor: robot subtree against itself. Terrain is a flat plane.

## Rewards

These are initial weights; MJLab multiplies rewards by the control timestep.

| Term | Weight / setting |
| --- | --- |
| Linear velocity tracking | 2.0, Gaussian std sqrt(0.1) |
| Angular velocity tracking | 2.0, std sqrt(0.5) |
| Upright | 2.0, std sqrt(0.05) |
| Leg posture | 1.0, MD standing/walking joint tolerances |
| Body angular velocity | -0.05 |
| Angular momentum | -0.02 |
| Joint-limit violation | -1.0 |
| Action rate | -0.1 initially, ramps to -1.0 |
| Foot air time | 3.0, 0.125–0.300 s window |
| Foot clearance | -2.0, target 0.02 m |
| Swing peak height | -0.25, target 0.02 m |
| Foot slip | -0.1 |
| Self collision | -1.0 |
| Head pose tracking | 2.0, mean per-joint Gaussian, std 0.5 rad |
| Head pose bias | Initially zero, ramps to 3.0; negative L1 of a 1 s error EMA |

Foot rewards are gated at command magnitude 0.01. The MD soft-landing reward
is removed. The zero-weight body-pose reward is omitted while retaining its
commands and observation slots. No K1-specific posture, height or gait rewards
are mixed into the Duck configuration.

## Reset, randomization and curricula

Episodes last at most 20 seconds; tilt beyond 70 degrees terminates an episode.
The inherited terrain-bound termination is retained, and invalid joint/root
state or foot contact forces trigger the non-finite-state termination.

- Reset at HOME with zero joint velocity, planar root offset ±0.5 m, random
  yaw ±3.14 rad, and root height 0.12–0.13 m above the environment origin.
- Foot friction 0.7–1.3 at startup; trunk mass/inertia scale 0.95–1.05 at
  startup using MJLab pseudo-inertia randomization.
- Per-reset trunk and head COM offsets initially ±3 mm; armature scale 0.9–1.1;
  BAM dry/load-friction scale 0.9–1.1. The native DR operations use defaults
  and do not compound repeated additive/scaling resets.
- Planar velocity pushes ±0.3 m/s every 3–6 seconds. As in MD, play mode uses
  a shorter 0.5–1 s interval; play is not a deterministic evaluation preset.
- Encoder bias, IMU mounting error, observation noise/delay and BAM voltage/sag/
  command delay remain as documented in the policy and robot asset files.

Curriculum steps use policy steps (`iteration * 24`): action-rate weight reaches
-1 by iteration 1500; requested standing fraction reaches 25% by iteration 2000;
head-bias weight becomes 1 at 600, 2 at 1000 and 3 at 1500. Trunk COM variation
ramps to ±15 mm at 1500 and head COM to ±10 mm at 1000. Velocity-command bounds
remain fixed. Head ranges expand in five stages, bounded by reachable soft
limits as described below. Curricula modify the live managers. Head error EMA
is cleared for reset worlds; MJLab resets the action history itself.

## Adaptations and retained limitations

1. **Reset height:** Cyclo's asset HOME already has z=0.12 m. The reset offset
   is therefore 0–0.01 m, instead of adding another 0.12–0.13 m. This preserves
   the intended MD absolute spawn height.
2. **Head command bounds:** the reference's late symmetric pitch caps exceed
   the existing walking skeleton's soft limits. Each curriculum range is
   intersected with `soft_joint_limit - HOME`; joint limits themselves remain
   unchanged. Final ranges are neck pitch [-1.1, 0.56720], head pitch
   [-1.1, 1.06462], head yaw ±1.4 and head roll ±0.31 rad.
3. **COM events:** the base template's unconfigured `base_com` is replaced by
   the MD trunk/head events. The MD head COM list includes `bearing_roll`,
   which is actually a hip body; this membership is deliberately retained
   for recipe parity and should be revisited during Cyclo-specific tuning.
4. **Viewer arrows:** use standard MJLab command visualization; the MD custom
   arrow-only renderer is not copied. This does not change command samples.
5. **Deployment export:** BAM uses [schema version 2](bam_policy_export.md),
   with separate firmware gain/model parameters and unverified hardware fields.
   Training saves `params/sim2real.yaml`; policy export writes a matching YAML
   beside the networks and embeds the contract/hash in ONNX. K1 PD export is
   unchanged. Hardware runtime integration and motor calibration remain separate.
6. **BAM and collision scope:** the previously measured upstream CPU/GPU M6
   friction discrepancy is retained. Walking collision proxies are not a full
   body-on-ground recovery model. No hardware execution or long-run locomotion
   validation is implied by the tests below.

## Validation

On an RTX 5090, 16 worlds ran 120 random-action policy steps plus explicit
fall, timeout and late-curriculum checks. Observations and rewards remained
finite; the ordinary rollout produced 17 automatic resets. Observed initial
heights were 0.12112–0.12895 m. Checks covered:

- 61/76/14 input/output dimensions and fresh configuration instances;
- forced standing and turn-in-place command modes;
- every head curriculum range within the actual soft joint limits;
- fallen-world reset and timeout handling;
- live reward, command and COM curriculum updates, including the nonzero
  head-bias reward and selected-world history reset.

The repository's `run_train()` then completed one PPO iteration with eight
worlds and 24 rollout steps using the configured five epochs/four minibatches.
Mean value loss was 0.0232, surrogate loss -0.0590 and entropy loss 19.8551.
`model_0.pt`, `params/env.yaml` and `params/agent.yaml` were saved. This initial
task check preceded BAM export support. The subsequent export checks are
documented separately in `bam_policy_export.md`. K1 task registrations remained available.

Temporary validation scripts and training artifacts were kept outside the
repository. This confirms integration and one learning update, not convergence,
standing quality or successful velocity tracking. The next stage is a longer
pilot training run with quantitative tracking/fall metrics and policy playback.
