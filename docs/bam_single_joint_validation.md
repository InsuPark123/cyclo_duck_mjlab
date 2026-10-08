# MJLab alignment and BAM single-joint validation

Date: 2026-10-08. Author: Insu Park.

## Scope and result

The `cyclo_duck` container now runs the MD reference's simulation stack.
The pinned BAM GPU actuator runs on the existing Cyclo Duck loaded-arm model,
including voltage variation, command delay and reset behavior. A separate
friction calculation check exposed an upstream CPU/GPU M6 formula difference;
full numerical equivalence is **not** established. The upstream implementation
is unchanged so that subsequent MD task migration retains its behavior.

This single-joint stage did not connect BAM to the 14-joint Cyclo Duck robot or add a
walking task. It does not identify motor parameters from physical hardware.
The robot MJCF still has its existing position actuators. Subsequent runtime
BAM integration is documented in `source/assets/robots/cyclo_duck/README.md`;
that integration converts the actuators in memory without changing the XML.

## Pinned sources and environment

- MD reference: `pollen-robotics/microduck_rl` at
  `273afe0b31c4ab365b9ff806a927b63ac92b5ddd` (`pyproject.toml`, `uv.lock`).
- BAM: `Rhoban/bam` at `62bd8ce12154340be97e06f7f41a0ca8f116d967`.
- Cyclo assets: `InsuPark123/cyclo_duck` at
  `d1151ac4a12ce2067b774dae6c00f06eaebc5456`.
- Python 3.12.15; MJLab 1.3.0; MuJoCo 3.10.0; MuJoCo Warp 3.8.1;
  Warp 1.12.0; PyTorch 2.9.1; RSL-RL 5.0.1.
- NVIDIA GeForce RTX 5090; PyTorch CUDA 12.8; Warp toolkit 12.9 / driver 13.0.
- `pip check`: no broken requirements.

Install BAM's base package from the pinned revision. Its `mjlab` extra at that
revision declares a different MuJoCo version range; this project pins the
simulation dependencies explicitly instead. See `THIRD_PARTY_LICENSES.md`.

Only the `cyclo_duck` container was rebuilt and recreated. The original
`cyclo_mjlab` container was unchanged. The previous local image is retained as
`cyclo-duck:pre-mjlab-1.3` for rollback; using it also requires restoring the
matching source/dependency configuration.

## Single-joint procedure

Use `cyclo_duck_simulation.model.build_spec()` from the submodule. This loads
`Bam_Loaded_Arm/loaded_arm_150mm_100g`, with joint `joint_1` and actuator `xl330`.
Create one MJLab entity with `BamActuatorCfg(motor_name="xl330", model="m6",
kp_fw=200, target_names_expr=("joint_1",))`.

Simulation settings: 5 ms timestep, `implicitfast`, gravity `(0, 0, -9.80665)`,
zero initial joint angle/velocity and zero `noslip_iterations` in both backends.
Expand `dof_frictionloss` and `dof_damping` with
`Simulation.expand_model_fields()` before stepping: BAM updates these fields
independently for each world. Compare with `bam.mujoco.MujocoController` using
a copy of the same compiled MuJoCo model and the same BAM parameters.

Run three GPU worlds at 5 V, 7.4 V and a duplicate 5 V. Each CPU world has the
matching voltage. Run 1,200 steps (6 seconds) with this target sequence:

| Time | Joint target |
| --- | --- |
| 0–0.5 s | 0 rad |
| 0.5–2 s | +0.35 rad |
| 2–3.5 s | -0.35 rad |
| 3.5–6 s | `0.2 * sin(2*pi*0.5*(t-3.5))` rad |

| Measurement | 5 V | 7.4 V |
| --- | ---: | ---: |
| CPU/GPU angle RMSE | 7.1213e-6 rad | 7.8174e-6 rad |
| Maximum absolute angle difference | 2.2460e-5 rad | 3.0112e-5 rad |
| Peak motor torque | 0.21760 Nm | 0.35155 Nm |

The largest angle difference was approximately 0.00173 degrees. Duplicate 5 V
worlds produced identical angles. All sampled states and torques were finite.
These results describe this trajectory, not a global error bound. Both models
show load/friction-dependent tracking error relative to the commanded angle;
small CPU/GPU differences do not imply perfect tracking.

For delay isolation, overwrite the joint state with zero each step and apply
a 0.3 rad target at step 10. Torque first exceeded 0.01 Nm at step 10 with no
delay and step 14 with a four-step delay: exactly 20 ms.

For the MD settings, run eight worlds for two seconds with seed 42,
`vin_range=(6.5, 8.2)`, `vin_drop_gain_range=(0, 0.2)`, `vin_min=6`, and delay
lags 3–6. Observed supply voltages were 6.51710–7.75162 V and drop gains were
0.02578–0.19754. Joint positions remained finite and the effective voltage
stayed at or above 6 V. Reset cleared previous motor torque while preserving
sampled supply voltages and drop gains.

## Known upstream M6 discrepancy

Compare `bam/model.py::compute_frictions` with
`bam/mjlab.py::_compute_friction_budget` at the pinned BAM revision.

The CPU quadratic load-friction term is enabled only when motor and external
torque have different signs. Its direction masks use strict comparisons, so
equal magnitudes activate neither quadratic contribution. The GPU formula
omits the sign condition and treats equal magnitudes as backdrive. Therefore
it can add quadratic friction where the CPU formula does not.

A 75-point grid used motor and external torques from
`[-0.5, -0.2, 0, 0.2, 0.5]` Nm and velocities `[-3, 0, 3]` rad/s. Maximum
friction-budget difference was **0.00249312 Nm**. An independent evaluation of
the missing gate and different equality handling reproduced the discrepancy
within 5e-8 Nm absolute tolerance plus 1e-4 relative tolerance.

The friction-equivalence check is recorded as **false**, not passed. No BAM
patch was applied. Before treating CPU and GPU as interchangeable for motor
identification or deployment, choose a common M6 definition and repeat both
the grid and trajectory checks. Keeping the pinned GPU definition preserves
the MD training reference's current behavior.

## Existing task compatibility

MJLab 1.3 replaces `flat_orientation` with `upright`, and foot-height observation
and clearance reward functions now consume a `TerrainHeightSensor`. The K1
configuration now adds `foot_height_scan` at both foot sites, with a 0.04 m
ring, two rays, 1 m range and terrain geometry group 0. Observation and reward
parameters use the new sensor names. Actor and critic observation sizes remain
unchanged.

The following tasks each reset and ran eight zero-action GPU steps with two
worlds; observations and rewards remained finite:

| Task | Actions | Actor observations | Critic observations |
| --- | ---: | ---: | ---: |
| Cyclo-Velocity-Flat-K1-Rev1-v0 | 23 | 80 | 95 |
| Cyclo-Mimic-K1-Rev1-Dance1 | 23 | 124 | 256 |
| Cyclo-Mimic-K1-Rev1-Dance2 | 23 | 124 | 256 |

The repository's `run_train()` entry point also completed one PPO iteration
for Velocity and Dance1 with four worlds, eight rollout steps, two mini-batches
and one learning epoch. Both runs exported `sim2real.yaml` and saved a
checkpoint. Mean value losses were 0.2907 and 0.1523, respectively; surrogate
losses were -0.0293 and 0.0370. This checks the training path, not convergence
or compatibility of previously trained checkpoints. Dance2 shares the Mimic
training implementation and was checked at environment level only.

MJLab emitted asset-attachment notices about selecting the first keyframe and
ignoring the child model's integrator setting, plus a Warp deprecation warning.
The task simulation configuration controls the integrator. These notices did
not prevent environment stepping or the PPO updates.

Both Cyclo Duck `cyclo_duck.xml` and `scene.xml` loaded with `nq=21`, `nv=20`,
`nu=14` and total mass 0.684013385 kg.

Validation harnesses and plots were kept outside this repository. No generator,
viewer or permanent test scripts were added. Results include `summary.json`,
`cpu_gpu.csv`, `friction_grid.csv`, `regression.json` and
`bam_single_joint.png` in the accompanying validation output.
