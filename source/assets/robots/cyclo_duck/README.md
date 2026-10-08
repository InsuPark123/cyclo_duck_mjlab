# Cyclo Duck MJCF

Cyclo Duck CAD visuals and inertias on the MD **walking** skeleton. This is a
robot asset and inspection scene, with a fourteen-joint MJLab BAM configuration.
The flat walking task is registered as `Cyclo-Velocity-Flat-Duck-v0`.

## Model contract

- Fourteen hinge joints, with the same names, order, axes, frames and hard
  limits as the MD walking model. The floating root is `trunk_base_freejoint`.
  There are 21 generalized positions, 20 velocities and 14 XML actuators.
- The Cyclo URDF jaw (`head_pitch`) is fixed at zero. Its visual and full inertia
  are retained and merged into the `jaw_soft` body, which moves with `head_roll`.
  The MJCF joint called `head_pitch` corresponds to Cyclo's `neck_joint_2_pitch`.
  The historical MD body name `jaw_soft` does not imply a soft-jaw Cyclo part.
- All 16 Cyclo visual meshes are retained. Total mass is 0.68401338504048 kg.
  Inertias are rotated into MD body frames; the fixed jaw is merged using the
  parallel-axis theorem. Meshes never determine mass or inertia at compile time.
- `HOME` exactly matches the rounded MD RL `HOME_FRAME` joint values. `STAND`
  preserves the slightly different, higher-precision MD scene keyframe. `INIT`
  has zero joint angles. All three use root position `(0, 0, 0.12)` and identity
  orientation. Select a keyframe explicitly when loading the model; MuJoCo's
  ordinary `mj_resetData` selects the zero configuration, not a keyframe.
- Seven sites retain the MD positions and orientations: `imu`, `left_foot`,
  `right_foot`, `head_camera`, `tof`, `head_imu`, `mouth_tip`. The `head_camera`
  camera and six sensors (`orientation`, `angular-velocity`, `imu_ang_vel`,
  `imu_lin_vel`, `imu_accel`, `root_angmom`) are retained. These reference sensor
  locations have not been calibrated against Cyclo hardware.
- Five reference collision geoms use four MD meshes: two sole geoms for ground
  contact, plus two leg geoms and power support for self-collision. Collision
  geometry is intentionally a reference proxy, not Cyclo's full visual mesh.
  Foot contact uses the MD runtime settings: `condim=3`, `priority=1`, sliding
  friction 1.0. This is not an all-body fall/standup collision model.
- The on-disk XML keeps the reference position actuators, damping, friction and
  armature for direct inspection. `get_cyclo_duck_cfg()` converts the fourteen
  actuators to BAM M6 motors in memory, replaces their armature with BAM's
  reflected motor inertia, and lets BAM compute dry friction and damping each
  step. It uses the MD soft joint limit factor (0.9), voltage and delay settings.
  A static HOME pose or finite simulation is not a balance test.

The scene uses a 0.005 s physics step. The robot-only XML supplies no floor and
does not override the containing environment's timestep.

## Joint mapping

Angles below obey `q_cyclo_urdf = sign * q_mjcf` at the current model zeros.
Hardware encoder direction, offsets and motor IDs remain uncalibrated.

| MJCF / MD joint | Cyclo URDF joint | Sign | MJCF range (degrees) |
| --- | --- | --- | --- |
| left_hip_yaw | left_hip_yaw | -1 | -25 to 30 |
| left_hip_roll | left_joint_1_roll | +1 | -22 to 22 |
| left_hip_pitch | left_joint_2_pitch | +1 | -90 to 90 |
| left_knee | left_joint_3_pitch | -1 | -90 to 90 |
| left_ankle | left_joint_4_pitch | +1 | -90 to 90 |
| neck_pitch | neck_joint_1_pitch | -1 | -90 to 60 |
| head_pitch | neck_joint_2_pitch | +1 | -90 to 90 |
| head_yaw | head_yaw | +1 | -170 to 170 |
| head_roll | head_roll | -1 | -25 to 25 |
| right_hip_yaw | right_hip_yaw | -1 | -30 to 25 |
| right_hip_roll | right_joint_1_roll | +1 | -22 to 22 |
| right_hip_pitch | right_joint_2_pitch | -1 | -90 to 90 |
| right_knee | right_joint_3_pitch | +1 | -90 to 90 |
| right_ankle | right_joint_4_pitch | -1 | -90 to 90 |

## Files

- `cyclo_duck.py`: MJLab robot configuration and required BAM events.
- `cyclo_duck.xml`: robot-only MJCF, including HOME/STAND/INIT keyframes.
- `scene.xml`: robot with a floor, lighting and a 0.005 s physics step.
- `collision/`: MD walking contact meshes.
- `visual/`: two re-encoded Cyclo visual meshes and their license declaration.

Load either XML directly with MuJoCo. Keep the repository/submodule layout
intact: ordinary visual STL files are referenced directly from
`third_party/cyclo_duck`, without copying them.

## MJLab BAM integration

When constructing a `ManagerBasedRlEnvCfg`, install both the robot and events:

```python
from source.assets.robots import get_cyclo_duck_cfg, get_cyclo_duck_bam_events

cfg.scene.entities = {"robot": get_cyclo_duck_cfg()}
cfg.events.update(get_cyclo_duck_bam_events())
```

The startup event declares `dof_frictionloss` and `dof_damping` for per-world
expansion before simulation initialization. Omitting it makes multi-world BAM
simulation invalid. When using `Scene`/`Simulation` directly, call
`sim.expand_model_fields(("dof_frictionloss", "dof_damping"))` before stepping.
The reset event independently samples a friction multiplier in 0.9–1.1 for
each selected world, as in the MD walking configuration. It scales dry and
load-dependent friction, not viscous damping, and does not compound on reset.

All fourteen joints share one actuator group: XL330 M6, firmware gain 200,
supply voltage 6.5–8.2 V, voltage-drop gain 0–0.2 V/Nm, minimum effective
voltage 6 V, and command delay 3–6 physics steps. At the MD 5 ms timestep the
delay is 15–30 ms. Supply voltage/drop gain are sampled at initialization and
persist across resets. Voltage sag depends on the sum of all fourteen motor
torques in each world. The existing CPU/GPU M6 friction difference is preserved;
see `docs/bam_single_joint_validation.md` at the repository root.

For subsequent walking-task integration, MD uses joint-position actions with
scale 1.0 and the HOME offset, a 5 ms physics timestep, and decimation 4.
These policy timing/action settings belong to the environment configuration;
the asset configuration alone does not define rewards, observations or balance.
`CYCLO_DUCK_JOINT_NAMES` documents the fourteen-joint model order. Get a fresh
configuration with `get_cyclo_duck_cfg()` before changing parameters.

### Integration check (2026-10-08)

An eight-world GPU run used a flat floor, 5 ms physics, decimation 4, HOME
targets and a 0.05 rad / 0.5 Hz head-yaw sine for two seconds. Checks confirmed:

- Exactly fourteen motors with no residual XML position-controller bias.
- Preserved body masses/inertias, hard joint limits and HOME joint angles;
  total mass 0.684013385 kg and action dimension 14.
- Per-world friction/damping fields, correct friction-scale broadcasting,
  non-compounding partial-world friction updates, and reset torque clearing
  while preserving sampled supply voltages/drop gains.
- Finite states and torques; observed peak motor torque 0.27999 Nm and peak
  joint speed 4.33926 rad/s.

This was an open-loop integration check, not a balance test. Two worlds ended
with the root below floor level (-0.0663 m and -0.0822 m), consistent with
falling under the walking model's limited body collision geometry. Ground
contact for the whole body and a trained balance policy are not established.
The upstream CPU/GPU M6 formula discrepancy remains unchanged. Temporary
validation code was kept outside the repository.

## Provenance

Cyclo source revision: `d1151ac4a12ce2067b774dae6c00f06eaebc5456`,
`cyclo_duck_description/Cyclo_Duck/urdf/urdf_alpha.urdf`.

MD reference: Pollen Robotics, revision
`273afe0b31c4ab365b9ff806a927b63ac92b5ddd`; walking `robot_walk.xml`,
`scene_walk.xml`, robot constants `HOME_FRAME`, and four collision STL assets.
The reference model was generated with onshape-to-robot. Its original CAD source
is [this Onshape assembly](https://cad.onshape.com/documents/804927696f06d877f3f1803e/w/5b75db19292e71970de02dee/e/ef6e972847fec8d82570b35e).

Cyclo's two STL files exceeding MuJoCo's 200,000-face STL limit were re-encoded
into binary MSH under `visual/`. Every triangle and vertex position is retained;
this is not mesh decimation. Normal loading needs no asset conversion step.

The MD adaptations/collision assets retain the Apache-2.0 license and Pollen
Robotics attribution in `LICENSE.md-reference`. Cyclo CAD-derived geometry and
inertial data retain their original Proprietary declaration, including the two
re-encoded meshes (`visual/LICENSE`). No CAD asset is relicensed by this module.
