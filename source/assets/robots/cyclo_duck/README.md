# Cyclo Duck MJCF

Cyclo Duck CAD visuals and inertias on the MD **walking** skeleton. This is a
robot asset and inspection scene; a Cyclo Duck RL task and BAM actuator are not
registered yet.

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
- The reference XML position actuators, damping, friction and armature are
  preserved for inspection/smoke tests. They are **not** the BAM M6 actuator
  used by MD training. The training-time soft joint limit factor (0.9), BAM
  voltage/delay settings and policy control loop belong in the later MJLab
  integration. A static HOME pose or finite simulation is not a balance test.

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

- `cyclo_duck.xml`: robot-only MJCF, including HOME/STAND/INIT keyframes.
- `scene.xml`: robot with a floor, lighting and a 0.005 s physics step.
- `collision/`: MD walking contact meshes.
- `visual/`: two re-encoded Cyclo visual meshes and their license declaration.

Load either XML directly with MuJoCo. Keep the repository/submodule layout
intact: ordinary visual STL files are referenced directly from
`third_party/cyclo_duck`, without copying them.

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
