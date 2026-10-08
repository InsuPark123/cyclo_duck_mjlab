# Cyclo Duck walking policy interface

Author: Insu Park. Reference: MD walking configuration at
`273afe0b31c4ab365b9ff806a927b63ac92b5ddd`, with MJLab 1.3.0.

This stage defines observation/action ordering and timing. It does not register
a walking task or add reward, termination, command-sampling or PPO settings.
The existing K1 interface is unchanged.

## Usage and dependencies

```python
from source.tasks.velocity.config.cyclo_duck.policy_io import (
    configure_cyclo_duck_policy_io,
)

configure_cyclo_duck_policy_io(cfg)
```

Apply this to a Cyclo Duck `ManagerBasedRlEnvCfg` during construction. It sets
fresh actor/critic groups, the joint-position action, physics timestep,
decimation and the startup encoder-bias event. It preserves other events.
The containing task must supply:

- Entity `robot` from `get_cyclo_duck_cfg()` and `get_cyclo_duck_bam_events()`.
- Commands `twist` (3), `head_pose` (4), and `body_pose` (6).
- Contact sensor `feet_ground_contact`, with found/force fields and air-time
  tracking, and terrain-height sensor `foot_height_scan`, both left foot first.

The task's command distributions and sensor configuration will be integrated
in the next stage. Missing dependencies produce MJLab configuration errors;
there are no silent zero-command or fabricated-contact fallbacks.

## Actor: 61 inputs

Indices below are zero-based half-open slices, in concatenation order.
No observation history is stacked; delayed samples still occupy one frame.
All terms use unit scale, and observations are not clipped here.

| Slice | Term | Meaning |
| --- | --- | --- |
| 0:3 | base_ang_vel | Root angular velocity in body coordinates, rad/s, with IMU mounting error |
| 3:6 | projected_gravity | Unit gravity direction in body coordinates, with the same mounting error |
| 6:20 | joint_pos | Encoder joint angles minus HOME, rad |
| 20:34 | joint_vel | Joint velocity relative to default (zero), rad/s |
| 34:48 | actions | Last supplied policy action, before HOME/bias conversion and BAM delay |
| 48:51 | command | Body-frame velocity command: vx, vy (m/s), yaw rate (rad/s) |
| 51:55 | head_command | HOME-relative neck_pitch, head_pitch, head_yaw, head_roll, rad |
| 55:61 | body_command | Nominal-body-relative x, y, z (m), roll, pitch, yaw (rad) |

The ten head/body command dimensions are retained from the MD walking input
contract, including body commands whose walking reward is disabled in the
reference. No K1 gait-phase feature, privileged linear velocity or terrain
height scan is added to the actor.

## Critic: 76 inputs

| Slice | Term |
| --- | --- |
| 0:3 | True root linear velocity in body coordinates |
| 3:6 | `robot/imu_ang_vel` angular-velocity sensor |
| 6:9 | True projected gravity |
| 9:23 | True joint angles minus HOME |
| 23:37 | Current joint velocities |
| 37:51 | Last supplied policy action |
| 51:54 | Velocity command |
| 54:56 | Foot heights relative to terrain, left then right |
| 56:58 | Foot air times, left then right |
| 58:60 | Foot contact indicators, left then right |
| 60:66 | Foot force components, left then right, transformed as `sign(f)*log1p(abs(f))` |
| 66:70 | Head command |
| 70:76 | Body command |

Critic terms are independent objects and have no actor noise, observation delay,
encoder bias or mounting-error rotation. As in MD, non-finite height, air-time
and contact-force values are replaced with zero. This handles sensor readings
only; state-failure termination must still be provided by the future task.

## Actions: 14 outputs

The same explicit order is used by joint observations and action targets:

| Index | Joint | HOME (rad) |
| --- | --- | ---: |
| 0 | left_hip_yaw | 0 |
| 1 | left_hip_roll | -0.0873 |
| 2 | left_hip_pitch | -0.4579 |
| 3 | left_knee | -0.0049 |
| 4 | left_ankle | 0.4530 |
| 5 | neck_pitch | 0.3491 |
| 6 | head_pitch | 0.3491 |
| 7 | head_yaw | 0 |
| 8 | head_roll | 0 |
| 9 | right_hip_yaw | 0 |
| 10 | right_hip_roll | 0.0873 |
| 11 | right_hip_pitch | 0.4579 |
| 12 | right_knee | 0.0049 |
| 13 | right_ankle | -0.4530 |

One action unit is one radian. The processed encoder-coordinate target is
`HOME + action`. MJLab's `JointPositionAction` then supplies physical joint
target `HOME + action - encoder_bias` to BAM, matching the biased encoder
observation `q + encoder_bias - HOME`. Thus zero action means HOME in encoder
coordinates; a nonzero sampled encoder bias shifts the simulated target.

No action clipping or extra target-limit clamp is introduced in this module.
Hard MJCF limits and the asset's 0.9 soft-limit factor remain unchanged. The
future runner/task owns any policy-output clipping; action value 1 is not a
promise that every joint can physically move another radian.

## Timing and actor sensor effects

- Physics: 0.005 s (200 Hz); decimation: 4; policy: 0.020 s (50 Hz).
- BAM command delay: 3–6 **physics** steps (15–30 ms), defined by the asset.
- Actor angular velocity and gravity delay: 0–1 **policy** steps (0–20 ms),
  delay update period 64 policy steps, independently managed per term/world.
- Actor joint velocity delay: exactly one policy step (20 ms).
- Uniform observation noise: angular velocity ±0.03 rad/s, gravity components
  ±0.01, joint angle ±0.001 rad, joint velocity ±0.25 rad/s.
- Encoder bias: uniform ±0.015 rad per joint/world at startup, retained on reset.
- IMU mounting error: one random-axis rotation of up to 6 degrees per world,
  shared by actor angular velocity and gravity and retained across resets.

Factories return fresh term/configuration objects, avoiding actor/critic and
cross-environment configuration mutation through shared dictionaries. Merely
turning off `enable_corruption` disables additive noise; it does not remove
bias, mounting error or configured delays.

## Validation

A temporary GPU harness supplies fixed nonzero command vectors and the two
required foot sensors, without registering a training task. It checks actual
concatenated dimensions/slots, all fourteen joint targets and encoder-bias
compensation, actor-only bias, measured one-step velocity lag, shared IMU
rotation, reset persistence, finite foot-sensor handling and independent
configuration instances. It then steps the BAM robot with four parallel worlds.
Validation code stays outside the repository, as requested.

On 2026-10-08, all of these checks passed on the RTX 5090 with four worlds and
twelve policy steps (0.24 s). The actor/critic shapes were `(4, 61)` and
`(4, 76)` and all sampled observations were finite. Joint order was also
compared directly with the pinned MD walking XML. Existing K1 task registrations
remained unchanged. This is an interface check, not a locomotion performance test.
