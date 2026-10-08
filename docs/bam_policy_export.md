# BAM policy export contract (schema version 2)

Author: Insu Park. Validated on 2026-10-08.

The Cyclo Duck exporter records the policy interface and BAM configuration
without interpreting MuJoCo motor gains as physical PD gains. Existing K1
PD/Mimic YAML and ONNX metadata use the previous path unchanged.

## Files

- Training: `params/sim2real.yaml`, in addition to `env.yaml` and `agent.yaml`.
- Policy export: `exported/policy.onnx`, `exported/policy.pt` (TorchScript),
  and `exported/sim2real.yaml`.
- ONNX metadata: `schema_version=2`, `actuator_model_type=bam`, `run_path`,
  `policy_contract` (JSON string) and `policy_contract_sha256`.

Use `play.py ... --export-only` to export without opening a viewer. The actor's
checkpoint normalization is included in both exported networks. Inputs are
unnormalized actor observations after the documented assembly/scale/clip
operations; do not apply the checkpoint normalizer a second time. The tested
ONNX interface is float32 `obs` with shape [1, 61], and `actions` [1, 14].
Network outputs are action offsets, not joint angles or motor torques.

## What the BAM actuator computes

At the pinned BAM revision, the XL330 voltage-controlled model performs:

1. Delayed target minus joint position, multiplied by firmware `kp_fw` and
   `error_gain`, produces the requested PWM duty cycle.
2. The modeled current limiter constrains duty using voltage and back-EMF;
   the physical PWM clamp then bounds it to ±`max_pwm`.
3. Effective supply voltage times duty yields motor voltage.
4. Motor torque follows `kt * V / R - kt**2 * joint_velocity / R`.
5. BAM independently computes dry/load-dependent friction and viscous damping,
   writes `dof_frictionloss`/`dof_damping`, and lets the MuJoCo solver apply them.

Voltage sag uses the sum of absolute previous motor torques in the actuator
group. The current configuration has all fourteen joints in one group.

`kp_fw=200` is a firmware-model gain, not stiffness in Nm/rad. The source model
also supplies `error_gain=0.0028773775022263564`, maximum PWM duty 1.0 and a
modeled current limit of 1.75 A. These are serialized as model settings; they
are not evidence of verified registers on a physical Cyclo Duck. No firmware
registers are read or written by the exporter.

The generic MJLab ONNX helper derives stiffness/damping from MuJoCo motor
arrays, which does not describe BAM. BAM export therefore bypasses that helper
and emits no `joint_stiffness` or `joint_damping` metadata.

## Schema contents

The schema is a new contract; it is not backward-compatible with a K1 PD-only
consumer. A reader must explicitly support version 2 and its `kind`.

| Section | Contents |
| --- | --- |
| `schema_version`, `kind` | `2`, `bam_joint_position_policy` |
| `policy` | Joint order, HOME, default joint velocity, input/output dimensions, ordered observation slices/units/scales/clips and normalization contract |
| `control` | Joint-position interface, 20 ms period, per-joint affine scale/offset, runner clipping and target/bias semantics |
| `joint_limits` | Model hard/soft limits in policy order; hardware verification remains false |
| `commands` | Velocity/head/body dimensions, sampling ranges at export, resampling intervals and configured pose-range curricula |
| `actuators` | Joint groups, BAM/XL330/M6 identity, firmware gain, complete fitted parameter JSON with SHA-256, and voltage-controller settings |
| `actuators[].simulation` | Supply voltage/sag, modeled delay and friction-solver settings |
| `simulation` | Physics timestep/decimation, observation noise/delay/mounting error, encoder-bias and BAM friction-scale ranges |
| `provenance` | Installed BAM version, source URL and exact commit, plus MJLab version |
| `hardware` | Verification status and unresolved per-joint motor ID/direction/zero offset and firmware-register mapping |
| `contract_sha256` | Canonical content checksum |

An actuator excerpt from the generated file is:

```yaml
firmware_gain:
  kp_fw: 200.0
  convention: bam_voltage_controlled_servo_kp
  hardware_verified: false
voltage_controller:
  error_gain: 0.0028773775022263564
  max_pwm_duty: 1.0
  modeled_current_limit_a: 1.75
simulation:
  supply_voltage_range_v: [6.5, 8.2]
  voltage_drop_gain_range_v_per_nm: [0.0, 0.2]
  minimum_voltage_v: 6.0
  command_delay_steps: [3, 6]
  delay_step_unit: physics_step
```

Hardware entries such as `motor_id`, `direction`, `zero_offset_rad` and
`firmware_register_mapping` are null. `hardware.verified` and
`hardware.deployment_ready` are false. CAD joint signs and fitted BAM
`q_offset` are not silently substituted for measured encoder calibration.
A hardware consumer must require completed, verified calibration and an
explicit register mapping before enabling motors. No hardware consumer is
implemented in this change.

Simulation effects are identified separately and
`apply_randomization_on_hardware` is false. A runtime should use calibrated
measurements, preserve observation/action order and reset its previous-action
buffer; it must not inject the random simulated mounting/encoder biases.

The current model uses `target = HOME + action`; a configured runner clip is
applied before the affine transform. In simulation the joint-position action
subtracts the sampled encoder bias before sending the target to BAM. The YAML
records both operations without exporting one world's random bias as a physical
offset. Likewise, voltage samples are not exported as a measured robot battery.

Command sampling/curriculum settings describe the environment configuration at
export; they do not assert that a checkpoint has mastered the final command
range. The exported simulation section covers the actuator and observation
interface; `params/env.yaml` remains the complete training environment record,
including COM/mass/terrain/push randomization.

## Consistency and supported scope

`policy_contract` decoded from ONNX equals the adjacent YAML document.
The hash is SHA-256 of UTF-8 JSON with sorted keys, compact separators and no
NaN/infinity, excluding the top-level `contract_sha256` field. The JSON and
YAML retain numeric precision. Writers reject a contract whose stored checksum
does not match the content. The checksum checks content consistency; it does
not certify calibration or locomotion quality.

The present exporter supports an all-BAM action entity, exactly one
joint-position action, single-frame actor observations and the current Duck
observation terms. It rejects mixed PD/BAM groups, unknown observation terms,
mismatched joint ordering and per-joint processed-target clipping instead of
emitting incomplete deployment semantics. The existing PD exporter is kept
separate.

## Validation

A one-iteration Duck smoke checkpoint was exported through the actual
`run_play(..., export_only=True)` path. This checkpoint is not a validated
walking controller. Checks confirmed:

- A fresh one-iteration training run completed with automatic schema-v2 YAML
  generation through the actual `run_train` path.
- Successful ONNX checker validation and TorchScript loading.
- 40 identical float32 inputs (zeros, random values and real environment
  observations) through the checkpoint actor, ONNX's CPU reference evaluator,
  and TorchScript, including the checkpoint normalizer.
- Maximum absolute output differences: ONNX **1.90735e-6**, TorchScript **0**.
  ONNX Runtime and target hardware execution were not tested here.
- Exact equality between the training-export YAML, adjacent policy YAML and
  embedded ONNX JSON; valid checksum and rejection after modifying a field.
- Firmware gain 200 and the modeled 1.75 A limit recorded separately from PD;
  no PD stiffness/damping metadata, and all hardware mapping entries unresolved.
- K1 PD YAML byte-for-byte identical to the exporter before this change.

Temporary verification scripts remain outside the repository. Export support
is ready for pilot-training artifacts; motor calibration, a schema-v2 hardware
reader and actual robot behavior still require separate validation.
