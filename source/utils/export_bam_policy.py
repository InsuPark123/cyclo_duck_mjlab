# Copyright 2026 ROBOTIS CO., LTD.
# SPDX-License-Identifier: Apache-2.0
# Author: Insu Park

"""Versioned BAM policy contract shared by YAML and ONNX metadata.

Firmware gains, fitted motor parameters and simulation randomization are not
physical PD gains or verified hardware settings. Uncalibrated mappings stay null.
"""

from __future__ import annotations

import hashlib
import json
import math
from importlib import metadata
from pathlib import Path

import torch
import yaml
from bam.mjlab import BamActuator
from bam.model import load_model

from .export_sim2real_cfg import _position_action


def uses_bam(env) -> bool:
  return any(isinstance(a, BamActuator)
             for entity in env.scene.entities.values() for a in entity.actuators)


def _vector(value, length):
  if isinstance(value, torch.Tensor):
    value = value.detach().cpu()
    if value.ndim > 1:
      value = value[0]
    result = value.reshape(-1).tolist()
  elif isinstance(value, (tuple, list)):
    result = list(value)
  else:
    result = [float(value)]
  if len(result) == 1:
    result *= length
  if len(result) != length:
    raise ValueError(f"Expected {length} values, got {len(result)}")
  return result


def canonical_json(value) -> str:
  return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)


def contract_digest(contract: dict) -> str:
  payload = {k: v for k, v in contract.items() if k != "contract_sha256"}
  return hashlib.sha256(canonical_json(payload).encode()).hexdigest()


def _bam_provenance():
  distribution = metadata.distribution("better-actuator-models")
  direct = json.loads(distribution.read_text("direct_url.json") or "{}")
  return {"package": "better-actuator-models", "version": distribution.version,
          "source_url": direct.get("url"),
          "revision": direct.get("vcs_info", {}).get("commit_id")}


_SEMANTICS = {
  "base_ang_vel": ("body_angular_velocity", "rad/s"),
  "projected_gravity": ("unit_gravity_vector_in_body_frame", "unitless"),
  "joint_pos": ("calibrated_joint_angle_minus_home", "rad"),
  "joint_vel": ("calibrated_joint_velocity_minus_default", "rad/s"),
  "actions": ("previous_policy_action_after_runner_clip_before_affine_transform", "action_units"),
  "command": ("body_velocity_command_vx_vy_wz", "m/s,m/s,rad/s"),
  "head_command": ("home_relative_neck_pitch_head_pitch_head_yaw_head_roll", "rad"),
  "body_command": ("nominal_body_relative_x_y_z_roll_pitch_yaw", "m,m,m,rad,rad,rad"),
}


def build_bam_policy_contract(env, *, observation_group="actor", clip_actions=None):
  action = _position_action(env)
  robot = env.scene[action.cfg.entity_name]
  if not robot.actuators or not all(isinstance(a, BamActuator) for a in robot.actuators):
    raise ValueError("BAM schema requires an all-BAM action entity; mixed PD/BAM is unsupported.")
  if action.cfg.clip is not None:
    raise ValueError("BAM schema does not yet support per-joint processed-target clipping.")
  names = list(action.target_names)
  ids = action.target_ids.cpu().tolist()
  home = robot.data.default_joint_pos[0, ids].cpu().tolist()
  manager = env.observation_manager
  if not manager.group_obs_concatenate[observation_group]:
    raise ValueError("BAM export requires concatenated policy observations.")
  observations, effects, start = [], {}, 0
  for name, shape in zip(manager.active_terms[observation_group],
                         manager.group_obs_term_dim[observation_group], strict=True):
    if name not in _SEMANTICS:
      raise ValueError(f"Unsupported BAM deployment observation: {name}")
    term = manager.get_term_cfg(observation_group, name)
    if term.history_length > 1:
      raise ValueError("BAM schema currently supports one-frame observations only.")
    size = math.prod(shape)
    semantics, unit = _SEMANTICS[name]
    item = {"name": name, "slice": [start, start + size], "dimension": size,
            "semantics": semantics, "unit": unit,
            "scale": _vector(1.0 if term.scale is None else term.scale, size),
            "clip": term.clip, "history_length": 1,
            "simulation_function": f"{term.func.__module__}.{term.func.__name__}"}
    if "command_name" in term.params:
      item["command_name"] = term.params["command_name"]
    if name in ("joint_pos", "joint_vel"):
      asset = term.params.get("asset_cfg")
      joint_ids = slice(None) if asset is None else asset.joint_ids
      resolved_ids = (list(range(robot.num_joints))[joint_ids]
                      if isinstance(joint_ids, slice) else joint_ids)
      observation_names = [robot.joint_names[i] for i in resolved_ids]
      if observation_names != names:
        raise ValueError("Joint observation order differs from policy action order.")
      item["joint_names"] = observation_names
    observations.append(item)
    effect = {"delay_steps": [term.delay_min_lag, term.delay_max_lag],
              "delay_step_unit": "policy_step", "delay_update_period": term.delay_update_period}
    if term.noise is not None:
      noise = term.noise
      if not hasattr(noise, "n_min") or not hasattr(noise, "n_max"):
        raise ValueError(f"Unsupported observation noise for {name}")
      effect["uniform_noise"] = [noise.n_min, noise.n_max]
    if "max_angle_deg" in term.params:
      effect["mounting_error_max_angle_deg"] = term.params["max_angle_deg"]
      effect["mounting_error_shared_group"] = "imu"
    effect["encoder_bias_enabled"] = bool(term.params.get("biased", False))
    effects[name] = effect
    start += size
  if start != math.prod(manager.group_obs_dim[observation_group]):
    raise ValueError("Observation dimensions do not match the policy group.")

  commands = {}
  for name in env.command_manager.active_terms:
    command = env.command_manager.get_term(name)
    ranges = command.cfg.ranges
    if hasattr(ranges, "lin_vel_x"):
      entry = {"dimensions": ["vx", "vy", "wz"],
               "sampling_ranges_at_export": [ranges.lin_vel_x, ranges.lin_vel_y, ranges.ang_vel_z]}
    else:
      entry = {"dimensions": len(ranges), "sampling_ranges_at_export": ranges}
    entry["dimension"] = int(command.command.shape[-1])
    entry["resampling_time_range_s"] = command.cfg.resampling_time_range
    for curriculum in env.cfg.curriculum.values():
      if curriculum.params.get("command_name") == name and "range_stages" in curriculum.params:
        entry["range_curriculum"] = curriculum.params["range_stages"]
    commands[name] = entry

  groups = []
  for actuator in robot.actuators:
    cfg = actuator.cfg
    parameter_path = Path(cfg._resolved_json_path)
    parameter_bytes = parameter_path.read_bytes()
    model = load_model(str(parameter_path))
    motor = model.actuator
    groups.append({
      "joint_names": list(actuator.target_names),
      "model_type": "bam", "motor_model": cfg.motor_name or json.loads(parameter_bytes).get("actuator"),
      "model_variant": cfg.model or json.loads(parameter_bytes).get("model"),
      "firmware_gain": {"kp_fw": float(actuator._base_kp),
                        "convention": "bam_voltage_controlled_servo_kp",
                        "hardware_verified": False},
      "parameter_file": {"name": parameter_path.name,
                         "sha256": hashlib.sha256(parameter_bytes).hexdigest(),
                         "parameters": json.loads(parameter_bytes)},
      "voltage_controller": {"error_gain": motor.error_gain,
                             "max_pwm_duty": motor.max_pwm,
                             "modeled_current_limit_a": motor.max_current},
      "simulation": {
        "supply_voltage_range_v": cfg.vin_range,
        "fixed_supply_voltage_v": None if cfg.vin_range else (cfg.vin if cfg.vin is not None else motor.vin),
        "voltage_drop_gain_range_v_per_nm": cfg.vin_drop_gain_range,
        "voltage_drop_load": "sum_abs_previous_motor_torques_in_this_group",
        "minimum_voltage_v": cfg.vin_min,
        "command_delay_steps": [cfg.delay_min_lag, cfg.delay_max_lag],
        "delay_step_unit": "physics_step", "delay_hold_probability": cfg.delay_hold_prob,
        "delay_update_period": cfg.delay_update_period,
        "delay_per_environment_phase": cfg.delay_per_env_phase,
        "stiff_frictionloss": cfg.stiff_frictionloss,
      },
    })
  if sorted(n for group in groups for n in group["joint_names"]) != sorted(names):
    raise ValueError("BAM actuator groups do not cover policy joints exactly once.")
  bias_event = env.cfg.events.get("encoder_bias")
  friction_event = env.cfg.events.get("randomize_bam_friction")
  contract = {
    "schema_version": 2, "kind": "bam_joint_position_policy",
    "policy": {"observation_group": observation_group, "input_dimension": start,
               "output_dimension": action.action_dim, "joint_names": names,
               "home_position_rad": home,
               "default_joint_velocity_rad_s": robot.data.default_joint_vel[0, ids].cpu().tolist(),
               "observations": observations,
               "normalization": "checkpoint_actor_normalizer_included_in_exported_network"},
    "control": {"interface": "joint_position", "position_unit": "rad",
                "period_s": float(env.step_dt), "action_scale": _vector(action.scale, len(names)),
                "action_offset_rad": _vector(action.offset, len(names)),
                "runner_action_clip": None if clip_actions is None else [-clip_actions, clip_actions],
                "processed_target_clip": None,
                "target_formula": "offset + scale * clipped_policy_action",
                "simulation_encoder_bias_compensation": "physical_target = target - sampled_encoder_bias",
                "runtime_encoder_bias": "use calibrated measurements; do not sample simulation bias",
                "last_action_reset_value": 0.0},
    "joint_limits": {
      "hard_rad": [env.sim.mj_model.joint(f"{action.cfg.entity_name}/{name}").range.tolist() for name in names],
      "soft_rad": robot.data.soft_joint_pos_limits[0, ids].cpu().tolist(),
      "hardware_verified": False},
    "commands": commands,
    "actuators": groups,
    "simulation": {"physics_dt_s": float(env.physics_dt), "decimation": env.cfg.decimation,
                   "observation_effects": effects,
                   "encoder_bias_range_rad": None if bias_event is None else bias_event.params["bias_range"],
                   "bam_friction_scale_range": None if friction_event is None else friction_event.params["scale_range"],
                   "apply_randomization_on_hardware": False},
    "provenance": {"bam": _bam_provenance(),
                   "mjlab_version": metadata.version("mjlab")},
    "hardware": {"verified": False, "deployment_ready": False,
                 "joint_mapping": {n: {"motor_id": None, "direction": None, "zero_offset_rad": None} for n in names},
                 "firmware_register_mapping": None},
  }
  # Normalize tuples and validate finite JSON before hashing or writing either format.
  contract = json.loads(canonical_json(contract))
  contract["contract_sha256"] = contract_digest(contract)
  return contract


def write_bam_policy_contract(contract: dict, path: str | Path) -> Path:
  if contract_digest(contract) != contract["contract_sha256"]:
    raise ValueError("BAM policy contract checksum does not match its content.")
  path = Path(path)
  path.parent.mkdir(parents=True, exist_ok=True)
  path.write_text(yaml.safe_dump(contract, sort_keys=False, allow_unicode=True), encoding="utf-8")
  return path


def bam_onnx_metadata(contract: dict, run_path: str) -> dict[str, str]:
  if contract_digest(contract) != contract["contract_sha256"]:
    raise ValueError("BAM policy contract checksum does not match its content.")
  return {"schema_version": "2", "actuator_model_type": "bam", "run_path": run_path,
          "policy_contract_sha256": contract["contract_sha256"],
          "policy_contract": canonical_json(contract)}
