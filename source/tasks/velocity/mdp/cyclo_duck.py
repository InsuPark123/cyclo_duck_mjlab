# Copyright 2026 Pollen Robotics
# Copyright 2026 ROBOTIS CO., LTD.
# SPDX-License-Identifier: Apache-2.0
# Author: Insu Park
# Adapted from MD tasks/mdp.py at 273afe0b31c4ab365b9ff806a927b63ac92b5ddd.

"""Commands, head tracking and curricula used by Cyclo Duck flat walking."""

from __future__ import annotations

from dataclasses import dataclass

import torch
from mjlab.managers.command_manager import CommandTerm, CommandTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.tasks.velocity.mdp import UniformVelocityCommand, UniformVelocityCommandCfg


class DuckVelocityCommand(UniformVelocityCommand):
  """MD velocity sampler including deliberate turn-in-place commands."""

  def _resample_command(self, env_ids: torch.Tensor) -> None:
    super()._resample_command(env_ids)
    ids = env_ids[torch.rand(len(env_ids), device=self.device) < self.cfg.rel_turn_in_place_envs]
    if not len(ids):
      return
    self.vel_command_b[ids, :2] = 0.0
    max_rate = max(abs(v) for v in self.cfg.ranges.ang_vel_z)
    sign = torch.where(torch.rand(len(ids), device=self.device) < 0.5, -1.0, 1.0)
    magnitude = torch.empty(len(ids), device=self.device).uniform_(0.4 * max_rate, max_rate)
    self.vel_command_b[ids, 2] = sign * magnitude
    self.is_standing_env[ids] = False
    self.vel_command_w[ids] = self.vel_command_b[ids]


@dataclass(kw_only=True)
class DuckVelocityCommandCfg(UniformVelocityCommandCfg):
  rel_turn_in_place_envs: float = 0.15

  def build(self, env) -> DuckVelocityCommand:
    return DuckVelocityCommand(self, env)


class UniformPoseCommand(CommandTerm):
  """Independent per-dimension pose samples, held between resamples."""

  def __init__(self, cfg, env):
    super().__init__(cfg, env)
    self._command = torch.zeros(self.num_envs, len(cfg.ranges), device=self.device)

  @property
  def command(self):
    return self._command

  def _update_metrics(self):
    pass

  def _update_command(self):
    pass

  def _resample_command(self, env_ids):
    for i, (lo, hi) in enumerate(self.cfg.ranges):
      self._command[env_ids, i] = torch.empty(len(env_ids), device=self.device).uniform_(lo, hi)


@dataclass(kw_only=True)
class UniformPoseCommandCfg(CommandTermCfg):
  ranges: tuple[tuple[float, float], ...]

  def build(self, env) -> UniformPoseCommand:
    return UniformPoseCommand(self, env)


def _head_error(env, command_name, asset_cfg):
  asset = env.scene[asset_cfg.name]
  ids = asset_cfg.joint_ids
  delta = asset.data.joint_pos[:, ids] - asset.data.default_joint_pos[:, ids]
  return delta - env.command_manager.get_command(command_name)


def head_pose_tracking(env, command_name: str, std: float, asset_cfg: SceneEntityCfg):
  """Mean per-joint Gaussian tracking of HOME-relative head commands."""
  return torch.exp(-(_head_error(env, command_name, asset_cfg) / std).square()).mean(-1)


def head_pose_bias_penalty(env, command_name: str, tau_s: float, asset_cfg: SceneEntityCfg):
  """Negative L1 of a one-second EMA, preserving the MD droop penalty."""
  error = _head_error(env, command_name, asset_cfg)
  if not hasattr(env, "_duck_head_bias_ema"):
    env._duck_head_bias_ema = torch.zeros_like(error)
  env._duck_head_bias_ema[env.episode_length_buf <= 1] = 0.0
  alpha = min(1.0, env.step_dt / max(tau_s, 1e-6))
  env._duck_head_bias_ema.lerp_(error, alpha)
  return -env._duck_head_bias_ema.abs().mean(-1)


def reset_head_history(env, env_ids):
  """Clear selected head-error histories; MJLab clears its action histories."""
  if hasattr(env, "_duck_head_bias_ema"):
    env._duck_head_bias_ema[env_ids] = 0.0


def nonfinite_state(env, asset_cfg: SceneEntityCfg, sensor_name: str):
  """Detect invalid robot state or foot contact forces before the next rollout."""
  data = env.scene[asset_cfg.name].data
  fields = (data.joint_pos, data.joint_vel, data.root_link_pos_w,
            data.root_link_quat_w, data.root_link_lin_vel_w, data.root_link_ang_vel_w)
  bad = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
  for value in fields:
    bad |= ~torch.isfinite(value).all(dim=-1)
  force = env.scene[sensor_name].data.force
  if force is not None:
    bad |= ~torch.isfinite(force).flatten(1).all(-1)
  return bad


def reward_weight(env, env_ids, reward_name: str, weight_stages: list[dict]):
  term = env.reward_manager.get_term_cfg(reward_name)
  for stage in weight_stages:
    if env.common_step_counter > stage["step"]:
      term.weight = stage["weight"]
  return torch.tensor(term.weight, device=env.device)


def standing_envs_curriculum(env, env_ids, command_name: str, standing_stages: list[dict]):
  cfg = env.command_manager.get_term(command_name).cfg
  for stage in standing_stages:
    if env.common_step_counter > stage["step"]:
      cfg.rel_standing_envs = stage["rel_standing_envs"]
  return torch.tensor(cfg.rel_standing_envs, device=env.device)


def pose_command_range_curriculum(env, env_ids, command_name: str, range_stages: list[dict]):
  ranges = range_stages[0]["ranges"]
  for stage in range_stages:
    if env.common_step_counter >= stage["step"]:
      ranges = stage["ranges"]
  env.command_manager.get_term(command_name).cfg.ranges = tuple(ranges)
  return torch.tensor(max(max(abs(lo), abs(hi)) for lo, hi in ranges), device=env.device)


def com_range_curriculum(env, env_ids, event_name: str, range_stages: list[dict]):
  width = range_stages[0]["range"]
  for stage in range_stages:
    if env.common_step_counter > stage["step"]:
      width = stage["range"]
  env.event_manager.get_term_cfg(event_name).params["ranges"] = (-width, width)
  return torch.tensor(width, device=env.device)
