# Copyright 2026 Pollen Robotics
# Copyright 2026 ROBOTIS CO., LTD.
# SPDX-License-Identifier: Apache-2.0
# Author: Insu Park
# Adapted from MD tasks/mdp.py at 273afe0b31c4ab365b9ff806a927b63ac92b5ddd.

"""MD walking observation semantics for the Cyclo Duck policy interface."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import torch
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.tasks.velocity import mdp
from mjlab.utils.lab_api.math import quat_apply, quat_from_angle_axis

if TYPE_CHECKING:
  from mjlab.envs import ManagerBasedRlEnv


def _imu_misalignment_quat(env: ManagerBasedRlEnv, max_angle_deg: float):
  """Sample one fixed mounting rotation per world, shared by both IMU terms."""
  q = getattr(env, "_cyclo_duck_imu_misalignment", None)
  if q is None:
    axis = torch.randn(env.num_envs, 3, device=env.device)
    axis = axis / (torch.norm(axis, dim=-1, keepdim=True) + 1e-8)
    angle = torch.rand(env.num_envs, device=env.device) * math.radians(max_angle_deg)
    q = quat_from_angle_axis(angle, axis)
    env._cyclo_duck_imu_misalignment = q
  return q


def projected_gravity_imu_misaligned(
  env: ManagerBasedRlEnv, max_angle_deg: float, asset_cfg: SceneEntityCfg
) -> torch.Tensor:
  q = _imu_misalignment_quat(env, max_angle_deg)
  return quat_apply(q, env.scene[asset_cfg.name].data.projected_gravity_b)


def base_ang_vel_imu_misaligned(
  env: ManagerBasedRlEnv, max_angle_deg: float, asset_cfg: SceneEntityCfg
) -> torch.Tensor:
  q = _imu_misalignment_quat(env, max_angle_deg)
  return quat_apply(q, env.scene[asset_cfg.name].data.root_link_ang_vel_b)


def foot_contact_forces_safe(env: ManagerBasedRlEnv, sensor_name: str) -> torch.Tensor:
  return torch.nan_to_num(
    mdp.foot_contact_forces(env, sensor_name), nan=0.0, posinf=0.0, neginf=0.0
  )


def foot_height_safe(env: ManagerBasedRlEnv, sensor_name: str) -> torch.Tensor:
  return torch.nan_to_num(
    mdp.foot_height(env, sensor_name), nan=0.0, posinf=0.0, neginf=0.0
  )


def foot_air_time_safe(env: ManagerBasedRlEnv, sensor_name: str) -> torch.Tensor:
  return torch.nan_to_num(
    mdp.foot_air_time(env, sensor_name), nan=0.0, posinf=0.0, neginf=0.0
  )
