# Copyright 2026 Pollen Robotics
# Copyright 2026 ROBOTIS CO., LTD.
# SPDX-License-Identifier: Apache-2.0
# Author: Insu Park
# Adapted from the MD reference at 273afe0b31c4ab365b9ff806a927b63ac92b5ddd:
# actuator/friction_dr_bam.py and tasks/mdp.py. Backlash support is not included.

"""BAM friction scaling and MJLab model-field setup for parallel worlds."""

from dataclasses import dataclass
from typing import TYPE_CHECKING

import torch
from bam.mjlab import BamActuator, BamActuatorCfg
from mjlab.managers.event_manager import requires_model_fields
from mjlab.managers.scene_entity_config import SceneEntityCfg

if TYPE_CHECKING:
  from mjlab.envs import ManagerBasedRlEnv


class FrictionDRBamActuator(BamActuator):
  """Scale dry/load-dependent BAM friction per world, leaving viscosity nominal."""

  def initialize(self, mj_model, model, data, device) -> None:
    super().initialize(mj_model, model, data, device)
    self.friction_scale = torch.ones_like(self.kp_scale)
    self.default_friction_scale = self.friction_scale.clone()

  def _compute_friction_budget(self, motor_torque, external_torque, stribeck_coeff):
    base = super()._compute_friction_budget(
      motor_torque, external_torque, stribeck_coeff
    )
    scale = getattr(self, "friction_scale", None)
    return base if scale is None else base * scale

  def set_friction_scale(self, env_ids, friction_scale: torch.Tensor) -> None:
    self.friction_scale[env_ids] = friction_scale

  def reset_friction_scale(self, env_ids) -> None:
    self.friction_scale[env_ids] = self.default_friction_scale[env_ids]


@dataclass(kw_only=True)
class FrictionDRBamActuatorCfg(BamActuatorCfg):
  """BAM configuration with the MD per-world friction scaling hook."""

  def build(self, entity, target_ids, target_names) -> FrictionDRBamActuator:
    return FrictionDRBamActuator(self, entity, target_ids, target_names)


@requires_model_fields("dof_frictionloss", "dof_damping")
def expand_bam_friction_fields(
  env: "ManagerBasedRlEnv", env_ids: torch.Tensor | None
) -> None:
  """Register BAM's per-world fields before simulation initialization.

  Install as a startup event. MJLab reads the decorator before applying events;
  no work is needed in the event body itself.
  """


def randomize_bam_friction(
  env: "ManagerBasedRlEnv",
  env_ids: torch.Tensor | None,
  scale_range: tuple[float, float],
  asset_cfg: SceneEntityCfg,
) -> None:
  """Replace each selected world's friction scale with a fresh reset sample."""
  lo, hi = scale_range
  if not 0 <= lo <= hi:
    raise ValueError("Friction scale range must satisfy 0 <= low <= high.")
  if env_ids is None:
    env_ids = torch.arange(env.num_envs, device=env.device)
  else:
    env_ids = env_ids.to(device=env.device, dtype=torch.long)
  for actuator in env.scene[asset_cfg.name].actuators:
    if isinstance(actuator, FrictionDRBamActuator):
      actuator.reset_friction_scale(env_ids)
      samples = torch.rand(len(env_ids), 1, device=env.device) * (hi - lo) + lo
      actuator.set_friction_scale(env_ids, samples)
