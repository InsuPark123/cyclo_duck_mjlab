# Copyright 2026 Pollen Robotics
# Copyright 2026 ROBOTIS CO., LTD.
# SPDX-License-Identifier: Apache-2.0
# Author: Insu Park
# Adapted from MD robot/microduck_constants.py at
# 273afe0b31c4ab365b9ff806a927b63ac92b5ddd. Uses Cyclo CAD MJCF assets.

"""Cyclo Duck walking asset with the MD BAM M6 actuator configuration."""

import copy
from pathlib import Path

import mujoco
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.managers.event_manager import EventTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.utils.spec_config import CollisionCfg

from source.actuators.bam import (
  FrictionDRBamActuatorCfg,
  expand_bam_friction_fields,
  randomize_bam_friction,
)

CYCLO_DUCK_XML_PATH = Path(__file__).with_name("cyclo_duck.xml")
CYCLO_DUCK_JOINT_NAMES = (
  "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
  "neck_pitch", "head_pitch", "head_yaw", "head_roll",
  "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle",
)
CYCLO_DUCK_FOOT_SITE_NAMES = ("left_foot", "right_foot")
CYCLO_DUCK_FOOT_COLLISION_GEOM_NAMES = (
  "left_foot_collision", "right_foot_collision",
)
CYCLO_DUCK_HOME_JOINT_POS = dict(zip(CYCLO_DUCK_JOINT_NAMES, (
  0.0, -0.0873, -0.4579, -0.0049, 0.4530,
  0.3491, 0.3491, 0.0, 0.0,
  0.0, 0.0873, 0.4579, 0.0049, -0.4530,
), strict=True))


def _load_cyclo_duck_spec() -> mujoco.MjSpec:
  # BAM converts the existing position actuators to motors at runtime, keeping
  # exactly fourteen actuators. The on-disk XML stays usable as a PD asset.
  return mujoco.MjSpec.from_file(str(CYCLO_DUCK_XML_PATH))


CYCLO_DUCK_CFG = EntityCfg(
  spec_fn=_load_cyclo_duck_spec,
  init_state=EntityCfg.InitialStateCfg(
    pos=(0.0, 0.0, 0.12),
    joint_pos=CYCLO_DUCK_HOME_JOINT_POS,
    joint_vel={".*": 0.0},
  ),
  collisions=(CollisionCfg(
    geom_names_expr=(".*_collision",),
    condim={r"^(left|right)_foot_collision$": 3, ".*_collision": 1},
    priority={r"^(left|right)_foot_collision$": 1},
    friction={r"^(left|right)_foot_collision$": (1.0,)},
  ),),
  articulation=EntityArticulationInfoCfg(
    actuators=(FrictionDRBamActuatorCfg(
      motor_name="xl330",
      model="m6",
      target_names_expr=CYCLO_DUCK_JOINT_NAMES,
      kp_fw=200.0,
      vin_range=(6.5, 8.2),
      vin_drop_gain_range=(0.0, 0.2),
      vin_min=6.0,
      delay_min_lag=3,
      delay_max_lag=6,
    ),),
    soft_joint_pos_limit_factor=0.9,
  ),
)


def get_cyclo_duck_cfg() -> EntityCfg:
  """Return an independent fourteen-joint BAM robot configuration."""
  return copy.deepcopy(CYCLO_DUCK_CFG)


def get_cyclo_duck_bam_events(
  entity_name: str = "robot",
) -> dict[str, EventTermCfg]:
  """Required field expansion and MD reset friction variation (0.9–1.1)."""
  return {
    "expand_bam_friction_fields": EventTermCfg(
      func=expand_bam_friction_fields, mode="startup",
    ),
    "randomize_bam_friction": EventTermCfg(
      func=randomize_bam_friction,
      mode="reset",
      params={
        "asset_cfg": SceneEntityCfg(entity_name),
        "scale_range": (0.9, 1.1),
      },
    ),
  }
