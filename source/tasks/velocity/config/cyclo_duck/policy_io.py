# Copyright 2026 Pollen Robotics
# Copyright 2026 ROBOTIS CO., LTD.
# Copyright 2025, The mjlab Developers
# SPDX-License-Identifier: Apache-2.0
# Author: Insu Park
# Adapted from MD tasks/microduck_velocity_env_cfg.py at
# 273afe0b31c4ab365b9ff806a927b63ac92b5ddd and MJLab 1.3.0 velocity observations.

"""Cyclo Duck policy input/output contract, independent of rewards and PPO.

The containing task must provide robot/BAM events, commands twist(3),
head_pose(4), body_pose(6), and sensors feet_ground_contact/foot_height_scan.
Both foot sensors must report left then right. This module does not register
an RL task or create command samplers, rewards, terrain or termination rules.
"""

from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs.mdp import dr
from mjlab.envs.mdp.actions import JointPositionActionCfg
from mjlab.managers import EventTermCfg, ObservationGroupCfg, ObservationTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.tasks.velocity import mdp
from mjlab.utils.noise import UniformNoiseCfg as Unoise

from source.assets.robots.cyclo_duck import CYCLO_DUCK_JOINT_NAMES
from source.tasks.velocity.mdp import cyclo_duck_observations as duck_obs

CYCLO_DUCK_PHYSICS_DT = 0.005
CYCLO_DUCK_DECIMATION = 4
CYCLO_DUCK_POLICY_DT = CYCLO_DUCK_PHYSICS_DT * CYCLO_DUCK_DECIMATION
CYCLO_DUCK_ACTION_DIM = len(CYCLO_DUCK_JOINT_NAMES)

# Ordered concatenation contract. Dimensions are per policy step, without stacking.
CYCLO_DUCK_ACTOR_LAYOUT = (
  ("base_ang_vel", 3), ("projected_gravity", 3),
  ("joint_pos", 14), ("joint_vel", 14), ("actions", 14),
  ("command", 3), ("head_command", 4), ("body_command", 6),
)
CYCLO_DUCK_CRITIC_LAYOUT = (
  ("base_lin_vel", 3), *CYCLO_DUCK_ACTOR_LAYOUT[:6],
  ("foot_height", 2), ("foot_air_time", 2), ("foot_contact", 2),
  ("foot_contact_forces", 6), *CYCLO_DUCK_ACTOR_LAYOUT[6:],
)
CYCLO_DUCK_ACTOR_DIM = sum(size for _, size in CYCLO_DUCK_ACTOR_LAYOUT)
CYCLO_DUCK_CRITIC_DIM = sum(size for _, size in CYCLO_DUCK_CRITIC_LAYOUT)


def _joints() -> SceneEntityCfg:
  return SceneEntityCfg(
    "robot", joint_names=CYCLO_DUCK_JOINT_NAMES, preserve_order=True
  )


def _command(name: str) -> ObservationTermCfg:
  return ObservationTermCfg(func=mdp.generated_commands, params={"command_name": name})


def get_cyclo_duck_observation_cfg() -> dict[str, ObservationGroupCfg]:
  """Fresh actor (61) / critic (76) terms in the MD concatenation order."""
  actor = {
    "base_ang_vel": ObservationTermCfg(
      func=duck_obs.base_ang_vel_imu_misaligned,
      params={"max_angle_deg": 6.0, "asset_cfg": SceneEntityCfg("robot")},
      noise=Unoise(n_min=-0.03, n_max=0.03),
      delay_min_lag=0, delay_max_lag=1, delay_update_period=64,
    ),
    "projected_gravity": ObservationTermCfg(
      func=duck_obs.projected_gravity_imu_misaligned,
      params={"max_angle_deg": 6.0, "asset_cfg": SceneEntityCfg("robot")},
      noise=Unoise(n_min=-0.01, n_max=0.01),
      delay_min_lag=0, delay_max_lag=1, delay_update_period=64,
    ),
    "joint_pos": ObservationTermCfg(
      func=mdp.joint_pos_rel, params={"asset_cfg": _joints(), "biased": True},
      noise=Unoise(n_min=-0.001, n_max=0.001),
    ),
    "joint_vel": ObservationTermCfg(
      func=mdp.joint_vel_rel, params={"asset_cfg": _joints()},
      noise=Unoise(n_min=-0.25, n_max=0.25),
      delay_min_lag=1, delay_max_lag=1,
    ),
    "actions": ObservationTermCfg(func=mdp.last_action),
    "command": _command("twist"),
    "head_command": _command("head_pose"),
    "body_command": _command("body_pose"),
  }
  # Construct independently: actor noise, bias and delay must not leak to critic.
  critic = {
    "base_lin_vel": ObservationTermCfg(func=mdp.base_lin_vel, scale=1.0),
    "base_ang_vel": ObservationTermCfg(
      func=mdp.builtin_sensor, params={"sensor_name": "robot/imu_ang_vel"},
    ),
    "projected_gravity": ObservationTermCfg(func=mdp.projected_gravity),
    "joint_pos": ObservationTermCfg(
      func=mdp.joint_pos_rel, params={"asset_cfg": _joints(), "biased": False},
    ),
    "joint_vel": ObservationTermCfg(
      func=mdp.joint_vel_rel, params={"asset_cfg": _joints()},
    ),
    "actions": ObservationTermCfg(func=mdp.last_action),
    "command": _command("twist"),
    "foot_height": ObservationTermCfg(
      func=duck_obs.foot_height_safe, params={"sensor_name": "foot_height_scan"},
    ),
    "foot_air_time": ObservationTermCfg(
      func=duck_obs.foot_air_time_safe, params={"sensor_name": "feet_ground_contact"},
    ),
    "foot_contact": ObservationTermCfg(
      func=mdp.foot_contact, params={"sensor_name": "feet_ground_contact"},
    ),
    "foot_contact_forces": ObservationTermCfg(
      func=duck_obs.foot_contact_forces_safe,
      params={"sensor_name": "feet_ground_contact"},
    ),
    "head_command": _command("head_pose"),
    "body_command": _command("body_pose"),
  }
  return {
    "actor": ObservationGroupCfg(
      terms=actor, concatenate_terms=True, enable_corruption=True,
    ),
    "critic": ObservationGroupCfg(
      terms=critic, concatenate_terms=True, enable_corruption=False,
    ),
  }


def get_cyclo_duck_action_cfg() -> dict[str, JointPositionActionCfg]:
  """Fourteen HOME-relative joint targets, one radian per action unit."""
  return {"joint_pos": JointPositionActionCfg(
    entity_name="robot", actuator_names=CYCLO_DUCK_JOINT_NAMES,
    preserve_order=True, scale=1.0, use_default_offset=True,
  )}


def configure_cyclo_duck_policy_io(cfg: ManagerBasedRlEnvCfg) -> None:
  """Apply the MD input/output contract and sensor bias to a Duck task config."""
  cfg.observations = get_cyclo_duck_observation_cfg()
  cfg.actions = get_cyclo_duck_action_cfg()
  cfg.sim.mujoco.timestep = CYCLO_DUCK_PHYSICS_DT
  cfg.decimation = CYCLO_DUCK_DECIMATION
  cfg.events["encoder_bias"] = EventTermCfg(
    func=dr.encoder_bias, mode="startup",
    params={"asset_cfg": _joints(), "bias_range": (-0.015, 0.015)},
  )
