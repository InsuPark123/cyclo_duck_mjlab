# Copyright 2026 Pollen Robotics
# Copyright 2026 ROBOTIS CO., LTD.
# Copyright 2025, The mjlab Developers
# SPDX-License-Identifier: Apache-2.0
# Author: Insu Park
# Adapted from MD tasks/microduck_velocity_env_cfg.py at
# 273afe0b31c4ab365b9ff806a927b63ac92b5ddd; flat walking only.

"""Cyclo Duck flat walking with MD commands, rewards and randomization."""

import copy
import math

import mujoco
from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs.mdp import dr
from mjlab.managers import CurriculumTermCfg, EventTermCfg, RewardTermCfg, TerminationTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.sensor import ContactMatch, ContactSensorCfg, ObjRef, RingPatternCfg, TerrainHeightSensorCfg
from mjlab.tasks.velocity import mdp
from mjlab.tasks.velocity.velocity_env_cfg import make_velocity_env_cfg

from source.assets.robots.cyclo_duck import (
  CYCLO_DUCK_FOOT_SITE_NAMES, CYCLO_DUCK_HOME_JOINT_POS, CYCLO_DUCK_XML_PATH,
  get_cyclo_duck_cfg, get_cyclo_duck_bam_events,
)
from source.tasks.velocity.mdp import cyclo_duck as duck_mdp
from .policy_io import configure_cyclo_duck_policy_io

HEAD_JOINTS = ("neck_pitch", "head_pitch", "head_yaw", "head_roll")
BODY_RANGES = ((-0.005, 0.005),) * 3 + ((-0.05, 0.05),) * 3


def _head_range_stages():
  # MD's late pitch-command caps exceed this walking skeleton's soft limits.
  # Clip commands, not joint limits, so the tracking objective stays reachable.
  spec = mujoco.MjSpec.from_file(str(CYCLO_DUCK_XML_PATH))
  limits = []
  for name in HEAD_JOINTS:
    lo, hi = map(float, spec.joint(name).range)
    center, half = (lo + hi) / 2, (hi - lo) * 0.9 / 2
    home = CYCLO_DUCK_HOME_JOINT_POS[name]
    limits.append((center - half - home, center + half - home))
  caps = ((0.05, 0.05, 0.07, 0.015), (0.17, 0.17, 0.21, 0.047),
          (0.39, 0.39, 0.49, 0.11), (0.72, 0.72, 0.91, 0.20),
          (1.10, 1.10, 1.40, 0.31))
  return [{"step": i * 500 * 24,
           "ranges": tuple((max(-c, lo), min(c, hi)) for c, (lo, hi) in zip(stage, limits))}
          for i, stage in enumerate(caps)]


def cyclo_duck_flat_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
  cfg = copy.deepcopy(make_velocity_env_cfg())
  cfg.scene.entities = {"robot": get_cyclo_duck_cfg()}
  cfg.scene.terrain.terrain_type = "plane"
  cfg.scene.terrain.terrain_generator = None
  cfg.viewer.body_name = "trunk_base"
  cfg.viewer.distance = 0.7
  cfg.scene.sensors = (
    ContactSensorCfg(
      name="feet_ground_contact",
      primary=ContactMatch(mode="geom", pattern=r"^(left_foot_collision|right_foot_collision)$", entity="robot"),
      secondary=ContactMatch(mode="body", pattern="terrain"),
      fields=("found", "force"), reduce="netforce", num_slots=1, track_air_time=True,
    ),
    ContactSensorCfg(
      name="self_collision",
      primary=ContactMatch(mode="subtree", pattern="trunk_base", entity="robot"),
      secondary=ContactMatch(mode="subtree", pattern="trunk_base", entity="robot"),
      fields=("found",), reduce="none", num_slots=1,
    ),
    TerrainHeightSensorCfg(
      name="foot_height_scan",
      frame=tuple(ObjRef(type="site", name=n, entity="robot") for n in CYCLO_DUCK_FOOT_SITE_NAMES),
      pattern=RingPatternCfg.single_ring(radius=0.04, num_samples=2),
      ray_alignment="yaw", max_distance=1.0, exclude_parent_body=True,
      include_geom_groups=(0,), debug_vis=False,
    ),
  )
  configure_cyclo_duck_policy_io(cfg)
  command = cfg.commands["twist"]
  cfg.commands["twist"] = duck_mdp.DuckVelocityCommandCfg(**vars(command))
  command = cfg.commands["twist"]
  command.rel_standing_envs = 0.02
  command.rel_heading_envs = 0.0
  command.ranges.lin_vel_x = (-0.4, 0.4)
  command.ranges.lin_vel_y = (-0.3, 0.3)
  command.ranges.ang_vel_z = (-1.0, 1.0)
  command.viz.z_offset = 0.5
  head_stages = _head_range_stages()
  cfg.commands["head_pose"] = duck_mdp.UniformPoseCommandCfg(
    resampling_time_range=(2.0, 5.0), ranges=head_stages[0]["ranges"],
  )
  cfg.commands["body_pose"] = duck_mdp.UniformPoseCommandCfg(
    resampling_time_range=(2.0, 5.0), ranges=BODY_RANGES,
  )

  standing = {".*hip_yaw.*": 0.1, ".*hip_roll.*": 0.05, ".*hip_pitch.*": 0.15,
              ".*knee.*": 0.15, ".*ankle.*": 0.1}
  walking = {".*hip_yaw.*": 0.3, ".*hip_roll.*": 0.05, ".*hip_pitch.*": 0.4,
             ".*knee.*": 0.4, ".*ankle.*": 0.25}
  cfg.rewards["pose"].params.update(
    std_standing=standing, std_walking=walking, std_running=walking,
    walking_threshold=0.01,
    asset_cfg=SceneEntityCfg("robot", joint_names=(r"^(left|right)_.*",)),
  )
  cfg.rewards["upright"].weight = 2.0
  cfg.rewards["upright"].params.update(std=math.sqrt(0.05), asset_cfg=SceneEntityCfg("robot", body_names=("trunk_base",)))
  cfg.rewards["body_ang_vel"].weight = -0.05
  cfg.rewards["body_ang_vel"].params["asset_cfg"].body_names = ("trunk_base",)
  cfg.rewards["angular_momentum"].weight = -0.02
  cfg.rewards["track_linear_velocity"].params["std"] = math.sqrt(0.1)
  cfg.rewards.pop("soft_landing")
  for name in ("foot_clearance", "foot_slip"):
    cfg.rewards[name].params["asset_cfg"].site_names = CYCLO_DUCK_FOOT_SITE_NAMES
  for name in ("air_time", "foot_clearance", "foot_swing_height", "foot_slip"):
    cfg.rewards[name].params["command_threshold"] = 0.01
  cfg.rewards["air_time"].weight = 3.0
  cfg.rewards["air_time"].params.update(threshold_min=0.125, threshold_max=0.300)
  for name in ("foot_clearance", "foot_swing_height"):
    cfg.rewards[name].params["target_height"] = 0.02
  cfg.rewards["self_collisions"] = RewardTermCfg(
    func=mdp.self_collision_cost, weight=-1.0, params={"sensor_name": "self_collision"},
  )
  cfg.rewards["head_pose_tracking"] = RewardTermCfg(
    func=duck_mdp.head_pose_tracking, weight=2.0,
    params={"command_name": "head_pose", "std": 0.5,
            "asset_cfg": SceneEntityCfg("robot", joint_names=HEAD_JOINTS, preserve_order=True)},
  )
  cfg.rewards["head_pose_bias"] = RewardTermCfg(
    func=duck_mdp.head_pose_bias_penalty, weight=0.0,
    params={"command_name": "head_pose", "tau_s": 1.0,
            "asset_cfg": SceneEntityCfg("robot", joint_names=HEAD_JOINTS, preserve_order=True)},
  )
  # MD body-pose reward is disabled (weight zero); retain its six command inputs.

  cfg.events.update(get_cyclo_duck_bam_events())
  cfg.events["reset_head_history"] = EventTermCfg(func=duck_mdp.reset_head_history, mode="reset")
  # reset_root_state_uniform adds this offset to the asset's 0.12 m HOME height.
  cfg.events["reset_base"].params["pose_range"]["z"] = (0.0, 0.01)
  cfg.events["foot_friction"].params.update(
    asset_cfg=SceneEntityCfg("robot", geom_names=("left_foot_collision", "right_foot_collision")),
    ranges=(0.7, 1.3),
  )
  cfg.events["push_robot"] = EventTermCfg(
    func=mdp.push_by_setting_velocity, mode="interval",
    interval_range_s=(0.5, 1.0) if play else (3.0, 6.0),
    params={"velocity_range": {"x": (-0.3, 0.3), "y": (-0.3, 0.3)},
            "asset_cfg": SceneEntityCfg("robot")},
  )
  # Replace the base template's unconfigured COM event with MD reset events.
  cfg.events.pop("base_com", None)
  for name, bodies in (
    ("randomize_com", ("trunk_base",)),
    # bearing_roll is a hip body; deliberately retained for MD recipe parity.
    ("randomize_head_com", ("neck", "neck_pitch", "yaw_roll_motion", "jaw_soft", "bearing_roll")),
  ):
    cfg.events[name] = EventTermCfg(
      func=dr.body_ipos, mode="reset",
      params={"asset_cfg": SceneEntityCfg("robot", body_names=bodies),
              "operation": "add", "ranges": (-0.003, 0.003)},
    )
  cfg.events["randomize_mass_inertia"] = EventTermCfg(
    func=dr.pseudo_inertia, mode="startup",
    params={"asset_cfg": SceneEntityCfg("robot", body_names=("trunk_base",)),
            "alpha_range": (math.log(0.95) / 2, math.log(1.05) / 2)},
  )
  cfg.events["randomize_armature"] = EventTermCfg(
    func=dr.joint_armature, mode="reset",
    params={"asset_cfg": SceneEntityCfg("robot", joint_names=(".*",)),
            "operation": "scale", "ranges": (0.9, 1.1)},
  )
  cfg.terminations["nan_state"] = TerminationTermCfg(
    func=duck_mdp.nonfinite_state,
    params={"asset_cfg": SceneEntityCfg("robot"), "sensor_name": "feet_ground_contact"},
  )
  cfg.curriculum = {
    "action_rate_weight": CurriculumTermCfg(func=duck_mdp.reward_weight, params={
      "reward_name": "action_rate_l2", "weight_stages": [
        {"step": i * 24, "weight": w} for i, w in
        ((0, -0.1), (500, -0.2), (750, -0.4), (1000, -0.6), (1250, -0.8), (1500, -1.0))]}),
    "standing_envs": CurriculumTermCfg(func=duck_mdp.standing_envs_curriculum, params={
      "command_name": "twist", "standing_stages": [
        {"step": i * 24, "rel_standing_envs": p} for i, p in
        ((0, .02), (500, .05), (750, .1), (1000, .15), (1500, .2), (2000, .25))]}),
    "head_pose_range": CurriculumTermCfg(func=duck_mdp.pose_command_range_curriculum,
      params={"command_name": "head_pose", "range_stages": head_stages}),
    "body_pose_range": CurriculumTermCfg(func=duck_mdp.pose_command_range_curriculum,
      params={"command_name": "body_pose", "range_stages": [{"step": 0, "ranges": BODY_RANGES}]}),
    "head_pose_bias_weight": CurriculumTermCfg(func=duck_mdp.reward_weight, params={
      "reward_name": "head_pose_bias", "weight_stages": [
        {"step": i * 24, "weight": w} for i, w in ((0, 0.), (600, 1.), (1000, 2.), (1500, 3.))]}),
  }
  for name, event, stages in (
    ("com_range", "randomize_com", ((0,.003),(500,.005),(1000,.01),(1500,.015))),
    ("head_com_range", "randomize_head_com", ((0,.003),(500,.005),(1000,.01))),
  ):
    cfg.curriculum[name] = CurriculumTermCfg(func=duck_mdp.com_range_curriculum, params={
      "event_name": event, "range_stages": [{"step": i * 24, "range": r} for i, r in stages]})
  return cfg
