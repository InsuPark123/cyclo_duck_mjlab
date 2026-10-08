# Copyright 2026 ROBOTIS CO., LTD.
# SPDX-License-Identifier: Apache-2.0
# Author: Insu Park

"""Cyclo Duck flat walking task registration."""

from mjlab.tasks.registry import register_mjlab_task
from source.tasks.velocity.rl import VelocityOnPolicyRunner
from .env_cfgs import cyclo_duck_flat_env_cfg
from .rl_cfg import cyclo_duck_ppo_runner_cfg

register_mjlab_task(
  task_id="Cyclo-Velocity-Flat-Duck-v0",
  env_cfg=cyclo_duck_flat_env_cfg(),
  play_env_cfg=cyclo_duck_flat_env_cfg(play=True),
  rl_cfg=cyclo_duck_ppo_runner_cfg(),
  runner_cls=VelocityOnPolicyRunner,
)
