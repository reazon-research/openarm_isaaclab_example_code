# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# Copyright (c) 2025, Reazon Holdings, inc.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import math

from isaaclab.utils import configclass

from isaaclab.managers import EventTermCfg as EventTerm
import isaaclab_tasks.manager_based.openarm_manipulation.reach.mdp as mdp
from isaaclab_tasks.manager_based.openarm_manipulation.reach.reach_env_cfg import ReachEnvCfg

from isaaclab_tasks.manager_based.openarm_manipulation.assets.openarm_single import OPEN_ARM_CFG

##
# Environment configuration
##

@configclass
class OpenArmReachEnvCfg(ReachEnvCfg):

    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # switch robot to OpenArm
        self.scene.robot = OPEN_ARM_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        # override rewards
        self.rewards.end_effector_position_tracking.params["asset_cfg"].body_names = ["URDF_swivel_rotor_v24_1"]
        self.rewards.end_effector_position_tracking_fine_grained.params["asset_cfg"].body_names = ["URDF_swivel_rotor_v24_1"]
        self.rewards.end_effector_orientation_tracking.params["asset_cfg"].body_names = ["URDF_swivel_rotor_v24_1"]

        # override actions
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot",
            joint_names=[
                "Revolute_.*",
                "Slider_.*",
                ],
            scale=0.5,
            use_default_offset=True,
        )

        # override command generator body
        # end-effector is along z-direction
        self.commands.ee_pose.body_name = "URDF_swivel_rotor_v24_1"

@configclass
class OpenArmReachEnvCfg_PLAY(OpenArmReachEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False
