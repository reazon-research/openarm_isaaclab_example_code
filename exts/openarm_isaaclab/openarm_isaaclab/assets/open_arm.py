# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# Copyright (c) 2025, Reazon Holdings, inc.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR

from openarm_isaaclab import OPENARM_ROOT_DIR

OPEN_ARM_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{OPENARM_ROOT_DIR}/usds/openarm_v1_3.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
            # rigid_body_enabled=True,
            # max_linear_velocity=1000.0,
            # max_angular_velocity=1000.0,
            # enable_gyroscopic_forces=True,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=0,
            # sleep_threshold=0.005,
            # stabilization_threshold=0.001,
        ),
    ),
    init_state = ArticulationCfg.InitialStateCfg(
        joint_pos={
            "Revolute_1": 1.570796,
            "Revolute_2": 0.0,
            "Revolute_3": 0.0,
            "Revolute_4": 1.570796,
            "Revolute_5": 0.0,
            "Revolute_6": 0.0,
            "Revolute_7": 0.0,
            "Revolute_8": 0.0,
            "Revolute_9": 0.0,
            "Revolute_10": 0.0,
            "Slider_1": 0.0,
            "Slider_2": 0.0,
            # "EE_point": 0.0,
        },
    ),
    actuators = {
        "openarm_shoulder": ImplicitActuatorCfg(
            joint_names_expr=["Revolute_1", "Revolute_2", "Revolute_3", "Revolute_4"],
            effort_limit=100.0,
            velocity_limit=100.0,
            stiffness=10.0,
            damping=1.0,
        ),
        "openarm_forearm": ImplicitActuatorCfg(
            joint_names_expr=["Revolute_5", "Revolute_6", "Revolute_7"],
            effort_limit=100.0,
            velocity_limit=100.0,
            stiffness=10.0,
            damping=1.0,
        ),
        "openarm_gripper": ImplicitActuatorCfg(
            joint_names_expr=["Revolute_8", "Revolute_9", "Revolute_10", "Slider_1", "Slider_2"],# "EE_point"],
            effort_limit=100.0,
            velocity_limit=100.0,
            stiffness=10.0,
            damping=1.0,
        ),
    },
    soft_joint_pos_limit_factor = 1.0,
)
