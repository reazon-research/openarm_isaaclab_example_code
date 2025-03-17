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
        usd_path=f"{OPENARM_ROOT_DIR}/usds/openarm_grip.usd",
        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, 
            solver_position_iteration_count=8, 
            solver_velocity_iteration_count=0
        ),
    ),
    init_state = ArticulationCfg.InitialStateCfg(
        joint_pos={
            "rev1": 1.57,
            "rev2": -1, #-1.527,      # (-3.05433 + 0) / 2
            "rev3": 0.0,
            "rev4": -0.50,        # (-2.0 + 0) / 2
            "rev5": 0.0,
            "rev6": 0.6354,      # (-0.3 + 1.5708) / 2
            "rev7": 0.0,
            "slider_left": -0.02275,   # (-0.0455 + 0) / 2
            "slider_right": -0.02275,  # 同上
        },
    ),
    actuators = {
        "openarm_shoulder": ImplicitActuatorCfg(
            joint_names_expr=["rev1", "rev2", "rev3", "rev4"],
            effort_limit=100.0,        # ※各ジョイントに十分なトルク余裕を与える例
            velocity_limit=2.5,        # ctrlrangeから推定した動作速度（例）
            stiffness=100.0,            # panda例に倣い設定
            damping=1.0,
        ),
        "openarm_forearm": ImplicitActuatorCfg(
            joint_names_expr=["rev5", "rev6", "rev7"],
            effort_limit=100.0,         # 前腕部は比較的小さいトルクで十分と想定
            velocity_limit=2.0,
            stiffness=100.0,
            damping=1.0,
        ),
        "openarm_gripper": ImplicitActuatorCfg(
            joint_names_expr=["slider_left", "slider_right"],
            effort_limit=200.0,        # グリッパは比較的高い力が必要
            velocity_limit=0.2,
            stiffness=2000.0,
            damping=100.0,
        ),
    },
    soft_joint_pos_limit_factor = 1.0,
)