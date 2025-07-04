import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

from isaaclab_tasks.manager_based.openarm_manipulation import OPENARM_ROOT_DIR

OPEN_ARM_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{OPENARM_ROOT_DIR}/usds/openarm_v1_3.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=0,
        ),
    ),
    init_state = ArticulationCfg.InitialStateCfg(
        joint_pos={
            "Revolute_1": 1.57,
            "Revolute_2": 0.0,
            "Revolute_3": 1.57,
            "Revolute_4": 1.57,
            "Revolute_5": 0.0,
            "Revolute_6": 0.0,
            "Revolute_7": 0.0,
            "Revolute_8": 0.0,
            "Revolute_9": 0.0,
            "Revolute_10": 0.0,
            "Slider_1": 0.0,
            "Slider_2": 0.0,
        },
    ),
    actuators = {
        "openarm_shoulder": ImplicitActuatorCfg(
            joint_names_expr=["Revolute_1", "Revolute_2", "Revolute_3", "Revolute_4"],
            effort_limit_sim=87.0,
            velocity_limit_sim=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "openarm_forearm": ImplicitActuatorCfg(
            joint_names_expr=["Revolute_5", "Revolute_6", "Revolute_7", "Revolute_8", "Revolute_9", "Revolute_10"],
            effort_limit_sim=12.0,
            velocity_limit_sim=2.61,
            stiffness=80.0,
            damping=4.0,
        ),
        "openarm_gripper": ImplicitActuatorCfg(
            joint_names_expr=["Slider_1", "Slider_2"],
            effort_limit=100.0,
            velocity_limit=100.0,
            stiffness=10.0,
            damping=1.5,
        ),
    },
    soft_joint_pos_limit_factor = 1.0,
)
