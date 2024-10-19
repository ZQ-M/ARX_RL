"""配置ARX X5 机器人"""

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators import ImplicitActuatorCfg
from omni.isaac.lab.assets import ArticulationCfg

##
# 机械臂配置
##

X5_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="/home/ultron/ARX_RL/X5/x5Effort.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(  # 刚体物理属性
            rigid_body_enabled=True,            # 启用刚体
            max_linear_velocity=1000.0,         # 最大线速度
            max_angular_velocity=1000.0,        # 最大角速度
            max_depenetration_velocity=5.0,   # 接触两个刚体相互接触的最大速度
            enable_gyroscopic_forces=True,      # 启用陀螺力。当物体快速旋转时，陀螺力会影响物体的旋转行为
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(  # 结构物理属性
            enabled_self_collisions=True,       # 开启自身碰撞检测
            solver_position_iteration_count=8,  # 位置迭代次数，表示物理引擎每次模拟中用于解决位置约束的迭代次数
            solver_velocity_iteration_count=0,  # 速度迭代次数，表示物理引擎在模拟中用于解决速度约束的迭代次数
            sleep_threshold=0.005,              # 关节系统速度低于此阈值，物体休眠，物理引擎停止其动态模拟
            stabilization_threshold=0.001,      # 关节系统接近此阈值时，稳定化处理，防止关节系统扰动或误差产生震颤
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        # pos机器人初始坐标，joint_pos关节初始角度
        pos=(0.0, 0.0, 0.0),
        joint_pos={
            "joint1": 0.0,
            "joint2": 1.0,  # 机械臂urdf的BUG 不抬起会造成碰撞箱锁死
            "joint3": 1.0,
            "joint4": 0.0,
            "joint5": 0.0,
            "joint6": 0.0
        },
    ),
    actuators={
        "joint1_actuator": ImplicitActuatorCfg(
            joint_names_expr=["joint1"],
            effort_limit=400.0,     # 最大力矩
            velocity_limit=100.0,   # 最大速度
            stiffness=100.0,          # 关节刚度  0.0 意味着关节是柔性的 
            damping=4.0,           # 关节阻尼系数，运动中能量损失，数值越高，运动的衰减越明显
        ),
        "joint2_actuator": ImplicitActuatorCfg(
            joint_names_expr=["joint2"],
            effort_limit=400.0,
            velocity_limit=100.0,
            stiffness=100.0,
            damping=4.0,
        ),
        "joint3_actuator": ImplicitActuatorCfg(
            joint_names_expr=["joint3"],
            effort_limit=400.0,
            velocity_limit=100.0,
            stiffness=80.0,
            damping=4.0,
        ),
        "joint4_actuator": ImplicitActuatorCfg(
            joint_names_expr=["joint4"],
            effort_limit=400.0,
            velocity_limit=100.0,
            stiffness=100.0,
            damping=4.0,
        ),
        "joint5_actuator": ImplicitActuatorCfg(
            joint_names_expr=["joint5"],
            effort_limit=400.0,
            velocity_limit=100.0,
            stiffness=100.0,
            damping=4.0,
        ),
        "joint6_actuator": ImplicitActuatorCfg(
            joint_names_expr=["joint6"],
            effort_limit=400.0,
            velocity_limit=100.0,
            stiffness=100.0,
            damping=4.0,
        ),
    },
    soft_joint_pos_limit_factor=4.0,
)
"""配置ARX X5 机器人"""

FRANKA_PANDA_HIGH_PD_CFG = X5_CFG.copy()
FRANKA_PANDA_HIGH_PD_CFG.spawn.rigid_props.disable_gravity = True
FRANKA_PANDA_HIGH_PD_CFG.actuators["joint1_actuator"].stiffness = 400.0
FRANKA_PANDA_HIGH_PD_CFG.actuators["joint1_actuator"].damping = 80.0
FRANKA_PANDA_HIGH_PD_CFG.actuators["joint5_actuator"].stiffness = 400.0
FRANKA_PANDA_HIGH_PD_CFG.actuators["joint5_actuator"].damping = 80.0
"""
具有更刚性 PD 控制的 Franka Emika Panda 机器人配置。
此配置对于使用差分 IK 的任务空间控制很有用
"""