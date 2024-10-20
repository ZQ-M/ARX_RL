""" 配置场景 """
import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from omni.isaac.lab.envs import ManagerBasedEnvCfg
from omni.isaac.lab.managers import EventTermCfg as EventTerm
from omni.isaac.lab.managers import ObservationGroupCfg as ObsGroup
from omni.isaac.lab.managers import ObservationTermCfg as ObsTerm
from omni.isaac.lab.managers import SceneEntityCfg
from omni.isaac.lab.scene import InteractiveSceneCfg
from omni.isaac.lab.utils import configclass

import omni.isaac.lab_tasks.manager_based.classic.cartpole.mdp as mdp

from ARMX5 import X5_CFG


@configclass
class x5SceneCfg(InteractiveSceneCfg):
    """ 配置ARX X5 """

    # ground plane
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(size=(100.0, 100.0)),
    )

    # X5
    robot: ArticulationCfg = X5_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    # lights
    distant_light = AssetBaseCfg(
        prim_path="/World/DistantLight",
        spawn=sim_utils.DistantLightCfg(color=(0.9, 0.9, 0.9), intensity=2500.0),
        init_state=AssetBaseCfg.InitialStateCfg(rot=(0.738, 0.477, 0.477, 0.0)),
    )

    # CUP
    Cup = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Cup",
        spawn=sim_utils.UsdFileCfg(
            usd_path="/home/ultron/ARX_RL/X5/CUP.usd",  # 指定USD文件的路径
            visible=True,                                          # 是否可见
            mass_props=sim_utils.MassPropertiesCfg(mass=1.0),      # 质量
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),        # 刚体
            collision_props=sim_utils.CollisionPropertiesCfg(),    # 碰撞属性
        ),
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(0.5, 0.0, 0.0),
            rot=(0.0, 0.0, 0.0, 1.0)
        )
    )

    # 测试用蓝色的长方体
    BlueCuboid = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Cuboid",
        spawn=sim_utils.CuboidCfg(
            size=(0.1, 0.1, 0.1),       # 长方体的大小是？长宽高
            visible=True,               # 是否可见？
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),         # 刚体
            mass_props=sim_utils.MassPropertiesCfg(mass=1.0),       # 质量
            collision_props=sim_utils.CollisionPropertiesCfg(),     # 碰撞箱
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 0.0, 1.0)),  # 颜色
        ),
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(0.5, 0.0, 0.0),
            rot=(0.0, 0.0, 0.0, 1.0)
        )
    )


@configclass
class ActionsCfg:
    """Action specifications for the MDP."""
    # asset_name表示作用的机器人，joint_names表示作用的机器人关节，scale表示该关节的力矩值将会乘以 100
    # Effort力矩 Position位置
    # # 注释掉 effort代表不用力矩
    # joint_effort1 = mdp.JointEffortActionCfg(asset_name="robot", joint_names=["joint1"], scale=100.0)
    # joint_effort2 = mdp.JointEffortActionCfg(asset_name="robot", joint_names=["joint2"], scale=100.0)
    # joint_effort3 = mdp.JointEffortActionCfg(asset_name="robot", joint_names=["joint3"], scale=100.0)
    # joint_effort4 = mdp.JointEffortActionCfg(asset_name="robot", joint_names=["joint4"], scale=100.0)
    # joint_effort5 = mdp.JointEffortActionCfg(asset_name="robot", joint_names=["joint5"], scale=100.0)
    # joint_effort6 = mdp.JointEffortActionCfg(asset_name="robot", joint_names=["joint6"], scale=100.0)

    # 注释掉position表示不用位置
    joint_Position1 = mdp.JointPositionActionCfg(asset_name="robot", joint_names=["joint1"], scale=1)
    joint_Position2 = mdp.JointPositionActionCfg(asset_name="robot", joint_names=["joint2"], scale=1)
    joint_Position3 = mdp.JointPositionActionCfg(asset_name="robot", joint_names=["joint3"], scale=1)
    joint_Position4 = mdp.JointPositionActionCfg(asset_name="robot", joint_names=["joint4"], scale=1)
    joint_Position5 = mdp.JointPositionActionCfg(asset_name="robot", joint_names=["joint5"], scale=1)
    joint_Position6 = mdp.JointPositionActionCfg(asset_name="robot", joint_names=["joint6"], scale=1)


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        # observation terms (order preserved)
        joint_pos_rel = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel_rel = ObsTerm(func=mdp.joint_vel_rel)

        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    """Configuration for events."""

    # 重置场景到原样子
    reset_cup_position = EventTerm(
        func=mdp.reset_scene_to_default,
        mode="reset",
    )

    # 重置CUP位置
    reset_cup_pos = EventTerm(
        func=mdp.reset_root_state_uniform,  # 牢记这个reset只支持 RigidObject 和 Articulation
        mode="reset",
        params={
            "pose_range": {
                "x": (-0.5, 0.5),
                "y": (-0.5, 0.5),
                "z": (0.0 , 1.0),
                "yaw": (-3.14, 3.14)},
            "velocity_range": {
                "x": (0.0, 0.0),
                "y": (0.0, 0.0),
                "z": (0.0, 0.0),
            },
            "asset_cfg": SceneEntityCfg("BlueCuboid"),
        },
    )


##
# Environment configuration
##
@configclass
class X5EnvCfg(ManagerBasedEnvCfg):
    """Configuration for the cartpole environment."""

    # Scene settings
    scene: x5SceneCfg = x5SceneCfg(num_envs=9, env_spacing=2)
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    events: EventCfg = EventCfg()

    # Post initialization
    def __post_init__(self) -> None:
        """初始化"""
        # general setting
        self.decimation = 5  # 仿真数据间隔几步重新渲染到可视化中
        self.episode_length_s = 5  # 每个仿真周期的持续时间为5秒
        # viewer settings
        self.viewer.eye = (8.0, 0.0, 5.0)
        self.viewer.lookat = (0.0, 0.0, 1.0)
        # simulation settings
        self.sim.dt = 1 / 1000  # 200HZ仿真频率
        self.sim.render_interval = self.decimation
