import math

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import ArticulationCfg, AssetBaseCfg
from omni.isaac.lab.envs import ManagerBasedEnvCfg
from omni.isaac.lab.managers import EventTermCfg as EventTerm
from omni.isaac.lab.managers import ObservationGroupCfg as ObsGroup
from omni.isaac.lab.managers import ObservationTermCfg as ObsTerm
from omni.isaac.lab.managers import SceneEntityCfg
from omni.isaac.lab.scene import InteractiveSceneCfg
from omni.isaac.lab.utils import configclass

import omni.isaac.lab_tasks.manager_based.classic.cartpole.mdp as mdp

##
# 预定义的配置
##
# from omni.isaac.lab_assets.cartpole import CARTPOLE_CFG  # isort:skip
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
    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=500.0),
    )
    distant_light = AssetBaseCfg(
        prim_path="/World/DistantLight",
        spawn=sim_utils.DistantLightCfg(color=(0.9, 0.9, 0.9), intensity=2500.0),
        init_state=AssetBaseCfg.InitialStateCfg(rot=(0.738, 0.477, 0.477, 0.0)),
    )


@configclass
class ActionsCfg:
    """Action specifications for the MDP."""
    # asset_name表示作用的机器人，joint_names表示作用的机器人关节，scale表示该关节的力矩值将会乘以 100
    # Effort力矩 Position位置
    # 注释掉 effort代表不用力矩
    joint_effort1 = mdp.JointEffortActionCfg(asset_name="robot", joint_names=["joint1"], scale=100.0)
    joint_effort2 = mdp.JointEffortActionCfg(asset_name="robot", joint_names=["joint2"], scale=100.0)
    joint_effort3 = mdp.JointEffortActionCfg(asset_name="robot", joint_names=["joint3"], scale=100.0)
    joint_effort4 = mdp.JointEffortActionCfg(asset_name="robot", joint_names=["joint4"], scale=100.0)
    joint_effort5 = mdp.JointEffortActionCfg(asset_name="robot", joint_names=["joint5"], scale=100.0)
    joint_effort6 = mdp.JointEffortActionCfg(asset_name="robot", joint_names=["joint6"], scale=100.0)
    
    # #注释掉position表示不用位置
    # joint_Position1 = mdp.JointPositionActionCfg(asset_name="robot", joint_names=["joint1"], scale=1)
    # joint_Position2 = mdp.JointPositionActionCfg(asset_name="robot", joint_names=["joint2"], scale=1)
    # joint_Position3 = mdp.JointPositionActionCfg(asset_name="robot", joint_names=["joint3"], scale=1)
    # joint_Position4 = mdp.JointPositionActionCfg(asset_name="robot", joint_names=["joint4"], scale=1)
    # joint_Position5 = mdp.JointPositionActionCfg(asset_name="robot", joint_names=["joint5"], scale=1)
    # joint_Position6 = mdp.JointPositionActionCfg(asset_name="robot", joint_names=["joint6"], scale=1)


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

    # reset
    reset_cart_position = EventTerm(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=["joint1"]),
            "position_range": (-1.0, 1.0),
            "velocity_range": (-0.5, 0.5),
        },
    )

    reset_pole_position = EventTerm(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=["joint1"]),
            "position_range": (-0.25 * math.pi, 0.25 * math.pi),
            "velocity_range": (-0.25 * math.pi, 0.25 * math.pi),
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
        self.decimation = 1  # 仿真数据间隔几步重新渲染到可视化中
        self.episode_length_s = 5  # 每个仿真周期的持续时间为5秒
        # viewer settings
        self.viewer.eye = (8.0, 0.0, 5.0)
        self.viewer.lookat = (0.0, 0.0, 1.0)
        # simulation settings
        self.sim.dt = 1 / 1000  # 200HZ仿真频率
        self.sim.render_interval = self.decimation
