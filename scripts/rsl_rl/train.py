
"""
打开仿真，设置仿真参数并开始仿真
设计仿真场景
设置启动选项和启动参数
author:ZQM
"""
import argparse
import torch
from omni.isaac.lab.app import AppLauncher

# 打开SIM仿真
parser = argparse.ArgumentParser(description="Tutorial on ZQM creating ARX.")#
parser.add_argument("--size", type=float, default=1.0, help="传进一个浮点参数，暂时用于控制物块的体积(失效)")#自定义的传入参数
parser.add_argument("--number", type=int , default=1 , help="传进一个整形参数，用于控制实时仿真的数量")#自定义的传入参数
#官方定义的参数
parser.add_argument(
    "--width", type=int, default=1280, help="视角默认渲染分辨率宽度"
)
parser.add_argument(
    "--height", type=int, default=720, help="视角默认渲染分辨率高度"
)

AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# Isaac Sim 和其他库中导入不同的 Python 模块
import omni.isaac.core.utils.prims as prim_utils
import omni.isaac.lab.sim as sim_utils#仿真环境
from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR# 导入示例中的一个资产（桌子）
from omni.isaac.lab.utils import configclass

# #第三方自定义库（ISAAC扩展）
# import arx.isaac.interfaces
# import ext_template.tasks

# 生成随机位置
def random_position(x_range=(-1.0, 1.0), y_range=(-1.0, 1.0), z_range=(0.0, 2.0)):
    x = torch.rand(1).item() * (x_range[1] - x_range[0]) + x_range[0]
    y = torch.rand(1).item() * (y_range[1] - y_range[0]) + y_range[0]
    z = torch.rand(1).item() * (z_range[1] - z_range[0]) + z_range[0]
    return [x, y, z]

def random_quaternion():#生成随机四元数
    u1 = torch.rand(1)  
    u2 = torch.rand(1) * 2 * torch.pi  
    u3 = torch.rand(1) * 2 * torch.pi  
    sqrt1_minus_u1 = torch.sqrt(1 - u1) 
    sqrt_u1 = torch.sqrt(u1)
    x = sqrt1_minus_u1 * torch.sin(u2)  
    y = sqrt1_minus_u1 * torch.cos(u2)
    z = sqrt_u1 * torch.sin(u3)
    w = sqrt_u1 * torch.cos(u3)
    return [x.item(), y.item(), z.item(), w.item()]

#设计初始化场景（不纳入环境单元的东西，例如地面和光照）
def design_scene():
    # 神说要有地面
    cfg_ground = sim_utils.GroundPlaneCfg()#创造通用世界地面
    cfg_ground.func("/World/defaultGroundPlane", cfg_ground)#放置到环境中

    # 神说要有光
    cfg_light_distant = sim_utils.DistantLightCfg(
        intensity=3000.0,
        color=(0.75, 0.75, 0.75),
    )
    cfg_light_distant.func("/World/lightDistant", cfg_light_distant, translation=(1, 0, 10))

    # 神说要有文件夹
    prim_utils.create_prim("/World/Objects", "Xform")

import math
import omni.isaac.lab.envs.mdp as mdp
from omni.isaac.lab.envs import ManagerBasedEnv,ManagerBasedEnvCfg #用来创建基础强化环境单元并配置单元，ISAACLAB将此基础类进行复制
from omni.isaac.lab.managers import EventTermCfg as EventTerm
from omni.isaac.lab.managers import ObservationGroupCfg as ObsGroup #观察者设定
from omni.isaac.lab.managers import ObservationTermCfg as ObsTerm #观察者设定
from omni.isaac.lab.managers import SceneEntityCfg #场景中的对象
from omni.isaac.lab.scene import InteractiveSceneCfg#自定义场景所用的库

@configclass
class ActionsCfg:
    """环境中机器人具体的动作设置(暂时为空)"""

@configclass
class ObservationsCfg:
    """Observation specifications for the environment."""

    @configclass
    class PolicyCfg(ObsGroup):
        """观察者的政策制定"""
        # observation terms (order preserved)
        joint_pos_rel = ObsTerm(func=mdp.joint_pos_rel) #实时的滑块位置
        joint_vel_rel = ObsTerm(func=mdp.joint_vel_rel) #实时的滑块速度
        def __post_init__(self) -> None:
            self.enable_corruption = False #是否引用数据噪声来模拟现实环境
            self.concatenate_terms = True  #是否将多个观察对象串联表示出来

    # observation groups
    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    """事件的政策制定"""

    # on startup
    add_pole_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=["pole"]),
            "mass_distribution_params": (0.1, 0.5),
            "operation": "add",
        },
    )

    # on reset
    reset_cart_position = EventTerm(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=["slider_to_cart"]),
            "position_range": (-1.0, 1.0),
            "velocity_range": (-0.1, 0.1),
        },
    )

    reset_pole_position = EventTerm(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=["cart_to_pole"]),
            "position_range": (-0.125 * math.pi, 0.125 * math.pi),
            "velocity_range": (-0.01 * math.pi, 0.01 * math.pi),
        },
    )

@configclass
class ARXArmSceneCfg(InteractiveSceneCfg):
    """自定义的ARM场景设置"""
    def __post_init__(self):
        
        # # 神说要从外部导入一张桌子
        # cfg = sim_utils.UsdFileCfg(usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/SeattleLabTable/table_instanceable.usd")
        # cfg.func("/World/Objects/Table", cfg, translation=(0.0, 0.0, 1.05))

        super().__post_init__()  # 调用父类的初始化
        self.num_envs = 512  # 自定义环境单元数量
        self.env_spacing = 3.0  # 自定义环境单元之间的间隔
        self.custom_gravity = [0.0, 0.0, -9.8]  # 自定义环境单元重力（单位：m/s）
        # 其他自定义场景设置可以在这里添加
        # 例如添加额外的场景实体、关节设置等
@configclass
class ARXArmEnvCfg(ManagerBasedEnvCfg):
    """配置整体强化环境Scene + Observation + Action + Event"""

    # Scene settings
    scene: ARXArmSceneCfg = ARXArmSceneCfg(num_envs=512, env_spacing=3.0)  # 使用自定义的场景配置
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()# 观察者设置（返回值）
    actionsla了: ActionsCfg = ActionsCfg()# 动作设置（尝试）
    events:EventCfg = EventCfg()# 事件设置（政策）

    def __post_init__(self):
        """Post initialization."""
        # viewer settings
        self.viewer.eye = [4.5, 0.0, 6.0]# 第一人称摄像机所在位置
        self.viewer.lookat = [0.0, 0.0, 2.0]# 第一人称摄像机所看向的目标点
        # step settings
        self.decimation = 4  # env step every 4 sim steps: 200Hz / 4 = 50Hz
        # simulation settings
        self.sim.dt = 0.005  # sim step every 5ms: 200Hz

from omni.isaac.lab_tasks.manager_based.classic.cartpole.cartpole_env_cfg import CartpoleSceneCfg
@configclass
class CartpoleEnvCfg(ManagerBasedEnvCfg):

    # Scene settings
    scene = CartpoleSceneCfg(num_envs=1024, env_spacing=2.5)
    # Basic settings
    observations = ObservationsCfg()
    actions = ActionsCfg()
    events = EventCfg()

    def __post_init__(self):
        """Post initialization."""
        # viewer settings
        self.viewer.eye = [4.5, 0.0, 6.0]
        self.viewer.lookat = [0.0, 0.0, 2.0]
        # step settings
        self.decimation = 4  # env step every 4 sim steps: 500Hz / 4 = 125Hz
        # simulation settings
        self.sim.dt = 0.002  # sim step every 2ms: 500Hz

def main():
    """导入环境单元"""
    env_cfg = CartpoleEnvCfg()
    env_cfg.scene.num_envs = args_cli.number# 环境单元数量设置
    # setup base environment
    env = ManagerBasedEnv(cfg=env_cfg)
    
    # 仿真循环    
    count = 0
    while simulation_app.is_running():
        with torch.inference_mode():
            if count % 500 == 0:# 渲染 500 步长后重置
                count = 0
                env.reset()
                print("[INFO]: Resetting environment and robot")
            # # sample random actions
            # joint_efforts = torch.randn_like(env.action_manager.action)
            # # step the environment
            # obs, _ = env.step(joint_efforts)
            # print current orientation of pole
            # update counter
            count += 1

    # close the environment
    env.close()


if __name__ == "__main__":
    main()
    # 关闭SIM
    simulation_app.close()

    
    
#启动脚本需要的命令
# source ~/IsaacLab/_isaac_sim/setup_conda_env.sh
# /home/ultron/IsaacLab/isaaclab.sh -p /home/ultron/L5_Handle/train.py -h

# 是的，你可以通过 Isaac Sim 的接口修改由 Isaac Lab 创建的物体的位置。
# prims 是用来调用的，objects 是用来创建

#以下是要改变的环境
#    # 神说要有刚体圆锥
#     cfg_cone_rigid = sim_utils.ConeCfg(
#         radius=0.15,#半径
#         height=0.5,#高度
#         rigid_props=sim_utils.RigidBodyPropertiesCfg(),#刚体
#         mass_props=sim_utils.MassPropertiesCfg(mass=1.0),#质量
#         collision_props=sim_utils.CollisionPropertiesCfg(),#碰撞箱
#         visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 1.0, 0.0)),#颜色
#     )
#     cfg_cone_rigid.func(
#         "/World/Objects/ConeRigid", cfg_cone_rigid, translation=(-0.2, 0.0, 2.0), orientation=(0.5, 0.0, 0.5, 0.0)#坐标和朝向
#     )
#     # # 物体随机化
#     # # 随机重置圆锥的位置
#     cone_position = random_position()
#     # cone_orient = random_quaternion()
#     # # 找到物块（圆锥和长方体）的模拟器全局路径
#     # cone_prim_path = RigidPrimView(prim_paths_expr="/World/Objects/ConeRigid")

#     # cone_prim_path.set_world_poses(positions=cone_position)
#     print(f"[INFO]: Resetting cone position to {cone_position}...")

#     # 神说要有可变形长方体
#     cfg_cuboid_deformable = sim_utils.MeshCuboidCfg(
#         size=(args_cli.size, 0.5, 0.2),
#         deformable_props=sim_utils.DeformableBodyPropertiesCfg(),
#         visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 0.0, 1.0)),#颜色设置为蓝色
#         physics_material=sim_utils.DeformableBodyMaterialCfg(),
#     )
#     cfg_cuboid_deformable.func("/World/Objects/CuboidDeformable", cfg_cuboid_deformable, translation=(0.15, 0.0, 2.0))

# @configclass
# class CartpoleSceneCfg(InteractiveSceneCfg):
#     """Configuration for a cart-pole scene."""

#     # ground plane
#     ground = AssetBaseCfg(
#         prim_path="/World/ground",
#         spawn=sim_utils.GroundPlaneCfg(size=(100.0, 100.0)),
#     )

#     # cartpole
#     robot: ArticulationCfg = CARTPOLE_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

#     # lights
#     dome_light = AssetBaseCfg(
#         prim_path="/World/DomeLight",
#         spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=500.0),
#     )
#     distant_light = AssetBaseCfg(
#         prim_path="/World/DistantLight",
#         spawn=sim_utils.DistantLightCfg(color=(0.9, 0.9, 0.9), intensity=2500.0),
#         init_state=AssetBaseCfg.InitialStateCfg(rot=(0.738, 0.477, 0.477, 0.0)),
#     )