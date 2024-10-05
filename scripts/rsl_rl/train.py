
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
parser.add_argument("--count", type=int , default= 1 , help="传进一个整形参数，暂时用于控制实时仿真的数量(失效)")#自定义的传入参数
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

#Isaac Sim 和其他库中导入不同的 Python 模块
from omni.isaac.lab.sim import SimulationCfg, SimulationContext
from omni.isaac.lab.assets import Articulation #用于操作资产（机器人关节控制） 
import omni.isaac.core.utils.prims as prim_utils
import omni.isaac.lab.sim as sim_utils#仿真环境
from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR# 导入示例中的一个资产（桌子）
from omni.isaac.lab_assets import CARTPOLE_CFG# 导入示例中的一个资产（倒立摆）
from omni.isaac.core.prims import RigidPrimView #Sim接口修改LAB创建的物体位置
import arx.isaac.interfaces

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

#设计场景
def design_scene():
    ARX.on_startup()
    # 神说要有地面
    origins = [[0.0, 0.0, 0.0], [-1.0, 0.0, 0.0]]#origins存储两个地面的坐标
    prim_utils.create_prim("/World/Origin1", "Xform", translation=origins[0])#机器人地面1，坐标是origins[0]
    prim_utils.create_prim("/World/Origin2", "Xform", translation=origins[1])#机器人地面2，坐标是origins[1]
    cfg_ground = sim_utils.GroundPlaneCfg()#创造通用世界地面
    cfg_ground.func("/World/defaultGroundPlane", cfg_ground)#放置到环境中
    
    # 神说要有机械臂
    cartpole_cfg = CARTPOLE_CFG.copy()
    cartpole_cfg.prim_path = "/World/Origin.*/Robot"
    cartpole = Articulation(cfg=cartpole_cfg)

    # 神说要有光
    cfg_light_distant = sim_utils.DistantLightCfg(
        intensity=3000.0,
        color=(0.75, 0.75, 0.75),
    )
    cfg_light_distant.func("/World/lightDistant", cfg_light_distant, translation=(1, 0, 10))

    # 神说要有文件夹
    prim_utils.create_prim("/World/Objects", "Xform")

    # 神说要有刚体圆锥
    cfg_cone_rigid = sim_utils.ConeCfg(
        radius=0.15,#半径
        height=0.5,#高度
        rigid_props=sim_utils.RigidBodyPropertiesCfg(),#刚体
        mass_props=sim_utils.MassPropertiesCfg(mass=1.0),#质量
        collision_props=sim_utils.CollisionPropertiesCfg(),#碰撞箱
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 1.0, 0.0)),#颜色
    )
    cfg_cone_rigid.func(
        "/World/Objects/ConeRigid", cfg_cone_rigid, translation=(-0.2, 0.0, 2.0), orientation=(0.5, 0.0, 0.5, 0.0)#坐标和朝向
    )
    # # 物体随机化
    # # 随机重置圆锥的位置
    # cone_position = random_position()
    # cone_orient = random_quaternion()
    # # 找到物块（圆锥和长方体）的模拟器全局路径
    # cone_prim_path = RigidPrimView(prim_paths_expr="/World/Objects/ConeRigid")

    # cone_prim_path.set_world_poses(positions=cone_position)
    print(f"[INFO]: Resetting cone position to {cone_position}...")

    # 神说要有可变形长方体
    cfg_cuboid_deformable = sim_utils.MeshCuboidCfg(
        size=(args_cli.size, 0.5, 0.2),
        deformable_props=sim_utils.DeformableBodyPropertiesCfg(),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 0.0, 1.0)),#颜色设置为蓝色
        physics_material=sim_utils.DeformableBodyMaterialCfg(),
    )
    cfg_cuboid_deformable.func("/World/Objects/CuboidDeformable", cfg_cuboid_deformable, translation=(0.15, 0.0, 2.0))

    # 神说要从外部导入一张桌子
    cfg = sim_utils.UsdFileCfg(usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/SeattleLabTable/table_instanceable.usd")
    cfg.func("/World/Objects/Table", cfg, translation=(0.0, 0.0, 1.05))

    # 神说要将机器人状态返回出去
    scene_entities = {"cartpole": cartpole}#记录机器人的状态
    return scene_entities, origins#返回机器人的状态和场景的初始位置

def run_simulator(sim: sim_utils.SimulationContext, entities: dict[str, Articulation], origins: torch.Tensor):
    """HELLO 这是我的新函数"""
    robot = entities["cartpole"]

    # 定义仿真渲染的步长（下面设置的100HZ，500就是5s）
    sim_dt = sim.get_physics_dt()
    count = 0
    
    # 仿真循环
    while simulation_app.is_running():
        if count % 500 == 0:# 渲染 500 步长后重置
            count = 0

            # 机器人重置
            root_state = robot.data.default_root_state.clone()
            root_state[:, :3] += origins
            robot.write_root_state_to_sim(root_state)
            # 关节随机化
            joint_pos, joint_vel = robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone()
            joint_pos += torch.rand_like(joint_pos) * 0.1
            robot.write_joint_state_to_sim(joint_pos, joint_vel)

            robot.reset()
            print("[INFO]: Resetting robot state...")

        # Apply random action
        # -- generate random joint efforts
        efforts = torch.randn_like(robot.data.joint_pos) * 5.0
        # -- apply action to the robot
        robot.set_joint_effort_target(efforts)
        # -- write data to sim
        robot.write_data_to_sim()
        # Perform step
        sim.step()
        # Increment counter
        count += 1
        # Update buffers
        robot.update(sim_dt)

def main():
    # 设置仿真参数
    sim_cfg = SimulationCfg(dt=0.01) #仿真渲染步长100HZ0.01秒
    sim = SimulationContext(sim_cfg)
    # 设置主摄像头位置和角度
    sim.set_camera_view([2.5, 2.5, 2.5], [0.0, 0.0, 0.0])

    # Design scene
    scene_entities, scene_origins = design_scene()#神的设计场景，返回机器人状态和初始位置
    scene_origins = torch.tensor(scene_origins, device=sim.device)#转换为一个 PyTorch 的 Tensor 对象。
    # 仿真的要求：All the scene designing must happen bhezheefore the simulation starts.
    #开始仿真
    sim.reset()
    print("[INFO]: Setup complete...")
    run_simulator(sim,scene_entities,scene_origins)


if __name__ == "__main__":
    main()
    # 关闭SIM
    simulation_app.close()

    
    
#启动脚本需要的命令
# source ~/IsaacLab/_isaac_sim/setup_conda_env.sh
# /home/ultron/IsaacLab/isaaclab.sh -p /home/ultron/L5_Handle/train.py -h

# 是的，你可以通过 Isaac Sim 的接口修改由 Isaac Lab 创建的物体的位置。
# prims 是用来调用的，objects 是用来创建