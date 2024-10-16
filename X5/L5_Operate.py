# # 使用该脚本
# /home/ultron/IsaacLab/isaaclab.sh -p /home/ultron/Desktop/TEST/L5_Operate.py --num_envs 32

import argparse

from omni.isaac.lab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="ARX-ZQM")
parser.add_argument("--num_envs", type=int, default=1, help="希望生成的环境数量")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import ArticulationCfg, AssetBaseCfg
from omni.isaac.lab.scene import InteractiveScene, InteractiveSceneCfg
from omni.isaac.lab.sim import SimulationContext
from omni.isaac.lab.utils import configclass
from omni.isaac.lab_assets import CARTPOLE_CFG  # isort:skip


@configclass
class ARX_X5SceneCfg(InteractiveSceneCfg):
    """设置场景"""

    # 基础地面
    ground = AssetBaseCfg(prim_path="/World/singleGround", spawn=sim_utils.GroundPlaneCfg())

    # 场景光线 有一个就行
    dome_light = AssetBaseCfg(
        prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    )

    GreenCone = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Cones",
        spawn=sim_utils.ConeCfg(
            radius=0.025,       # 半径
            height=0.1,         # 高度
            visible=True,       # 是否可见？
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),         # 刚体
            mass_props=sim_utils.MassPropertiesCfg(mass=1.0),       # 质量
            collision_props=sim_utils.CollisionPropertiesCfg(),     # 碰撞箱
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 1.0, 0.0)),  # 颜色
        )
    )
    # 红色的胶囊体
    RedCapsule = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Capsule",
        spawn=sim_utils.CapsuleCfg(
            radius=0.025,        # 半径
            height=0.1,          # 高度
            axis='Z',            # 胶囊体沿着Z轴放置
            visible=True,        # 是否可见？
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),         # 刚体
            mass_props=sim_utils.MassPropertiesCfg(mass=1.0),       # 质量
            collision_props=sim_utils.CollisionPropertiesCfg(),     # 碰撞箱
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0)),  # 颜色
        )
    )
    # 蓝色的长方体
    BlueCuboid = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Cuboid",
        spawn=sim_utils.CuboidCfg(
            size=[0.1, 0.1, 0.1],       # 长方体的大小是？长宽高
            visible=True,               # 是否可见？
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),         # 刚体
            mass_props=sim_utils.MassPropertiesCfg(mass=1.0),       # 质量
            collision_props=sim_utils.CollisionPropertiesCfg(),     # 碰撞箱
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 0.0, 1.0)),  # 颜色
        )
    )
    # 黑色的圆柱
    DarkCylinder = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Cylinder",
        spawn=sim_utils.CylinderCfg(
            radius=0.025,               # 半径
            height=0.1,                 # 高度
            axis='Z',                   # 圆柱体沿着Z轴放置
            visible=True,               # 是否可见？
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),         # 刚体
            mass_props=sim_utils.MassPropertiesCfg(mass=1.0),       # 质量
            collision_props=sim_utils.CollisionPropertiesCfg(),     # 碰撞箱
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 0.0, 0.0)),  # 颜色
        )
    )
    # 白色的球
    WhiteSphere = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Sphere",
        spawn=sim_utils.SphereCfg(
            radius=0.03,                                            # 半径
            visible=True,                                           # 是否可见？
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),         # 刚体
            mass_props=sim_utils.MassPropertiesCfg(mass=1.0),       # 质量
            collision_props=sim_utils.CollisionPropertiesCfg(),     # 碰撞箱
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.01, 0.01, 0.01)),  # 颜色
        )
    )
    # # X5_ARM
    # X5_ARM = ArticulationCfg(
    #     prim_path="{ENV_REGEX_NS}/X5", spawn=sim_utils.UsdFileCfg(usd_path="/home/ultron/ARX_RL/X5/x5.usd"))
    # cartpole: ArticulationCfg = CARTPOLE_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")


def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    """Runs the simulation loop."""
    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    count = 0
    # Simulation loop
    while simulation_app.is_running():
        # Reset
        if count % 500 == 0:
            # reset counter
            count = 0
            # clear internal buffers
            scene.reset()
            print("[INFO]: Resetting robot state...")
        # -- write data to sim
        scene.write_data_to_sim()
        # Perform step
        sim.step()
        # Increment counter
        count += 1
        # Update buffers
        scene.update(sim_dt)


def main():
    """Main function."""
    # Load kit helper
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = SimulationContext(sim_cfg)
    # Set main camera
    sim.set_camera_view([0.25, 0.0, 2.0], [0.0, 0.0, 1])
    # Design scene
    scene_cfg = ARX_X5SceneCfg(num_envs=args_cli.num_envs, env_spacing=5.0)
    scene = InteractiveScene(scene_cfg)
    
    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")

    # Run the simulator
    run_simulator(sim, scene)


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()

