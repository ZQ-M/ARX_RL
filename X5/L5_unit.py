# # 使用该脚本
# /home/ultron/IsaacLab/isaaclab.sh -p /home/ultron/ARX_RL/X5/L5_unit.py --num_envs 25
""" 设置启动脚本参数 """
import argparse

from omni.isaac.lab.app import AppLauncher

# add argparse arguments
parser = argparse.
(description="Tutorial on creating a cartpole base environment.")
parser.add_argument("--num_envs", type=int, default=16, help="Number of environments to spawn.")

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""导入基础场景单元并进行控制解算"""
import torch
from omni.isaac.lab.envs import ManagerBasedEnv
from X5_env_cfg import X5EnvCfg


def main():
    """Main function."""
    # parse the arguments
    env_cfg = X5EnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    # setup base environment
    env = ManagerBasedEnv(cfg=env_cfg)

    # simulate physics
    count = 0
    while simulation_app.is_running():
        with torch.inference_mode():
            # reset
            if count % 500 == 0:
                count = 0
                env.reset()
                print("-" * 80)
                print("[INFO]: Resetting environment...")
            # 设置每个关节的力矩
            joint_efforts = torch.tensor([[0, 0, 0, 0, 0, 0,]])     # 位置或力量单独控制
            # joint_efforts = torch.tensor([[2, 0, 0, 0, 0, 0,        # 力量和位置混合控制
            #                                0, 0, 0, 0, 0, 0]])
            # 赋值到环境中
            obs = env.step(joint_efforts)
            # # print current orientation of pole
            # print("[Env 0]: Pole joint: ", obs["policy"][0][1].item())
            # update counter
            count += 1

    # close the environment
    env.close()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
