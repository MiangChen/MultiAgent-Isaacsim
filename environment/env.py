from map.map_grid_map import GridMap
from robot.robot_jetbot import RobotCfgJetbot, RobotJetbot
from robot.robot_h1 import RobotH1, RobotCfgH1
from robot.robot_cf2x import RobotCf2x, RobotCfgCf2x
from robot.robot_swarm_manager import RobotSwarmManager
from files.assets_scripts_linux import PATH_PROJECT, PATH_ISAACSIM_ASSETS

import gymnasium as gym
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils.viewports import create_viewport_for_camera, set_camera_view
import isaacsim.core.utils.prims as prims_utils
from isaacsim.core.utils.prims import create_prim

from ros.ros_swarm import BaseNode, SceneMonitorNode
from rclpy.node import Node
import rclpy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Transform as RosTransform
from .msg import PrimTransform, SceneModifications, Plan, SkillInfo

def create_scene(usd_path: str, prim_path_root: str = "/World"):
    """

    Create a scene from config.(But just input usd file yet.)
    Args:
        usd_path (str): path to scene config file(use to be a .usd file)
        prim_path_root (str): path to root prim
    """
    if (
            usd_path.endswith("usd")
            or usd_path.endswith("usda")
            or usd_path.endswith("usdc")
    ):

        create_prim(
            prim_path_root,
            usd_path=usd_path,
            # scale=self.simulation.scene_scale,
            scale=[1, 1, 1],
            # translation=[0, 0, 0.81696],
            # orientation=[0.610, -0.789, -0.05184, 0.040] # wxyz, xyz还是zyx顺序不确定
        )
    else:
        raise RuntimeError("Env file path needs to end with .usd, .usda or .usdc .")
    return


class Env(gym.Env):

    def __init__(self, simulation_app=None, usd_path: str = None) -> None:

        super().__init__()
        self._render = None
        self._robot_name = None
        self._current_task_name = None
        # self._validate()
        # from isaacsim import SimulationApp
        # self.simulation_app = SimulationApp({"headless": False})  # we can also run as headless.

        self._runner = simulation_app
        self.world = World(physics_dt=1 / 200)
        # self.world.scene.add_default_ground_plane()  # 添加地面
        # self.world.set_gpu_dynamics_enabled(True)
        # import omni.physx
        #
        # physx_interface = omni.physx.acquire_physx_interface()
        #
        # physx_interface.set_gpu_dynamics_enabled(True)
        # is_enabled = physx_interface.get_gpu_dynamics_enabled()
        # print("is_enabled:", is_enabled)
        create_scene(usd_path=usd_path)
        self.cell_size = 0.1
        self.map_grid = GridMap(
            cell_size=self.cell_size
        )  # gridmap需要在robot swarm之前使用

        self.robot_swarm = RobotSwarmManager(self.world.scene, self.map_grid)
        self.robot_swarm.register_robot_class(
            robot_class_name="jetbot",
            robot_class=RobotJetbot,
            robot_class_cfg=RobotCfgJetbot,
        )  # 注册jetbot机器人
        self.robot_swarm.register_robot_class(
            robot_class_name="h1", robot_class=RobotH1, robot_class_cfg=RobotCfgH1
        )  # 注册h1机器人
        self.robot_swarm.register_robot_class(
            robot_class_name="cf2x", robot_class=RobotCf2x, robot_class_cfg=RobotCfgCf2x
        )  # 注册cf2x机器人
        # TODO 更新路径的设置
        self.robot_swarm.load_robot_swarm_cfg(
            f"{PATH_PROJECT}/files/robot_swarm_cfg.yaml"
        )

        # self.robot_swarm.create_robot(robot_class_name='jetbot', id=0, position=(0.0, 0.0, 0.0),
        #                               orientation=(0.0, 0.0, 0.0, 1),
        #                               robot_class_cfg=JetbotCfg)  # 机器人名字和cfg是对应的, 所以直接输入一个cfg就可以了

        self.robot_swarm.activate_robot(
            f"{PATH_PROJECT}/files/robot_swarm_active_flag.yaml"
        )  # 统一在这里加入机器人

        # 添加相机, 为相机添加viewport
        self.camera_prim_path = "/World/camera_test"
        xform_path = self.camera_prim_path + "/xform_camera"
        # prims_utils.delete_prim(xform_path) # 如果已经有了，要先删除
        # prims_utils.delete_prim(camera_prim_path)
        prims_utils.create_prim(
            prim_path=self.camera_prim_path,
            prim_type="Camera",
            position=np.array([5, 0, 50.0]),
        )
        # 创建一个 Xform 原始来控制相机的位置和旋转
        xform_prim = prims_utils.create_prim(prim_path=xform_path, prim_type="Xform")
        create_viewport_for_camera(
            viewport_name="/test_camera",
            camera_prim_path=self.camera_prim_path,
            width=720 / 2,
            height=720 / 2,
            position_x=800,  ## 设置他在IsaacSimAPP中的相对的位置
            position_y=400,
        )

        ## 可以设置相机的观察位置
        set_camera_view(
            eye=np.array([5, 0, 50]),
            target=np.array([5, 0, 0]),
            camera_prim_path=self.camera_prim_path,
        )
        print("init success")

        self.plan_receiver = BaseNode('plan_receiver')
        self.plan_receiver.plan_subscriber = self.create_subscriber(
            Plan,
            'Plan',
            self._plan_callback,
            100
        )
        self.plan_receiver.plan_subscriber
        self.scene_monitor = SceneMonitorNode('scene_monitor')

        return

    def reset(self):
        self.world.reset()
        self.init_robot()  # 似乎要在这里先初始化机器人, 才能在grid map中找到机器人的障碍
        self.map_grid.initialize()
        print("reset env & init grid map success")

        return

    def step(self, action):
        self.world.step()
        return

    def init_robot(self):
        for robot_class in self.robot_swarm.robot_active.keys():
            for robot in self.robot_swarm.robot_active[robot_class]:
                robot.initialize()
                robot.flag_world_reset = True

    def _plan_callback(self, msg: Plan):

        skills: Dict[int, Dict[str, Dict[str, Any]]] = {}

        # 遍历每个时间步
        for step in msg.steps:  # type: TimestepSkills
            t: int = step.timestep
            if t not in skills:
                skills[t] = {}

            # 遍历该时间步里，每个机器人的技能列表
            for robot_skill in step.robots:  # type: RobotSkill
                robot_id: str = robot_skill.robot_id

                # 通常这里 skill_list 长度为 1；如果有多个，你可以按需取第一个或全部
                # 下面示例取第一个
                if not robot_skill.skill_list:
                    continue

                sk: SkillInfo = robot_skill.skill_list[0]
                # 将参数列表转成 dict（如果你确实在 Parameter.msg 里定义了 key/value）
                params_dict = {p.key: p.value for p in sk.params}

                skills[t][robot_id] = {
                    'skill': sk.skill,
                    'params': params_dict,
                    'object_id': sk.object_id if sk.object_id else None,
                    'task_id': sk.task_id
                }

        # 存回成员变量
        self.skills_by_timestep = skills

        self._execute_and_render()

    def _execute_and_render (self):

        from map.map_semantic_map import MapSemantic
        map_semantic = MapSemantic()

        for i in range(500000):
            self.step(action=None)  # execute one physics step and one rendering step
            continue
            # 设置相机的位置
            camera_pose = np.zeros(3)  # 创建一个包含三个0.0的数组
            pos = self.robot_swarm.robot_active['jetbot'][0].get_world_poses()[0]  # x y z 坐标
            pos1 = self.robot_swarm.robot_active['jetbot'][1].get_world_poses()[0]  # x y z 坐标
            pos_cf2x = env.robot_swarm.robot_active['cf2x'][0].get_world_poses()[0]  # x y z 坐标
            camera_pose[:2] = pos_cf2x[:2]  # 将xy坐标赋值给result的前两个元素
            camera_pose[2] = pos_cf2x[-1] + 1
            set_camera_view(
                eye=camera_pose,  # np.array([5+i*0.001, 0, 50]),
                target=pos_cf2x,  # np.array([5+i*0.001, 0, 0]),
                camera_prim_path=env.camera_prim_path,
            )

            # 根据已经规划好的, 进行一个划分,
            # 要首先确定机器人的action complete状态, 肯定要记录上次的action是什么, 然后action 有一个 flag, 用于记录这个action启动后, 有没有完成; flag_action_complete
            # 如果发现每一个机器人都完成了action, 那么就可以进入plan搜索下一个step,
            # 一个flag用于确定是否启动回调函数
            # 一个flag用于确定机器人时候时候完成了某个action
            # 先不混用
            # 检查所有机器人上一个技能是否都执行完毕
            state_skill_complete_all = True

            for robot_class in env.robot_swarm.robot_class:
                for robot in env.robot_swarm.robot_active[robot_class]:
                    state_skill_complete_all &= robot.state_skill_complete

            if state_skill_complete_all:
                # 推进到下一个 timestep
                state_step += 1

                # 拿出这一步骤所有机器人的技能分配（默认空 dict）
                timestep_plan = self.skills_by_timestep.get(state_step, {})

                # 遍历每个机器人要执行的 skill
                for robot_id, skill_info in timestep_plan.items():
                    # --- 解析 robot_id 和索引 ---
                    # 假设 robot_id 格式总是 'robotN'
                    idx = int(robot_id.replace('robot', ''))

                    # --- 解析 SkillInfoDict ---
                    skill = skill_info['skill']  # 比如 'navigate-to'
                    params = skill_info.get('params', {})  # 原 plan 里的 start/goal 都放这里
                    # object_id/task_id 如果需要，也可以取： skill_info['object_id'], skill_info['task_id']

                    # --- 调度到具体方法 ---
                    if skill == 'navigate-to':
                        # 取出目的地
                        goal = params['goal']
                        pos_target = map_semantic.map_semantic[goal]
                        env.robot_swarm.robot_active['jetbot'][idx].navigate_to(pos_target)

                    elif skill == 'pick-up':
                        env.robot_swarm.robot_active['jetbot'][idx].pick_up()

                    elif skill == 'put-down':
                        env.robot_swarm.robot_active['jetbot'][idx].put_down()

            # 推进物理／渲染
            self.step(action=None)
