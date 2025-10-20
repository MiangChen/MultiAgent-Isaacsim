# =============================================================================
# Controller Policy Module - Policy-Based Robot Controller
# =============================================================================
#
# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
#
# This module provides policy-based controller implementation for robots
# using neural network policies for advanced locomotion and control.
#
# =============================================================================

# Standard library imports
import io
from typing import Optional

# Third-party library imports
import numpy as np
import omni
import torch

# Local project imports
from robot.controller.controller_cfg_loader import (
    get_articulation_props,
    get_physics_properties,
    get_robot_joint_properties,
    parse_env_config,
)
from log.log_manager import LogManager
from physics_engine.isaacsim_utils import BaseController

logger = LogManager.get_logger(__name__)


class PolicyController(BaseController):
    """
    策略控制器基类，支持加载预训练策略模型。

    Args:
        name (str): 控制器的名称。
        prim_path (str): 场景中的prim地址。
        root_path (Optional[str], None): 机器人的根节点。
        usd_path (Optional[str], optional):USD文件地址，默认为None。
        position (Optional[np.ndarray], optional): 机器人初始位置，默认为None.
        orientation (Optional[np.ndarray], optional): 机器人初始姿态，默认为None.

    Attributes:
        robot (SingleArticulation): The robot articulation.
    """

    def __init__(
        self,
        name: str,
        prim_path: str,
        root_path: Optional[str] = None,
        usd_path: Optional[str] = None,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
    ) -> None:
        pass
        # prim = get_prim_at_path(prim_path) # 获取prim地址
        #
        # if not prim.IsValid():
        #     prim = define_prim(prim_path, "Xform")
        #     if path_usd:
        #         prim.GetReferences().AddReference(path_usd) # 加载机器人USD模型
        #     else:
        #         carb.log_error("unable to add robot usd, path_usd not provided")
        #
        # # 移动到机器人本体中声明

    # # 初始化机器人关节树
    # if root_path == None:
    #     self.robot = SingleArticulation(prim_path=prim_path, namespace=namespace, position=position, orientation=orientation)
    # else:
    #     self.robot = SingleArticulation(prim_path=root_path, namespace=namespace, position=position, orientation=orientation)

    async def load_policy(self, policy_file_path, policy_env_path) -> None:
        """
        加载策略。

        Args:
            policy_file_path (str): 策略文件地址.
            policy_env_path (str): 环境配置文件地址.
        """
        # 读取策略
        result, _, file_content = await omni.client.read_file_async(policy_file_path)
        if result != omni.client.Result.OK:
            logger.info(
                f"Failed to read policy file from {policy_file_path}. Result: {result}"
            )
            return
        file = io.BytesIO(memoryview(file_content).tobytes())
        self.policy = torch.jit.load(file)
        # 读取环境配置
        self.policy_env_params = parse_env_config(policy_env_path)
        # 获取物理参数
        self._decimation, self._dt, self.render_interval = get_physics_properties(
            self.policy_env_params
        )

    def initialize(
        self,
        robot,
        physics_sim_view: omni.physics.tensors.SimulationView = None,
        effort_modes: str = "force",
        control_mode: str = "position",
        set_gains: bool = True,
        set_limits: bool = True,
        set_articulation_props: bool = True,
    ) -> None:
        """
        初始化机器人并创建控制器.

        Args:
            physics_sim_view (optional): 物理仿真试图.
            effort_modes (str, optional): 关节力模式（force/acceleration）. 默认 "force".
            control_mode (str, optional): 控制模式（position/velocity）. 默认 "position".
            set_gains (bool, optional): 是否设置关节PD参数. 默认 True.
            set_limits (bool, optional): 是否设置关节限制. 默认 True.
            set_articulation_props (bool, optional): 是否设置关节属性. 默认 True.
        """
        # 初始化机器人物理实体
        # self.robot.initialize(physics_sim_view=physics_sim_view)
        robot.initialize(physics_sim_view=physics_sim_view)
        # 设置关节力模式和控制模式
        # self.robot.get_articulation_controller().set_effort_modes(effort_modes)
        # self.robot.get_articulation_controller().switch_control_mode(control_mode)
        # robot.get_articulation_controller().set_effort_modes(effort_modes)
        robot.set_effort_modes(effort_modes)  # 'force'
        # robot.get_articulation_controller().switch_control_mode(control_mode)
        robot.switch_control_mode(control_mode)  # 'position'

        # 从配置文件加载关节属性
        max_effort, max_vel, stiffness, damping, self.default_pos, self.default_vel = (
            get_robot_joint_properties(self.policy_env_params, robot.dof_names)
        )
        max_effort = torch.tensor(max_effort, dtype=torch.float32)
        max_vel = torch.tensor(max_vel, dtype=torch.float32)
        stiffness = torch.tensor(stiffness, dtype=torch.float32)
        damping = torch.tensor(damping, dtype=torch.float32)
        self.default_pos = torch.tensor(self.default_pos, dtype=torch.float32)
        self.default_vel = torch.tensor(self.default_vel, dtype=torch.float32)

        # 应用参数到机器人
        if set_gains:
            # robot._articulation_view.set_gains(stiffness, damping) # 设置PD参数
            robot.set_gains(kps=stiffness, kds=damping)  # 设置PD参数
        if set_limits:
            # robot._articulation_view.set_max_efforts(max_effort) # 力矩限制
            robot.set_max_efforts(max_effort)  # 力矩限制
            # robot._articulation_view.set_max_joint_velocities(max_vel) # 力矩限制
            robot.set_max_joint_velocities(max_vel)  # 力矩限制
        if set_articulation_props:
            self._set_articulation_props(robot)  # 设置高级物理属性

    def _set_articulation_props(self, robot) -> None:
        """
        高级关节属性设置.
        """
        # 从配置文件读取高级属性
        articulation_prop = get_articulation_props(self.policy_env_params)
        # 物理求解器参数
        solver_position_iteration_count = articulation_prop.get(
            "solver_position_iteration_count"
        )  # 位置迭代次数
        solver_velocity_iteration_count = articulation_prop.get(
            "solver_velocity_iteration_count"
        )  # 速度迭代次数
        stabilization_threshold = articulation_prop.get(
            "stabilization_threshold"
        )  # 稳定阈值
        enabled_self_collisions = articulation_prop.get(
            "enabled_self_collisions"
        )  # 自碰撞开关
        sleep_threshold = articulation_prop.get("sleep_threshold")  # 休眠阈值
        # 应用参数到机器人
        if solver_position_iteration_count not in [None, float("inf")]:
            # robot.set_solver_position_iteration_count(solver_position_iteration_count)
            robot.set_solver_position_iteration_counts(
                [solver_position_iteration_count]
            )
        if solver_velocity_iteration_count not in [None, float("inf")]:
            robot.set_solver_velocity_iteration_counts(
                [solver_velocity_iteration_count]
            )
        if stabilization_threshold not in [None, float("inf")]:
            robot.set_stabilization_thresholds([stabilization_threshold])
        if isinstance(enabled_self_collisions, bool):
            robot.set_enabled_self_collisions([enabled_self_collisions])
        if sleep_threshold not in [None, float("inf")]:
            robot.set_sleep_thresholds([sleep_threshold])

    def _compute_action(self, obs: np.ndarray) -> np.ndarray:
        """
        策略推理方法.

         Args:
             obs (np.ndarray): 观测值.

         Returns:
             np.ndarray: 动作.
        """
        with torch.no_grad():
            obs = torch.from_numpy(obs).view(1, -1).float()  # 转换为PyTorch张量
            action = self.policy(obs).detach().view(-1).numpy()  # 策略推理
        return action

    def _compute_observation(self) -> NotImplementedError:
        """
        计算观测空间，暂未使用
        """

        raise NotImplementedError(
            "Compute observation need to be implemented, expects np.ndarray in the structure specified by env yaml"
        )

    def forward(self) -> NotImplementedError:
        """
        前进控制器，暂未使用
        """
        raise NotImplementedError(
            "Forward needs to be implemented to compute and apply robot control from observations"
        )

    def post_reset(self) -> None:
        """
        重置
        """
        self.robot.post_reset()
