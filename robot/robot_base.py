from typing import List, Tuple

from rclpy.action import ActionServer
from pxr import Usd, UsdGeom
import numpy as np
import torch
import carb

from isaacsim.core.utils.numpy import rotations
from isaacsim.core.api.scenes import Scene
from isaacsim.core.prims import RigidPrim
from isaacsim.core.utils.rotations import quat_to_rot_matrix
import isaacsim.core.utils.prims as prims_utils
from isaacsim.core.utils.prims import define_prim, get_prim_at_path
from isaacsim.core.utils.viewports import (
    create_viewport_for_camera,
    set_camera_view,
    set_intrinsics_matrix,
)

from map.map_grid_map import GridMap
from path_planning.path_planning_astar import AStar
from camera.camera_base import CameraBase
from camera.camera_cfg import CameraCfg
from camera.camera_third_cfg import CameraThirdCfg
from robot.robot_cfg import RobotCfg
from robot.robot_trajectory import Trajectory
from ros.node_robot import NodeRobot
from scene.scene_manager import SceneManager
from log.log_manager import LogManager
from utils import to_torch
from gsi2isaacsim.gsi_msgs_helper import (
    PrimTransform,
    SceneModifications,
    RobotFeedback,
    VelTwistPose,
    RobotSkill,
    PlanExecution,
    SkillExecution,
    SkillFeedback,
    Plan,
    SkillInfo,
    Parameter,
    VelTwistPose,
)

logger = LogManager.get_logger(__name__)


def _get_viewport_manager_from_container():
    """
    Get viewport manager from dependency injection container.

    This function provides a fallback mechanism to obtain the viewport manager
    when it's not directly injected into the constructor.

    Returns:
        ViewportManager: The viewport manager instance from the container
    """
    try:
        from containers import get_container

        container = get_container()
        return container.viewport_manager()
    except Exception as e:
        raise Exception("ViewportManager not found in container") from e


class RobotBase:
    """Base class of robot."""

    def __init__(
        self,
        cfg_body: RobotCfg = None,
        cfg_camera: CameraCfg = None,
        cfg_camera_third_person: CameraThirdCfg = None,
        scene: Scene = None,
        map_grid: GridMap = None,
        scene_manager: SceneManager = None,
    ):
        self.cfg_body = cfg_body
        self.cfg_camera = cfg_camera
        self.scene = scene
        self.scene_manager = scene_manager
        # self.node = node
        self.viewport_manager = (
            _get_viewport_manager_from_container()
        )  # 通过依赖注入获取viewport_manager

        # 通用的机器人本体初始化代码
        self.cfg_body.prim_path = (
            cfg_body.prim_path
            + f"/{cfg_body.type}"
            + f"/{cfg_body.type}_{cfg_body.id}"
        )
        self.cfg_body.name = cfg_body.type + f"_{cfg_body.id}"
        self.name = self.cfg_body.name

        prim = get_prim_at_path(self.cfg_body.prim_path)
        if not prim.IsValid():
            prim = define_prim(self.cfg_body.prim_path, "Xform")
            if cfg_body.usd_path:
                prim.GetReferences().AddReference(
                    cfg_body.usd_path
                )  # 加载机器人USD模型
            else:
                carb.log_error("unable to add robot usd, usd_path not provided")
        elif prim.IsA(UsdGeom.Xformable):
            # Convert the prim to an Xformable object
            xformable = UsdGeom.Xformable(prim)

            # Get the local-to-world transformation matrix
            # CORRECTED METHOD NAME: ComputeLocalToWorldTransform
            # You need to specify a time code.
            timecode = Usd.TimeCode.Default()  # Use default time or simulation time
            local_to_world_matrix = xformable.ComputeLocalToWorldTransform(timecode)

            # The local_to_world_matrix is a Gf.Matrix4d
            # Extract the translation (position) from the matrix
            cfg_body.position = list(
                local_to_world_matrix.ExtractTranslation()
            )  # Returns a Gf.Vec3d

            # Extract the rotation from the matrix
            quat = local_to_world_matrix.ExtractRotationQuat()  # Returns a Gf.Quatd
            cfg_body.quat = [quat.real] + list(quat.imaginary)
            cfg_body.euler_degree = None
        else:
            if prim:
                logger.info(
                    f"Prim at {prim.GetPath()} is not Xformable or does not exist."
                )
            else:
                logger.info(f"Prim not found at path {prim.GetPath()}")

        # 机器人的历史轨迹
        self.trajectory: Trajectory = None
        # 机器人的控制器
        self.controllers: dict = {}  # 用于存储多个控制器, 'controller name': function
        self.control_mode: str = (
            ""  # 'joint_efforts', 'joint_velocities', 'joint_positions', 'joint_indices', 'joint_names'
        )
        self.action: np.ndarray = None
        # 机器人的技能
        self.skills: dict = {}  # 用于记录其技能: 'skill name': function
        # 用于地图
        self.map_grid: GridMap = map_grid  # 用于存储一个实例化的gridmap
        # 用于回调函数中
        self.flag_active = False
        self.flag_world_reset: bool = False  # 用来记录下世界是不是被初始化了
        self.flag_action_navigation: bool = False  # 用来记录是不是启动导航了
        # 用于PDDL, 记录当前用的 skill, 之所以用skill, 是为了和action区分, action一般是底层的关节动作
        self.state_skill: str = ""
        self.state_skill_complete: bool = True  # 默认状态, 没有skill要做, 所以是True

        # 布置机器人的相机传感器, 可以有多个相机
        self.cameras: dict = {}
        self.view_angle: float = 2 * np.pi / 3  # 感知视野 弧度
        self.view_radius: float = 2  # 感知半径 米
        if cfg_camera is not None:
            logger.info(f"create camera for {self.cfg_body.name_prefix}")
            self.camera = CameraBase(cfg_body, cfg_camera)
            self.camera.create_camera(camera_path=self.cfg_body.camera_path)

        # 第三视角相机 一个机器人只有一个
        self.cfg_camera_third_person = cfg_camera_third_person
        self.camera_prim_path = None
        self.viewport_name = None  # 存储viewport名称
        self.relative_camera_pos = np.array([0, 0, 0])  # 默认为0向量
        self.transform_camera_pos = np.array([0, 0, 0])

        # 机器人的ros node
        self.node = NodeRobot(robot_name=self.name)
        self.skill_action_server = ActionServer(
            node=self.node,
            action_type=SkillExecution,
            action_name=f"/skill/{self.name}",
            execute_callback=self.execute_skill_callback,
        )

        # 机器人的任务状态
        self.current_task_id = "0"
        self.current_task_name = "n"

        # 以下三个变量用于记录navigate开始时的机器人位置，用于计算feedback里的progress
        self.nav_begin = None
        self.nav_end = None
        self.nav_dist = 1.0

    ########################## Skill Action Server ############################
    def execute_skill_callback(self, goal_handle):
        if self.state_skill_complete is False:
            logger.error(
                f"Robot is busy with skill '{self.state_skill}'. Rejecting new goal."
            )
            goal_handle.abort()
            return SkillExecution.Result(success=False, message="Robot is busy.")

        skill_name = goal_handle.request.skill_request.skill_list[0].skill
        self.state_skill_complete = False
        feedback_msg = SkillExecution.Feedback()

        # 模拟一个需要10个步骤的任务
        for i in range(10):
            # 检查是否收到了取消请求
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.state_skill_complete = True  # 重置状态
                logger.info(f"Skill '{skill_name}' was canceled.")
                return SkillExecution.Result(
                    success=False, message="Skill execution was canceled by client."
                )
            # 模拟工作
            import time

            time.sleep(2)
            # 发送反馈
            feedback_msg.status = (
                f"Executing step {i + 1} of 10 for skill '{skill_name}'."
            )
            goal_handle.publish_feedback(feedback_msg)
            logger.info(f"Feedback: {feedback_msg.status}")

        self.state_skill_complete = True
        goal_handle.succeed()
        result = SkillExecution.Result()
        result.success = True
        result.message = f"Skill '{skill_name}' executed successfully."
        return result

    ########################## obs ############################
    def get_obs(self) -> dict:
        """
        Get observation of robot, including controllers, cameras, and world pose.
        """
        raise NotImplementedError()

    @staticmethod
    def to_torch(
        data,
        dtype: torch.dtype = torch.float32,
        device: str = None,
        requires_grad: bool = False,
    ) -> torch.Tensor:
        if device is None:
            if hasattr(data, "device"):
                device = data.device
            else:
                device = "cpu"

        if isinstance(data, torch.Tensor):
            return data.to(device=device, dtype=dtype)
        return torch.as_tensor(data, dtype=dtype, device=device).requires_grad_(
            requires_grad
        )

    def get_world_poses(self) -> Tuple[torch.Tensor, torch.Tensor]:
        pos_IB, q_IB = self.robot_entity.get_world_poses()
        pos_IB, q_IB = pos_IB[0], q_IB[0]
        pos_IB = to_torch(pos_IB, device=pos_IB.device)
        q_IB = to_torch(q_IB, device=q_IB.device)
        return pos_IB, q_IB

    def create_robot_entity(self):
        """
        [Abstract Method] Initializes the specific robot entity wrapper.
        Subclasses MUST override this method to create either an Articulation,
        a RigidPrim, or another appropriate wrapper and assign it to self.robot_entity.
        """
        raise NotImplementedError

    def initialize(self) -> None:
        if self.cfg_camera is not None:
            self.camera.initialize()

        # 第三视角相机初始化 - 使用ViewportManager
        if self.cfg_camera_third_person and self.cfg_camera_third_person.enabled:
            self._initialize_third_person_camera()

    def move_to(
        self, target_position: np.ndarray, target_orientation: np.ndarray
    ) -> bool:
        """
        Args:
            target_position: 目标位置 , shape (3,)
            target_orientation: 目标朝向, shape (4,)
        Returns:
            bool: 是否到达了目标点附近
        """
        raise NotImplementedError()

    def move_along_path(self, path: list = None, flag_reset: bool = False) -> bool:
        """
        让机器人沿着一个list的路径点运动
        需求: 在while外面 能够记录已经到达的点, 每次到达某个目标点的 10cm附近,就认为到了, 然后准备下一个点

        """
        if flag_reset == True:
            self.path_index = 0
            self.path = path

        if self.path_index < len(self.path):  # 当index==len的时候, 就已经到达目标了
            flag_reach = self.move_to(self.path[self.path_index])
            if flag_reach == True:
                self.path_index += 1
            return False
        else:
            self.flag_action_navigation = False
            self.state_skill_complete = True
            return True

    def _calc_dist(self, pos1: torch.Tensor = None, pos2: torch.Tensor = None):
        pos1 = to_torch(pos1)
        pos2 = to_torch(pos2)
        return torch.linalg.norm(pos1 - pos2)

    def navigate_to(
        self,
        pos_target: np.ndarray = None,
        orientation_target: np.ndarray = None,
        reset_flag: bool = False,
        load_from_file: bool = False,
    ) -> None:
        """
        让机器人导航到某一个位置,
        不需要输入机器人的起始位置, 因为机器人默认都是从当前位置出发的;
        本质是使用A*等算法, 规划一系列的路径
        Args:
            target_pos: 机器人的目标位置
            target_orientaton: 机器人的目标方向
            reset_flag:

        Returns:
        """
        # 小车/人形机器人的导航只能使用2d的
        if pos_target == None:
            raise ValueError("no target position")
        elif pos_target[2] != 0:
            raise ValueError("The z-axis height of the robot must be on the plane")
        pos_robot = self.get_world_poses()[0]

        self.nav_begin = np.array(pos_robot)
        self.nav_end = np.array(pos_target)
        self.nav_dist = self._calc_dist(self.nav_begin, self.nav_end)

        pos_index_target = self.map_grid.compute_index(self.nav_end)
        pos_index_robot = self.map_grid.compute_index(self.nav_begin)
        pos_index_robot[-1] = 0  # todo : 这也是因为机器人限制导致的

        # 用于把机器人对应位置的设置为空的, 不然会找不到路线
        if load_from_file == True:
            grid_map = np.load("./value_map.npy")
            pos_map = np.load("./pos_map.npy")
        else:
            grid_map = self.map_grid.value_map
            np.save("./value_map.npy", grid_map)
            pos_map = self.map_grid.pos_map
            np.save("./pos_map.npy", pos_map)

        grid_map[pos_index_robot] = self.map_grid.empty_cell

        planner = AStar(
            grid_map,
            obs_value=1.0,
            free_value=0.0,
            directions="eight",
            penalty_factor=20,
        )
        path = planner.find_path(
            tuple(pos_index_robot), tuple(pos_index_target), render=True
        )

        real_path = np.zeros_like(path, dtype=np.float32)
        for i in range(path.shape[0]):  # 把index变成连续实际世界的坐标
            real_path[i] = self.map_grid.pos_map[tuple(path[i])]
            real_path[i][-1] = 0
        logger.info(real_path)

        self.move_along_path(real_path, flag_reset=True)

        # 标记一下, 开始运动
        self.flag_action_navigation = True

        # 标记当前的动作
        self.state_skill = "navigate_to"
        self.state_skill_complete = False

        return

    def on_physics_step(self, step_size) -> None:
        """
        Args:
            step_size:  dt 时间间隔
        """  # --- 新增：在每一步物理更新后，更新相机位置 ---
        # 更新相机的视野
        self._update_camera_view()
        return

    def post_reset(self) -> None:
        """Set up things that happen after the world resets."""
        for sensor in self.cameras.values():
            sensor.post_reset()
        return

    def put_down(self, robot_hand_prim_path: str, object_prim_path: str) -> dict:
        """
        通过删除关节并恢复物理属性来“放下”一个被抓取的物体。
        """

        hand_prim = RigidPrim(prim_paths_expr=robot_hand_prim_path)
        object_prim = RigidPrim(prim_paths_expr=object_prim_path)

        # 定位并删除用于抓取的固定关节

        joint_path = f"/World/grasp_joint_{object_prim.name}"
        stage = self.scene_manager._stage
        joint_prim = stage.GetPrimAtPath(joint_path)

        from pxr import UsdPhysics

        if joint_prim.IsValid():
            joint = UsdPhysics.Joint(joint_prim)
            joint.GetJointEnabledAttr().Set(False)  # <-- 关键的状态切换！
            # 碰撞属性
            self.scene_manager.set_collision_enabled(
                object_prim_path, collision_enabled=True
            )
            # 模拟惯性
            self._publish_feedback(
                params=[
                    Parameter(key="status", value="normal"),
                    Parameter(key="reason", value=""),
                ],
                progress=50,
            )
            hand_lin_vel = hand_prim.get_linear_velocities()
            hand_ang_vel = hand_prim.get_angular_velocities()
            object_prim.set_linear_velocities(hand_lin_vel)
            object_prim.set_angular_velocities(hand_ang_vel)
        else:
            self._publish_feedback(
                params=[
                    Parameter(key="status", value="failed"),
                    Parameter(key="reason", value="Joint not found"),
                ],
                progress=0,
            )
            logger.info(f"WARNING: Joint '{joint_path}' not found, cannot disable.")

        logger.info(f"INFO: Object '{object_prim.name}' dropped and physics restored.")
        self._publish_feedback(
            params=[
                Parameter(key="status", value="normal"),
                Parameter(key="reason", value=""),
            ],
            progress=100,
        )
        return {"status": "success", "message": "Object dropped successfully."}

    def pickup_object_if_close_unified(
        self,
        robot_hand_prim_path: str,
        object_prim_path: str,
        distance_threshold: float = 2.0,
    ) -> dict:
        """
        检查机器人手部与物体的距离，如果小于阈值，则执行抓取。
        """
        hand_prim = RigidPrim(prim_paths_expr=robot_hand_prim_path)
        object_prim = RigidPrim(prim_paths_expr=object_prim_path)

        hand_pos, _ = hand_prim.get_world_poses()
        object_pos, _ = object_prim.get_world_poses()

        distance = np.linalg.norm(hand_pos - object_pos)

        if distance <= distance_threshold:

            self._publish_feedback(
                params=[
                    Parameter(key="status", value="normal"),
                    Parameter(key="reason", value=""),
                ],
                progress=30,
            )
            # 停止物体当前的任何运动，清除惯性
            object_prim.set_linear_velocities(np.zeros(3))
            object_prim.set_angular_velocities(np.zeros(3))

            # 将物体传送到精确的抓取位置
            object_prim.set_world_poses(positions=hand_pos)

            # 禁用碰撞属性
            self.scene_manager.set_collision_enabled(
                prim_path=object_prim_path, collision_enabled=False
            )

            # 创建物理连接 (关节)
            joint_path = f"/World/grasp_joint_{object_prim.name}"
            stage = self.scene_manager._stage  # 假设可以从SceneManager获取stage
            joint_prim = stage.GetPrimAtPath(joint_path)

            self._publish_feedback(
                params=[
                    Parameter(key="status", value="normal"),
                    Parameter(key="reason", value=""),
                ],
                progress=60,
            )

            # 如果关节不存在，就创建一个。这通常只在第一次抓取时发生。
            if not joint_prim.IsValid():
                logger.info(
                    f"INFO: Joint '{joint_path}' not found, creating it for the first time."
                )
                self.scene_manager.create_joint(
                    joint_path=joint_path,
                    joint_type="fixed",
                    body0=robot_hand_prim_path,
                    body1=object_prim_path,
                    local_pos0=[0, 0, 1],
                    local_pos1=[0, 0, 0],
                    axis=[0, 0, 1],
                )
                joint_prim = stage.GetPrimAtPath(joint_path)
            from pxr import UsdPhysics

            joint = UsdPhysics.Joint(joint_prim)
            joint.GetJointEnabledAttr().Set(True)  # <-- 关键的状态切换！

            self._publish_feedback(
                params=[
                    Parameter(key="status", value="normal"),
                    Parameter(key="reason", value=""),
                ],
                progress=100,
            )

            return {"status": "success"}
        else:
            self._publish_feedback(
                params=[
                    Parameter(key="status", value="failed"),
                    Parameter(key="reason", value="Object is too far to pick up"),
                ],
                progress=0,
            )
            return {
                "status": "skipped",
                "message": f"Object is too far to pick up ({distance:.2f}m > {distance_threshold}m).",
            }

    def take_photo(self, file_path: str = None):
        if self.camera is not None:
            rgb = self.camera.get_rgb()
            if rgb != None and file_path is not None:
                self.camera.save_rgb_to_file(rgb_tensor_gpu=rgb, file_path=file_path)
            return rgb
        else:
            return None

    def quaternion_to_yaw(self, quaternion: Tuple[float, float, float, float]) -> float:
        """
        Args:
            quaternion: 四元数, 顺序是
        Returns:
            yaw: 绕着z轴的旋转角度
        """
        from math import atan2

        matrix = quat_to_rot_matrix(quaternion)
        yaw = atan2(matrix[1, 0], matrix[0, 0])
        return yaw

    def step(self, action: np.ndarray) -> Tuple[np.ndarray]:
        """

        Args:
            action: 使用np.ndarray存储的机器人速度

        Returns:
            obs: 机器人的观测量  但是似乎没有必要这样设计?
            info:
        """
        raise NotImplementedError

    def _initialize_third_person_camera(self):
        """初始化第三人称相机并注册到ViewportManager"""
        logger.info(f"Creating third-person camera for robot {self.cfg_body.id}...")

        # 1. 为相机创建唯一的prim路径和视口名称
        self.camera_prim_path = f"/World/Robot_{self.cfg_body.id}_Camera"
        self.viewport_name = f"Viewport_Robot_{self.cfg_body.id}"

        # 如果prim已存在，先删除（可选，用于热重载）
        if prims_utils.is_prim_path_valid(self.camera_prim_path):
            prims_utils.delete_prim(self.camera_prim_path)

        # 2. 创建相机Prim
        from isaacsim.sensors.camera import Camera

        camera = Camera(prim_path=self.camera_prim_path)
        camera.set_focal_length(2)

        # 3. 创建视口
        viewport_obj = create_viewport_for_camera(
            viewport_name=self.viewport_name,
            camera_prim_path=self.camera_prim_path,
            width=self.cfg_camera_third_person.viewport_size[0],
            height=self.cfg_camera_third_person.viewport_size[1],
            position_x=self.cfg_camera_third_person.viewport_position[0],
            position_y=self.cfg_camera_third_person.viewport_position[1],
        )

        # 4. 注册viewport到ViewportManager
        if viewport_obj and self.viewport_manager:
            success = self.viewport_manager.register_viewport(
                self.viewport_name, viewport_obj
            )
            if success:
                self.viewport_manager.map_camera(
                    self.viewport_name, self.camera_prim_path
                )
                logger.info(
                    f"Robot {self.cfg_body.id} viewport registered to ViewportManager"
                )
            else:
                raise RuntimeError(
                    f"Failed to register viewport for robot {self.cfg_body.id}"
                )

        # 5. 将相对位置转换为numpy数组以便后续计算
        self.relative_camera_pos = np.array(
            self.cfg_camera_third_person.relative_position
        )
        self.transform_camera_pos = np.array(
            self.cfg_camera_third_person.transform_position
        )

    def _update_camera_view(self):
        """更新第三人称相机的位置和朝向"""
        if (
            self.cfg_camera_third_person
            and self.cfg_camera_third_person.enabled
            and self.camera_prim_path
        ):
            # 1. 获取机器人的当前位置
            robot_position, _ = self.get_world_poses()  # torch.tensor

            # 2. 计算相机的位置 (eye) 和目标位置 (target)
            camera_eye_position = (
                robot_position + self.relative_camera_pos + self.transform_camera_pos
            )
            camera_target_position = robot_position

            # 3. 设置相机视图
            set_camera_view(
                eye=camera_eye_position,
                target=camera_target_position,
                camera_prim_path=self.camera_prim_path,
            )

    def get_viewport_info(self):
        """
        获取robot的viewport信息，供ViewportManager批量操作使用

        Returns:
            dict: 包含viewport_name, camera_path等信息的字典
        """
        return {
            "viewport_name": self.viewport_name,
            "camera_path": self.camera_prim_path,
            "robot_id": self.cfg_body.id if self.cfg_body else None,
            "enabled": (
                self.cfg_camera_third_person.enabled
                if self.cfg_camera_third_person
                else False
            ),
        }

    def _params_from_pose(self, pos: np.ndarray, quat: np.ndarray) -> list[Parameter]:
        # TODO:对齐这个param的类型
        p = np.asarray(pos, dtype=float)
        q = np.asarray(quat, dtype=float)
        if p.ndim >= 2:
            p = p[0]
        if q.ndim >= 2:
            q = q[0]

        # 现在期望 p.shape == (3,), q.shape == (4,)
        if p.size < 3 or q.size < 4:
            raise ValueError(f"Invalid pose shapes: pos={p.shape}, quat={q.shape}")

        px, py, pz = (float(v) for v in p)
        # 这里假定四元数顺序为 [x, y, z, w]
        qx, qy, qz, qw = (float(v) for v in q)

        base_return = [
            Parameter(key="pos_x", value=str(px)),
            Parameter(key="pos_y", value=str(py)),
            Parameter(key="pos_z", value=str(pz)),
            Parameter(key="quat_x", value=str(qx)),
            Parameter(key="quat_y", value=str(qy)),
            Parameter(key="quat_z", value=str(qz)),
            Parameter(key="quat_w", value=str(qw)),
        ]

        print(base_return)

        normal_return = [Parameter(key="status", value="normal"), *base_return]
        abnormal_return = [Parameter(key="status", value="abnormal"), *base_return]

        if self.previous_pos:
            if (
                np.sqrt(
                    (self.previous_pos[0] - px) ** 2
                    + (self.previous_pos[1] - py) ** 2
                    + (self.previous_pos[2] - pz) ** 2
                )
                < self.movement_threshold
            ):
                self.previous_pos = [px, py, pz]
                return abnormal_return
            else:
                self.previous_pos = [px, py, pz]
                return normal_return

        else:
            self.previous_pos = [px, py, pz]
            return normal_return

    def _publish_feedback(self, params, progress: int = 1, object_id: str = ""):

        pos, quat = self.robot_entity.get_world_poses()
        params = self._params_from_pose(pos, quat)

        skill = SkillInfo(
            skill=self.current_task_name,
            params=params,
            object_id=object_id,
            task_id=self.current_task_id,
            progress=int(
                progress
            ),  # progress 是当前技能的执行进度，progress是从0到100的一个 int32
        )

        msg = RobotFeedback(
            robot_id=f"{self.cfg_body.name_prefix}_{self.cfg_body.id}",
            skill_feedback=skill,
        )

        # self.node.publish_feedback(self.cfg_body.name_prefix, self.cfg_body.id, msg)

    def _publish_status_pose(self):

        import numpy as np

        positions, orientations = self.robot_entity.get_world_poses()
        pos = positions[0]
        orn = orientations[0]

        # 这些 API 返回 (N, 3) 的数组/张量；这里只取第 0 个
        lin_v = self.robot_entity.get_linear_velocities(indices=[0], clone=True)
        ang_v = self.robot_entity.get_angular_velocities(indices=[0], clone=True)

        # 统一成 numpy，做个健壮性兜底
        lin_v0 = np.asarray(lin_v)[0] if np.size(lin_v) else np.zeros(3, dtype=float)
        ang_v0 = np.asarray(ang_v)[0] if np.size(ang_v) else np.zeros(3, dtype=float)

        msg = VelTwistPose()

        # vel 字段：保持与你原逻辑一致，全部置 0
        msg.vel.x, msg.vel.y, msg.vel.z = [0.0, 0.0, 0.0]

        # twist：使用仿真里的实时速度
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = (
            float(v) for v in lin_v0
        )
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = (
            float(v) for v in ang_v0
        )
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (
            float(v) for v in pos
        )

        # 兼容 ndarray（常见返回）与带属性的四元数对象两种情况
        if hasattr(orn, "x"):
            quat_xyzw = orn.x, orn.y, orn.z, orn.w
        else:
            # 约定顺序为 [x, y, z, w]；若你的资源是 wxyz，请按需调整
            quat_xyzw = float(orn[0]), float(orn[1]), float(orn[2]), float(orn[3])

        (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ) = (float(v) for v in quat_xyzw)
        # self.node.publish_motion(
        #     robot_class=self.cfg_body.name_prefix,
        #     robot_id=self.cfg_body.id,
        #     msg=msg
        # )


if __name__ == "__main__":
    pass
