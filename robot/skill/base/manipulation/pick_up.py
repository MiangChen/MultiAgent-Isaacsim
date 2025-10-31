import json

import numpy as np
import torch

from log.log_manager import LogManager
from physics_engine.isaacsim_utils import RigidPrim
from physics_engine.pxr_utils import UsdPhysics, Gf

from gsi_msgs.gsi_msgs_helper import Parameter

logger = LogManager.get_logger(__name__)


def pick_up_skill(**kwargs):
    robot = kwargs.get("robot")
    
    # 初始化状态机（只在第一次调用时执行）
    if robot.skill_state is None:
        _init_pick_up(robot, kwargs)
    
    # 状态机：每个physics step执行一次
    if robot.skill_state == "INITIALIZING":
        return _handle_initializing(robot)
    elif robot.skill_state == "EXECUTING":
        return _handle_executing(robot)
    elif robot.skill_state == "COMPLETED":
        return _handle_completed(robot)
    elif robot.skill_state == "FAILED":
        return _handle_failed(robot)


def _init_pick_up(robot, kwargs):
    """初始化拾取技能"""
    robot._robot_hand_prim_path = kwargs.get("robot_hand_prim_path")
    robot._object_prim_path = kwargs.get("object_prim_path")
    robot._distance_threshold = kwargs.get("distance_threshold", 2.0)
    robot._axis = kwargs.get("axis", [0, 0, 1])
    robot._local_pos_hand = kwargs.get("local_pos_hand", [0, 0, 1])
    robot._local_pos_object = kwargs.get("local_pos_object", [0, 0, 0])
    
    # 处理字符串参数
    if type(robot._axis) is str:
        robot._axis = json.loads(robot._axis)
    if type(robot._local_pos_hand) is str:
        robot._local_pos_hand = json.loads(robot._local_pos_hand)
    if type(robot._local_pos_object) is str:
        robot._local_pos_object = json.loads(robot._local_pos_object)
    
    robot.skill_state = "INITIALIZING"


def _handle_initializing(robot):
    """初始化阶段：检查参数和距离"""
    try:
        # 检查必要参数
        if not robot._robot_hand_prim_path:
            robot.skill_state = "FAILED"
            robot.skill_error = "Robot hand prim path is not provided."
            return robot.form_feedback("failed", robot.skill_error)
        
        if not robot._object_prim_path:
            robot.skill_state = "FAILED"
            robot.skill_error = "Object prim path is not provided."
            return robot.form_feedback("failed", robot.skill_error)
        
        # 初始化刚体对象
        robot._hand_prim = RigidPrim(prim_paths_expr=robot._robot_hand_prim_path)
        robot._object_prim = RigidPrim(prim_paths_expr=robot._object_prim_path)
        
        # 检查场景管理器
        if not hasattr(robot, 'scene_manager') or robot.scene_manager is None:
            robot.skill_state = "FAILED"
            robot.skill_error = "Scene manager is not available."
            return robot.form_feedback("failed", robot.skill_error)
        
        # 计算距离
        hand_pos, _ = robot._hand_prim.get_world_poses()
        object_pos, _ = robot._object_prim.get_world_poses()
        robot._distance = np.linalg.norm(hand_pos - object_pos)
        
        logger.info(f"Distance between hand and object: {robot._distance:.2f}m")
        
        # 检查距离是否在阈值内
        if robot._distance <= robot._distance_threshold:
            robot.skill_state = "EXECUTING"
            return robot.form_feedback("processing", "Distance check passed, starting pick up...", 20)
        else:
            robot.skill_state = "FAILED"
            robot.skill_error = f"Object is too far to pick up ({robot._distance:.2f}m > {robot._distance_threshold}m)."
            return robot.form_feedback("failed", robot.skill_error)
            
    except Exception as e:
        robot.skill_state = "FAILED"
        robot.skill_error = f"Initialization failed: {str(e)}"
        return robot.form_feedback("failed", robot.skill_error)


def _handle_executing(robot):
    """执行拾取操作"""
    try:
        # 停止物体当前的任何运动，清除惯性
        robot._object_prim.set_linear_velocities(torch.zeros(3, dtype=torch.float32))
        robot._object_prim.set_angular_velocities(torch.zeros(3, dtype=torch.float32))
        
        # 获取手部位置并将物体传送到精确的抓取位置
        hand_pos, _ = robot._hand_prim.get_world_poses()
        robot._object_prim.set_world_poses(positions=hand_pos)
        
        # 禁用碰撞属性
        robot.scene_manager.set_collision_enabled(
            prim_path=robot._object_prim_path, collision_enabled=False
        )
        
        # 创建物理连接 (关节)
        robot._joint_path = f"/World/grasp_joint_{robot._object_prim.name}"
        joint_prim = robot.scene_manager.stage.GetPrimAtPath(robot._joint_path)
        
        # 如果关节不存在，就创建一个
        if not joint_prim.IsValid():
            logger.info(f"INFO: Joint '{robot._joint_path}' not found, creating it for the first time.")
            robot.scene_manager.create_joint(
                joint_path=robot._joint_path,
                joint_type="fixed",
                body0=robot._robot_hand_prim_path,
                body1=robot._object_prim_path,
                local_pos0=robot._local_pos_hand,
                local_pos1=robot._local_pos_object,
                axis=robot._axis,
            )
            joint_prim = robot.scene_manager.stage.GetPrimAtPath(robot._joint_path)
        
        # 配置关节
        joint = UsdPhysics.Joint(joint_prim)
        joint.GetLocalPos0Attr().Set(Gf.Vec3f(robot._local_pos_hand))
        joint.GetLocalPos1Attr().Set(Gf.Vec3f(robot._local_pos_object))
        joint.GetJointEnabledAttr().Set(True)  # 关键的状态切换！
        
        logger.info(f"INFO: Object '{robot._object_prim.name}' picked up successfully.")
        robot.skill_state = "COMPLETED"
        return robot.form_feedback("processing", "Picking up object...", 90)
        
    except Exception as e:
        robot.skill_state = "FAILED"
        robot.skill_error = f"Pick up execution failed: {str(e)}"
        return robot.form_feedback("failed", robot.skill_error)


def _handle_completed(robot):
    """处理完成状态"""
    _cleanup_pick_up(robot)
    return robot.form_feedback("finished", "Object picked successfully!", 100)


def _handle_failed(robot):
    """处理失败状态"""
    error_msg = getattr(robot, 'skill_error', "Unknown error")
    _cleanup_pick_up(robot)
    return robot.form_feedback("failed", error_msg)


def _cleanup_pick_up(robot):
    """清理拾取技能的状态"""
    attrs_to_remove = ['_robot_hand_prim_path', '_object_prim_path', '_distance_threshold',
                       '_axis', '_local_pos_hand', '_local_pos_object', '_hand_prim', 
                       '_object_prim', '_distance', '_joint_path']
    for attr in attrs_to_remove:
        if hasattr(robot, attr):
            delattr(robot, attr)
    
    robot.skill_state = None
    robot.skill_error = None
