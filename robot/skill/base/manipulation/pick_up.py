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
    skill_name = "pick_up_skill"
    
    current_state = robot.skill_states.get(skill_name)

    # 初始化状态机（只在第一次调用时执行）
    if current_state in [None, "INITIALIZING"]:
        _init_pick_up(robot, skill_name, kwargs)

    current_state = robot.skill_states.get(skill_name)
    if current_state == "EXECUTING":
        return _handle_executing(robot, skill_name)
    elif current_state == "COMPLETED":
        return _handle_completed(robot, skill_name)
    elif current_state == "FAILED":
        return _handle_failed(robot, skill_name)


def _init_pick_up(robot, skill_name, kwargs):
    """初始化拾取技能"""
    robot_hand_prim_path = kwargs.get("robot_hand_prim_path")
    object_prim_path = kwargs.get("object_prim_path")
    distance_threshold = kwargs.get("distance_threshold", 2.0)
    axis = kwargs.get("axis", [0, 0, 1])
    local_pos_hand = kwargs.get("local_pos_hand", [0, 0, 1])
    local_pos_object = kwargs.get("local_pos_object", [0, 0, 0])
    
    # 处理字符串参数
    if type(axis) is str:
        axis = json.loads(axis)
    if type(local_pos_hand) is str:
        local_pos_hand = json.loads(local_pos_hand)
    if type(local_pos_object) is str:
        local_pos_object = json.loads(local_pos_object)
    
    # 存储参数到技能私有数据
    robot.set_skill_data(skill_name, "robot_hand_prim_path", robot_hand_prim_path)
    robot.set_skill_data(skill_name, "object_prim_path", object_prim_path)
    robot.set_skill_data(skill_name, "distance_threshold", distance_threshold)
    robot.set_skill_data(skill_name, "axis", axis)
    robot.set_skill_data(skill_name, "local_pos_hand", local_pos_hand)
    robot.set_skill_data(skill_name, "local_pos_object", local_pos_object)
    
    robot.skill_states[skill_name] = "INITIALIZING"


def _handle_initializing(robot, skill_name):
    """初始化阶段：检查参数和距离"""
    try:
        # 获取参数
        robot_hand_prim_path = robot.get_skill_data(skill_name, "robot_hand_prim_path")
        object_prim_path = robot.get_skill_data(skill_name, "object_prim_path")
        distance_threshold = robot.get_skill_data(skill_name, "distance_threshold")
        
        # 检查必要参数
        if not robot_hand_prim_path:
            robot.skill_states[skill_name] = "FAILED"
            robot.skill_errors[skill_name] = "Robot hand prim path is not provided."
            return robot.form_feedback("failed", robot.skill_errors[skill_name])
        
        if not object_prim_path:
            robot.skill_states[skill_name] = "FAILED"
            robot.skill_errors[skill_name] = "Object prim path is not provided."
            return robot.form_feedback("failed", robot.skill_errors[skill_name])
        
        # 初始化刚体对象
        hand_prim = RigidPrim(prim_paths_expr=robot_hand_prim_path)
        object_prim = RigidPrim(prim_paths_expr=object_prim_path)
        
        # 存储刚体对象
        robot.set_skill_data(skill_name, "hand_prim", hand_prim)
        robot.set_skill_data(skill_name, "object_prim", object_prim)
        
        # 检查场景管理器
        if not hasattr(robot, 'scene_manager') or robot.scene_manager is None:
            robot.skill_states[skill_name] = "FAILED"
            robot.skill_errors[skill_name] = "Scene manager is not available."
            return robot.form_feedback("failed", robot.skill_errors[skill_name])
        
        # 计算距离
        hand_pos, _ = hand_prim.get_world_poses()
        object_pos, _ = object_prim.get_world_poses()
        distance = np.linalg.norm(hand_pos - object_pos)
        
        # 存储距离
        robot.set_skill_data(skill_name, "distance", distance)
        
        logger.info(f"Distance between hand and object: {distance:.2f}m")
        
        # 检查距离是否在阈值内
        if distance <= distance_threshold:
            robot.skill_states[skill_name] = "EXECUTING"
            return robot.form_feedback("processing", "Distance check passed, starting pick up...", 20)
        else:
            robot.skill_states[skill_name] = "FAILED"
            robot.skill_errors[skill_name] = f"Object is too far to pick up ({distance:.2f}m > {distance_threshold}m)."
            return robot.form_feedback("failed", robot.skill_errors[skill_name])
            
    except Exception as e:
        robot.skill_states[skill_name] = "FAILED"
        robot.skill_errors[skill_name] = f"Initialization failed: {str(e)}"
        return robot.form_feedback("failed", robot.skill_errors[skill_name])


def _handle_executing(robot, skill_name):
    """执行拾取操作"""
    try:
        # 获取存储的数据
        hand_prim = robot.get_skill_data(skill_name, "hand_prim")
        object_prim = robot.get_skill_data(skill_name, "object_prim")
        object_prim_path = robot.get_skill_data(skill_name, "object_prim_path")
        robot_hand_prim_path = robot.get_skill_data(skill_name, "robot_hand_prim_path")
        local_pos_hand = robot.get_skill_data(skill_name, "local_pos_hand")
        local_pos_object = robot.get_skill_data(skill_name, "local_pos_object")
        axis = robot.get_skill_data(skill_name, "axis")
        
        # 停止物体当前的任何运动，清除惯性
        object_prim.set_linear_velocities(torch.zeros(3, dtype=torch.float32))
        object_prim.set_angular_velocities(torch.zeros(3, dtype=torch.float32))
        
        # 获取手部位置并将物体传送到精确的抓取位置
        hand_pos, _ = hand_prim.get_world_poses()
        object_prim.set_world_poses(positions=hand_pos)
        
        # 禁用碰撞属性
        robot.scene_manager.set_collision_enabled(
            prim_path=object_prim_path, collision_enabled=False
        )
        
        # 创建物理连接 (关节)
        joint_path = f"/World/grasp_joint_{object_prim.name}"
        robot.set_skill_data(skill_name, "joint_path", joint_path)
        joint_prim = robot.scene_manager.stage.GetPrimAtPath(joint_path)
        
        # 如果关节不存在，就创建一个
        if not joint_prim.IsValid():
            logger.info(f"INFO: Joint '{joint_path}' not found, creating it for the first time.")
            robot.scene_manager.create_joint(
                joint_path=joint_path,
                joint_type="fixed",
                body0=robot_hand_prim_path,
                body1=object_prim_path,
                local_pos0=local_pos_hand,
                local_pos1=local_pos_object,
                axis=axis,
            )
            joint_prim = robot.scene_manager.stage.GetPrimAtPath(joint_path)
        
        # 配置关节
        joint = UsdPhysics.Joint(joint_prim)
        joint.GetLocalPos0Attr().Set(Gf.Vec3f(local_pos_hand))
        joint.GetLocalPos1Attr().Set(Gf.Vec3f(local_pos_object))
        joint.GetJointEnabledAttr().Set(True)  # 关键的状态切换！
        
        logger.info(f"INFO: Object '{object_prim.name}' picked up successfully.")
        robot.skill_states[skill_name] = "COMPLETED"
        return robot.form_feedback("processing", "Picking up object...", 90)
        
    except Exception as e:
        robot.skill_states[skill_name] = "FAILED"
        robot.skill_errors[skill_name] = f"Pick up execution failed: {str(e)}"
        return robot.form_feedback("failed", robot.skill_errors[skill_name])


def _handle_completed(robot, skill_name):
    """处理完成状态"""
    return robot.form_feedback("completed", "Object picked successfully!", 100)


def _handle_failed(robot, skill_name):
    """处理失败状态"""
    error_msg = robot.skill_errors.get(skill_name, "Unknown error")
    return robot.form_feedback("failed", error_msg)
