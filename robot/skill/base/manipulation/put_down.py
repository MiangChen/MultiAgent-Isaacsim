from log.log_manager import LogManager
from physics_engine.isaacsim_utils import RigidPrim
from physics_engine.pxr_utils import UsdPhysics

from gsi_msgs.gsi_msgs_helper import Parameter

logger = LogManager.get_logger(__name__)


def put_down_skill(**kwargs):
    robot = kwargs.get("robot")
    
    # 初始化状态机（只在第一次调用时执行）
    if robot.skill_state is None:
        _init_put_down(robot, kwargs)
    
    # 状态机：每个physics step执行一次
    if robot.skill_state == "INITIALIZING":
        return _handle_initializing(robot)
    elif robot.skill_state == "EXECUTING":
        return _handle_executing(robot)
    elif robot.skill_state == "COMPLETED":
        return _handle_completed(robot)
    elif robot.skill_state == "FAILED":
        return _handle_failed(robot)


def _init_put_down(robot, kwargs):
    """初始化放下技能"""
    robot._robot_hand_prim_path = kwargs.get("robot_hand_prim_path")
    robot._object_prim_path = kwargs.get("object_prim_path")
    robot.skill_state = "INITIALIZING"


def _handle_initializing(robot):
    """初始化阶段：检查必要的组件和路径"""
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
        
        # 构建关节路径
        robot._joint_path = f"/World/grasp_joint_{robot._object_prim.name}"
        
        # 检查场景管理器
        if not hasattr(robot, 'scene_manager') or robot.scene_manager is None:
            robot.skill_state = "FAILED"
            robot.skill_error = "Scene manager is not available."
            return robot.form_feedback("failed", robot.skill_error)
        
        # 初始化完成，进入执行阶段
        robot.skill_state = "EXECUTING"
        return robot.form_feedback("processing", "Initializing put down...", 20)
        
    except Exception as e:
        robot.skill_state = "FAILED"
        robot.skill_error = f"Initialization failed: {str(e)}"
        return robot.form_feedback("failed", robot.skill_error)


def _handle_executing(robot):
    """执行放下操作"""
    try:
        # 获取舞台和关节
        stage = robot.scene_manager.stage
        joint_prim = stage.GetPrimAtPath(robot._joint_path)
        
        if joint_prim.IsValid():
            # 禁用抓取关节
            joint = UsdPhysics.Joint(joint_prim)
            joint.GetJointEnabledAttr().Set(False)
            
            # 启用物体碰撞
            robot.scene_manager.set_collision_enabled(
                robot._object_prim_path, collision_enabled=True
            )
            
            # 模拟惯性传递
            hand_lin_vel = robot._hand_prim.get_linear_velocities()
            hand_ang_vel = robot._hand_prim.get_angular_velocities()
            robot._object_prim.set_linear_velocities(hand_lin_vel)
            robot._object_prim.set_angular_velocities(hand_ang_vel)
            
            logger.info(f"INFO: Object '{robot._object_prim.name}' dropped and physics restored.")
            robot.skill_state = "COMPLETED"
            return robot.form_feedback("processing", "Putting down object...", 90)
            
        else:
            logger.info(f"WARNING: Joint '{robot._joint_path}' not found, cannot disable.")
            robot.skill_state = "FAILED"
            robot.skill_error = "Grasp joint not found"
            return robot.form_feedback("failed", robot.skill_error)
            
    except Exception as e:
        robot.skill_state = "FAILED"
        robot.skill_error = f"Put down execution failed: {str(e)}"
        return robot.form_feedback("failed", robot.skill_error)


def _handle_completed(robot):
    """处理完成状态"""
    _cleanup_put_down(robot)
    return robot.form_feedback("finished", "Object put down successfully!", 100)


def _handle_failed(robot):
    """处理失败状态"""
    error_msg = getattr(robot, 'skill_error', "Unknown error")
    _cleanup_put_down(robot)
    return robot.form_feedback("failed", error_msg)


def _cleanup_put_down(robot):
    """清理放下技能的状态"""
    attrs_to_remove = ['_robot_hand_prim_path', '_object_prim_path', '_hand_prim', 
                       '_object_prim', '_joint_path']
    for attr in attrs_to_remove:
        if hasattr(robot, attr):
            delattr(robot, attr)
    
    robot.skill_state = None
    robot.skill_error = None
