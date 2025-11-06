from log.log_manager import LogManager
from physics_engine.isaacsim_utils import RigidPrim
from physics_engine.pxr_utils import UsdPhysics

from gsi_msgs.gsi_msgs_helper import Parameter

logger = LogManager.get_logger(__name__)


def put_down_skill(**kwargs):
    robot = kwargs.get("robot")
    skill_name = "put_down_skill"

    current_state = robot.skill_states.get(skill_name)

    # 初始化状态机（只在第一次调用时执行）
    if current_state in [None, "INITIALIZING"]:
        _init_put_down(robot, skill_name, kwargs)

    current_state = robot.skill_states.get(skill_name)
    if current_state == "INITIALIZING":
        return _handle_initializing(robot, skill_name)
    elif current_state == "EXECUTING":
        return _handle_executing(robot, skill_name)
    elif current_state == "COMPLETED":
        return _handle_completed(robot, skill_name)
    elif current_state == "FAILED":
        return _handle_failed(robot, skill_name)


def _init_put_down(robot, skill_name, kwargs):
    """初始化放下技能"""
    robot_hand_prim_path = kwargs.get("robot_hand_prim_path")
    object_prim_path = kwargs.get("object_prim_path")

    # 存储参数到技能私有数据
    robot.set_skill_data(skill_name, "robot_hand_prim_path", robot_hand_prim_path)
    robot.set_skill_data(skill_name, "object_prim_path", object_prim_path)

    robot.skill_states[skill_name] = "INITIALIZING"


def _handle_initializing(robot, skill_name):
    """初始化阶段：检查必要的组件和路径"""
    try:
        # 获取参数
        robot_hand_prim_path = robot.get_skill_data(skill_name, "robot_hand_prim_path")
        object_prim_path = robot.get_skill_data(skill_name, "object_prim_path")

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

        # 构建关节路径
        joint_path = f"/World/grasp_joint_{object_prim.name}"
        robot.set_skill_data(skill_name, "joint_path", joint_path)

        # 检查场景管理器
        if not hasattr(robot, "scene_manager") or robot.scene_manager is None:
            robot.skill_states[skill_name] = "FAILED"
            robot.skill_errors[skill_name] = "Scene manager is not available."
            return robot.form_feedback("failed", robot.skill_errors[skill_name])

        # 初始化完成，进入执行阶段
        robot.skill_states[skill_name] = "EXECUTING"
        return robot.form_feedback("processing", "Initializing put down...", 20)

    except Exception as e:
        robot.skill_states[skill_name] = "FAILED"
        robot.skill_errors[skill_name] = f"Initialization failed: {str(e)}"
        return robot.form_feedback("failed", robot.skill_errors[skill_name])


def _handle_executing(robot, skill_name):
    """执行放下操作"""
    try:
        # 获取存储的数据
        hand_prim = robot.get_skill_data(skill_name, "hand_prim")
        object_prim = robot.get_skill_data(skill_name, "object_prim")
        object_prim_path = robot.get_skill_data(skill_name, "object_prim_path")
        joint_path = robot.get_skill_data(skill_name, "joint_path")

        # 获取舞台和关节
        stage = robot.scene_manager.stage
        joint_prim = stage.GetPrimAtPath(joint_path)

        if joint_prim.IsValid():
            # 禁用抓取关节
            joint = UsdPhysics.Joint(joint_prim)
            joint.GetJointEnabledAttr().Set(False)

            # 启用物体碰撞
            robot.scene_manager.set_collision_enabled(
                object_prim_path, collision_enabled=True
            )

            # 模拟惯性传递
            hand_lin_vel = hand_prim.get_linear_velocities()
            hand_ang_vel = hand_prim.get_angular_velocities()
            object_prim.set_linear_velocities(hand_lin_vel)
            object_prim.set_angular_velocities(hand_ang_vel)

            logger.info(
                f"INFO: Object '{object_prim.name}' dropped and physics restored."
            )
            robot.skill_states[skill_name] = "COMPLETED"
            return robot.form_feedback("processing", "Putting down object...", 90)

        else:
            logger.info(f"WARNING: Joint '{joint_path}' not found, cannot disable.")
            robot.skill_states[skill_name] = "FAILED"
            robot.skill_errors[skill_name] = "Grasp joint not found"
            return robot.form_feedback("failed", robot.skill_errors[skill_name])

    except Exception as e:
        robot.skill_states[skill_name] = "FAILED"
        robot.skill_errors[skill_name] = f"Put down execution failed: {str(e)}"
        return robot.form_feedback("failed", robot.skill_errors[skill_name])


def _handle_completed(robot, skill_name):
    """处理完成状态"""
    return robot.form_feedback("completed", "Object put down successfully!", 100)


def _handle_failed(robot, skill_name):
    """处理失败状态"""
    error_msg = robot.skill_errors.get(skill_name, "Unknown error")
    return robot.form_feedback("failed", error_msg)
