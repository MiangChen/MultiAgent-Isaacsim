"""
Messages 导入辅助模块
在需要使用的地方导入这个模块
"""

import sys
from pathlib import Path


# 自动设置路径
def setup_msg_path():
    """设置 ros_msg 路径"""
    current_dir = Path(__file__).parent.absolute()

    plan_msgs_path = current_dir / "install/plan_msgs/local/lib/python3.10/dist-packages"
    scene_msgs_path = current_dir / "install/scene_msgs/local/lib/python3.10/dist-packages"
    print("in setup", current_dir, plan_msgs_path)

    paths_added = []

    if plan_msgs_path.exists() and str(plan_msgs_path) not in sys.path:
        sys.path.insert(0, str(plan_msgs_path))
        paths_added.append(str(plan_msgs_path))

    if scene_msgs_path.exists() and str(scene_msgs_path) not in sys.path:
        sys.path.insert(0, str(scene_msgs_path))
        paths_added.append(str(scene_msgs_path))

    return paths_added


# 自动执行设置
_paths = setup_msg_path()

# 导入所有消息类型

from plan_msgs.msg import (
    Parameter,
    Plan,
    RobotFeedback,
    RobotSkill,
    SkillInfo,
    TimestepSkills,
    VelTwistPose,
    SkillFeedback,  # 新增导入
)

from plan_msgs.action import PlanExecution, SkillExecution

from scene_msgs.msg import (
    SceneModifications,
    PrimTransform,
)

# 导出所有消息类型
__all__ = [
    "Parameter",
    "Plan",
    "RobotFeedback",
    "RobotSkill",
    "SkillInfo",
    "TimestepSkills",
    "VelTwistPose",
    "SceneModifications",
    "PrimTransform",
    "PlanExecution",
    "SkillExecution",
    "SkillFeedback",
]

print(f"✅ Messages 导入成功，路径: {_paths}")
