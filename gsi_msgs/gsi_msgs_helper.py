"""
GSI Messages 导入辅助模块
在需要使用 gsi_msgs 的地方导入这个模块
"""

import sys
import os
from pathlib import Path


# 自动设置 gsi_msgs 路径
def setup_gsi_msgs():
    """设置 gsi_msgs 路径"""
    current_dir = Path(__file__).parent.absolute()

    plan_msgs_path = current_dir / "install/plan_msgs/lib/python3.12/site-packages"
    scene_msgs_path = current_dir / "install/scene_msgs/lib/python3.12/site-packages"

    paths_added = []

    if plan_msgs_path.exists() and str(plan_msgs_path) not in sys.path:
        sys.path.insert(0, str(plan_msgs_path))
        paths_added.append(str(plan_msgs_path))

    if scene_msgs_path.exists() and str(scene_msgs_path) not in sys.path:
        sys.path.insert(0, str(scene_msgs_path))
        paths_added.append(str(scene_msgs_path))

    return paths_added


# 自动执行设置
_paths = setup_gsi_msgs()

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

print(f"✅ GSI Messages 导入成功，路径: {_paths}")
