"""Control Module - CARLA-style control classes for robots and skills"""

from typing import List, Optional, Any
from dataclasses import dataclass, field


# Low-level motion control
@dataclass
class RobotControl:
    """Low-level velocity control (all robot types)"""
    linear_velocity: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    angular_velocity: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])


# Mobility skills
@dataclass
class NavigationControl:
    target: Optional[List[float]] = None
    mode: str = 'a_star'
    speed: float = 1.0
    avoid_obstacles: bool = True
    tolerance: float = 0.5


@dataclass
class ExplorationControl:
    area: Optional[Any] = None
    strategy: str = 'frontier'
    coverage: float = 0.95
    speed: float = 1.0

@dataclass
class TakeOffControl:
    target_height: float = 5.0
    speed: float = 1.0

@dataclass
class LandControl:
    speed: float = 0.5
    target_height: float = 0.0

@dataclass
class HoverControl:
    altitude: Optional[float] = None
    duration: Optional[float] = None


# Perception skills
@dataclass
class DetectionControl:
    target_class: Optional[str] = None
    confidence: float = 0.8
    track: bool = False
    camera_id: str = 'front'

@dataclass
class PhotoControl:
    camera_id: str = 'front'
    resolution: List[int] = field(default_factory=lambda: [1920, 1080])
    format: str = 'jpg'
    save_path: Optional[str] = None

@dataclass
class VideoControl:
    camera_id: str = 'front'
    duration: float = 10.0
    resolution: List[int] = field(default_factory=lambda: [1920, 1080])
    fps: int = 30
    save_path: Optional[str] = None

@dataclass
class ScanControl:
    area: Optional[Any] = None
    resolution: float = 0.1
    save_path: Optional[str] = None


# Manipulation skills
@dataclass
class GraspControl:
    """抓取控制（纯数据，不调用 Isaac Sim API）"""
    hand_prim_path: str = ""
    object_prim_path: str = ""
    local_pos_hand: List[float] = field(default_factory=lambda: [0.0, 0.0, 1.0])
    local_pos_object: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    axis: List[float] = field(default_factory=lambda: [0.0, 0.0, 1.0])
    action: str = "check_distance"  # "check_distance", "attach"
    distance_threshold: float = 2.0

@dataclass
class ReleaseControl:
    """释放控制（纯数据）"""
    object_prim_path: str = ""
    joint_path: str = ""
    gentle: bool = True

# @dataclass
# class PlaceControl:
#     """放置控制（纯数据）"""
#     object_prim_path: str = ""
#     joint_path: str = ""
#     target_location: Optional[List[float]] = None
#     orientation: Optional[List[float]] = None
#     gentle: bool = True

# @dataclass
# class PushControl:
#     target_object: Optional[Any] = None
#     direction: Optional[List[float]] = None
#     force: float = 5.0
#
# @dataclass
# class PullControl:
#     target_object: Optional[Any] = None
#     direction: Optional[List[float]] = None
#     force: float = 5.0


# Interaction skills
@dataclass
class FollowControl:
    target: Optional[Any] = None
    distance: float = 2.0
    mode: str = 'behind'

@dataclass
class EscortControl:
    target: Optional[Any] = None
    destination: Optional[List[float]] = None
    distance: float = 1.5

@dataclass
class HandoverControl:
    object: Optional[Any] = None
    to_robot: Optional[Any] = None
    gentle: bool = True

@dataclass
class CommunicationControl:
    message: Optional[str] = None
    recipients: List[Any] = field(default_factory=list)
    priority: str = 'normal'
    broadcast: bool = False


# Factory function
def create_control(control_type: str, **kwargs):
    """Factory: create Control object by type name"""
    control_classes = {
        'robot': RobotControl,
        # 'navigation': NavigationControl,
        # 'exploration': ExplorationControl,
        # 'takeoff': TakeOffControl,
        # 'land': LandControl,
        # 'hover': HoverControl,
        # 'detection': DetectionControl,
        # 'photo': PhotoControl,
        # 'video': VideoControl,
        # 'scan': ScanControl,
        'grasp': GraspControl,
        'release': ReleaseControl,
        # 'place': PlaceControl,
        # 'push': PushControl,
        # 'pull': PullControl,
        # 'follow': FollowControl,
        # 'escort': EscortControl,
        # 'handover': HandoverControl,
        # 'communication': CommunicationControl,
    }

    control_class = control_classes.get(control_type.lower())
    if not control_class:
        raise ValueError(f"Unknown control type: {control_type}")

    return control_class(**kwargs)
