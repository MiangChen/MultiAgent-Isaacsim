"""Control Module - CARLA-style control classes for robots and skills

Architecture:
- Control: Base class for all controls (pure data, CARLA-style)
- ContinuousControl: Applied every physics step (e.g., velocity)
- DiscreteControl: One-shot action with state management (e.g., grasp, release)
"""

from enum import Enum
from typing import List, Optional, Any
from dataclasses import dataclass, field


# ============= Action States =============
class ControlAction(Enum):
    """Unified control action states for discrete controls"""
    # Common states (all discrete controls)
    PENDING = "pending"
    EXECUTING = "executing"
    COMPLETED = "completed"
    FAILED = "failed"
    
    # Grasp-specific states
    CHECK_DISTANCE = "check_distance"
    ATTACH = "attach"
    
    # Release-specific states
    RELEASE = "release"
    
    # Place-specific states
    PLACE = "place"


# ============= Base Classes =============
@dataclass
class Control:
    """Base class for all controls (CARLA-style pure data object)"""
    pass


@dataclass
class ContinuousControl(Control):
    """
    Continuous control (applied every physics step, CARLA-style)
    
    Examples: velocity control, force control
    These controls are applied continuously and don't have "completed" state.
    """
    pass


@dataclass
class DiscreteControl(Control):
    """
    Discrete control (one-shot action with state management)
    
    Examples: grasp, release, place
    These controls execute once and have completion state.
    """
    action: ControlAction = ControlAction.PENDING
    
    def is_completed(self) -> bool:
        """Check if action is completed"""
        return self.action == ControlAction.COMPLETED
    
    def is_failed(self) -> bool:
        """Check if action failed"""
        return self.action == ControlAction.FAILED
    
    def is_pending(self) -> bool:
        """Check if action is pending"""
        return self.action == ControlAction.PENDING
    
    def mark_completed(self):
        """Mark action as completed"""
        self.action = ControlAction.COMPLETED
    
    def mark_failed(self):
        """Mark action as failed"""
        self.action = ControlAction.FAILED


# ============= Continuous Controls =============
@dataclass
class RobotControl(ContinuousControl):
    """
    Low-level velocity control (CARLA-style continuous control)
    
    Applied every physics step, no completion state.
    """
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


# ============= Discrete Controls (Manipulation) =============
@dataclass
class GraspControl(DiscreteControl):
    """
    Grasp control with multi-stage actions (discrete control)
    
    Actions:
    - CHECK_DISTANCE: Check if object is within grasp distance
    - ATTACH: Attach object to hand (create joint)
    - COMPLETED: Action completed
    """
    hand_prim_path: str = ""
    object_prim_path: str = ""
    action: ControlAction = ControlAction.CHECK_DISTANCE
    distance_threshold: float = 2.0
    local_pos_hand: List[float] = field(default_factory=lambda: [0.0, 0.0, 1.0])
    local_pos_object: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    axis: List[float] = field(default_factory=lambda: [0.0, 0.0, 1.0])


@dataclass
class ReleaseControl(DiscreteControl):
    """
    Release control (discrete one-shot action)
    
    Actions:
    - RELEASE: Release object from hand (disable joint)
    - COMPLETED: Action completed
    """
    object_prim_path: str = ""
    joint_path: str = ""
    action: ControlAction = ControlAction.RELEASE
    gentle: bool = True


@dataclass
class PlaceControl(DiscreteControl):
    """
    Place control (discrete one-shot action)
    
    Actions:
    - PLACE: Place object at target location
    - COMPLETED: Action completed
    """
    object_prim_path: str = ""
    joint_path: str = ""
    target_location: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    action: ControlAction = ControlAction.PLACE
    gentle: bool = True

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
