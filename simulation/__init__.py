"""
Simulation Layer - CARLA Style

参考CARLA的架构设计，封装Isaac Sim的仿真层
"""

from simulation.transform import Transform, Location, Rotation, Vector3D
from simulation.actor_blueprint import ActorBlueprint, BlueprintLibrary

# Backward compatibility aliases
Blueprint = ActorBlueprint

__all__ = [
    "Server",
    "World",
    "Actor",
    "RobotActor",
    "Transform",
    "Location",
    "Rotation",
    "Vector3D",
    "ActorBlueprint",
    "BlueprintLibrary",
    # Backward compatibility
    "Blueprint",
    # 控制类
    # 'RobotControl',
    # 'NavigationControl',
    # 'ExplorationControl',
    # 'TakeOffControl',
    # 'LandControl',
    # 'HoverControl',
    # 'DetectionControl',
    # 'PhotoControl',
    # 'VideoControl',
    # 'ScanControl',
    "GraspControl",
    "ReleaseControl",
    # 'PlaceControl',
    # 'PushControl',
    # 'PullControl',
    # 'FollowControl',
    # 'EscortControl',
    # 'HandoverControl',
    # 'CommunicationControl',
    # 'create_control',
]
