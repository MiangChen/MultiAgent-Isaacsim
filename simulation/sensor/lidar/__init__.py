# =============================================================================
# LiDAR Sensor Package - LiDAR Sensor Classes
# =============================================================================
#
# This package provides LiDAR sensor implementations and configurations
# for robot perception systems within the Isaac Sim environment.
#
# =============================================================================

# Blueprints can be imported without Isaac Sim
from .lidar_blueprint import IsaacLidarBlueprint, OmniLidarBlueprint

# Config classes (no Isaac Sim dependency)
from .cfg_lidar import CfgLidar

# LiDAR implementations (require Isaac Sim - import on demand)
# from .lidar_isaac import LidarIsaac
# from .lidar_omni import LidarOmni

__all__ = [
    "IsaacLidarBlueprint",
    "OmniLidarBlueprint",
    "CfgLidar",
    # 'LidarIsaac',  # Import on demand
    # 'LidarOmni',   # Import on demand
]
