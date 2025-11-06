# =============================================================================
# Base LiDAR Module - LiDAR Sensor Base Implementation
# =============================================================================
#
# This module provides base LiDAR sensor functionality for drone platforms,
# including sensor creation and configuration within Isaac Sim.
#
# =============================================================================

# Standard library imports
import os
import shutil
from typing import Optional, Tuple, Dict, Any

# Local Library import
from log.log_manager import LogManager
from physics_engine.pxr_utils import Gf
from physics_engine.isaacsim_utils import get_extension_path_from_name, LidarRtx
from robot.sensor.lidar.cfg_lidar import CfgLidar

logger = LogManager.get_logger(__name__)


class LidarIsaac:
    """
    一个高层级的封装器，用于在 Isaac Sim 中创建、管理和读取 RTX Lidar 传感器数据。
    这个方法创建的Lidar是使用的 Isaacsim 的API
    """

    def __init__(self, cfg_lidar: CfgLidar, cfg_robot):
        self.cfg_lidar = cfg_lidar
        self.cfg_robot = cfg_robot
        self.lidar: Optional[LidarRtx] = None  # 用于持有 LidarRtx 实例

    def create_lidar(self, prim_path: Optional[str] = None) -> None:

        if prim_path is None:
            self.cfg_lidar.prim_path = (
                f"{self.cfg_robot.path_prim_swarm}/lidar/{self.cfg_lidar.name}"
            )
        else:
            self.cfg_lidar.prim_path = prim_path

        self.lidar = LidarRtx(
            prim_path=self.cfg_lidar.prim_path,
            name=self.cfg_lidar.type,
            translation=self.cfg_lidar.position,
            orientation=self.cfg_lidar.quat,
            config_file_name=self.cfg_lidar.config_file_name,
        )

        logger.info(
            f"Lidar sensor created or encapsulated at path: {self.cfg_lidar.prim_path}"
        )

    def copy_lidar_config(self, lidar_config):
        """Copy the lidar config from the current directory to the extension directory."""
        src_file_path = os.path.join(
            os.path.dirname(__file__), "../config/", lidar_config + ".json"
        )
        dst_file_path = os.path.abspath(
            os.path.join(
                get_extension_path_from_name("isaacsim.sensors.rtx"),
                "data/lidar_configs/" + lidar_config + ".json",
            )
        )
        shutil.copyfile(
            src_file_path,
            dst_file_path,
        )
        return dst_file_path

    def initialize(self) -> bool:
        """
        初始化 Lidar 传感器，这将在内部设置好 Replicator 图以准备数据流。

        Returns:
            True 如果 Lidar 实例已成功初始化。
        """
        if not self.lidar:
            return False

        self.lidar.initialize()
        return self.lidar.is_valid()

    def get_current_frame(self) -> Dict[str, Any]:
        """
        获取 Lidar 传感器的最新一帧数据。

        Returns:
            一个包含 Lidar 数据的字典，例如 {'distances': ..., 'emitterIds': ...}。
            如果传感器未准备好，则返回空字典。
        """
        if self.lidar and self.lidar.is_valid():
            return self.lidar.get_current_frame()
        else:
            print(type(self.lidar), self.lidar.is_valid())
            logger.warning(
                "Attempted to get frame from an invalid or uninitialized Lidar sensor."
            )
            return {}

    def get_world_pose(self) -> Optional[Tuple[Gf.Vec3d, Gf.Quatd]]:
        """
        获取 Lidar 传感器的世界位姿。

        Returns:
            一个包含位置 (Gf.Vec3d) 和朝向 (Gf.Quatd) 的元组，如果失败则返回 None。
        """
        if self.lidar and self.lidar.is_valid():
            prim = self.lidar.prim
            xformable = Gf.Xformable(prim)
            world_transform = xformable.GetLocalToWorldTransform()
            return (
                world_transform.ExtractTranslation(),
                world_transform.ExtractRotationQuat(),
            )
        return None
