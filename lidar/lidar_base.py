import os
import shutil
from isaacsim.core.utils.extensions import get_extension_path_from_name
import omni
import carb

def add_drone_lidar(drone_prim_path, lidar_config):
    """Adds lidar sensors to the drone."""

    # 1. Create The Camera
    _, sensor_lfr = omni.kit.commands.execute(
        "IsaacSensorCreateRtxLidar",
        path=drone_prim_path + "/Lidar/lfr",
        parent=None,
        config=lidar_config,
        translation=(0, 0, 0),
        orientation=Gf.Quatd(1, 0, 0, 0),
    )
    # 2. Create and Attach a render product to the camera
    render_product_lfr = omni.replicator.core.create.render_product(
        sensor_lfr.GetPath(), [1, 1]
    )

    # 3. Create Annotator to read the data from with annotator.get_data()
    annotator_lfr = omni.replicator.core.AnnotatorRegistry.get_annotator(
        "RtxSensorCpuIsaacReadRTXLidarData"
    )
    annotator_lfr.attach(render_product_lfr)

    # 1. Create The Camera
    _, sensor_ubd = omni.kit.commands.execute(
        "IsaacSensorCreateRtxLidar",
        path=drone_prim_path + "/Lidar/ubd",
        parent=None,
        config=lidar_config,
        translation=(0, 0, 0),
        orientation=Gf.Quatd(0, 0, 0.7071067811865476, 0.7071067811865476),
    )
    # 2. Create and Attach a render product to the camera
    render_product_ubd = omni.replicator.core.create.render_product(
        sensor_ubd.GetPath(), [1, 1]
    )

    # 3. Create Annotator to read the data from with annotator.get_data()
    annotator_ubd = omni.replicator.core.AnnotatorRegistry.get_annotator(
        "RtxSensorCpuIsaacReadRTXLidarData"
    )
    annotator_ubd.attach(render_product_ubd)

    return annotator_lfr, annotator_ubd


def create_lidar_step_wrapper(lidar_annotators):
    """Create a wrapper function for lidar simulation step that captures all required arguments."""

    depths = np.empty((2, 352, 120), dtype=np.float32)

    @carb.profiler.profile
    def lidar_step_wrapper():
        nonlocal depths

        depths.fill(1000)
        # Process lidar data
        for i, annotator in enumerate(lidar_annotators):
            data = annotator.get_data()
            for key in data.keys():
                print(key)
                try:
                    print(data[key].shape)
                except Exception as e:
                    print(data[key])
            print("***********************")
            # print(f"Lidar {i}\n{data}")
            lidar_depths = data["distances"]
            emitter_ids = data["emitterIds"]
            depths_flat = depths[i].reshape((depths.shape[1] * depths.shape[2]))
            depths_flat[emitter_ids] = lidar_depths
        return depths

    return lidar_step_wrapper


#######################

from typing import Optional, Tuple, Dict, Any
import numpy as np
from pxr import Gf

from isaacsim.sensors.rtx import LidarRtx

from lidar.lidar_cfg import LidarCfg
from robot.robot_cfg import RobotCfg
from log.log_manager import LogManager

logger = LogManager.get_logger(__name__)


class LidarBase:
    """
    一个高层级的封装器，用于在 Isaac Sim 中创建、管理和读取 RTX Lidar 传感器数据。
    """

    def __init__(self, cfg_lidar: LidarCfg, cfg_body: RobotCfg = None):
        self.cfg_lidar = cfg_lidar
        self.cfg_body = cfg_body
        self.lidar_sensor: Optional[LidarRtx] = None  # 用于持有 LidarRtx 实例

    def create_lidar(self, prim_path: Optional[str] = None) -> None:
        """
        根据配置，在 USD 舞台中创建或封装一个 LidarRtx 传感器。
        """
        if prim_path is None:
            # 如果没有提供特定路径，则在机器人 Prim 下创建一个默认路径
            self.cfg_lidar.prim_path = f"{self.cfg_body.prim_path_swarm}/lidar/{self.cfg_lidar.type}"
        else:
            self.cfg_lidar.prim_path = prim_path



        self.lidar_sensor = LidarRtx(
            prim_path=self.cfg_lidar.prim_path,
            name=self.cfg_lidar.type,
            translation=self.cfg_lidar.position,
            orientation=self.cfg_lidar.quat,
            config_file_name=self.cfg_lidar.config_file_name,
        )

        logger.info(f"Lidar sensor created or encapsulated at path: {self.cfg_lidar.prim_path}")

    def copy_lidar_config(self, lidar_config):
        """Copy the lidar config from the current directory to the extension directory."""
        src_file_path = os.path.join(os.path.dirname(__file__), "../config/", lidar_config + ".json")
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
        if not self.lidar_sensor:
            return False

        self.lidar_sensor.initialize()
        return self.lidar_sensor.is_valid()

    def get_current_frame(self) -> Dict[str, Any]:
        """
        获取 Lidar 传感器的最新一帧数据。

        Returns:
            一个包含 Lidar 数据的字典，例如 {'distances': ..., 'emitterIds': ...}。
            如果传感器未准备好，则返回空字典。
        """
        if self.lidar_sensor and self.lidar_sensor.is_valid():
            return self.lidar_sensor.get_current_frame()
        else:
            print(type(self.lidar_sensor), self.lidar_sensor.is_valid())
            logger.warning("Attempted to get frame from an invalid or uninitialized Lidar sensor.")
            return {}

    def get_world_pose(self) -> Optional[Tuple[Gf.Vec3d, Gf.Quatd]]:
        """
        获取 Lidar 传感器的世界位姿。

        Returns:
            一个包含位置 (Gf.Vec3d) 和朝向 (Gf.Quatd) 的元组，如果失败则返回 None。
        """
        if self.lidar_sensor and self.lidar_sensor.is_valid():
            prim = self.lidar_sensor.prim
            xformable = Gf.Xformable(prim)
            world_transform = xformable.GetLocalToWorldTransform()
            return world_transform.ExtractTranslation(), world_transform.ExtractRotationQuat()
        return None
