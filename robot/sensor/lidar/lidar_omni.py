# Third library imports
import carb
import numpy as np

# Local project imports
from physics_engine.omni_utils import omni
from log.log_manager import LogManager
from robot.cfg.cfg_robot import CfgRobot
from robot.sensor.lidar.cfg_lidar import CfgLidar
from physics_engine.pxr_utils import Gf

logger = LogManager.get_logger(__name__)


class LidarOmni:
    """
    一个高层级的封装器，用于在 Isaac Sim 中创建、管理和读取 RTX Lidar 传感器数据。
    这个方法创建的Lidar是使用的Isaacsim的API
    """

    def __init__(self, cfg_lidar: CfgLidar, cfg_robot: CfgRobot = None):
        self.cfg_lidar = cfg_lidar
        self.cfg_robot = cfg_robot
        self.lidar = None
        self.render_product = None
        self.annotator = None
        self._depth2pc_lut = None
        self._depth = np.empty((self.cfg_lidar.output_size[0], self.cfg_lidar.output_size[1]),
                               dtype=np.float32)  # 存储lidar 的原始深度信息

        self.create_lidar()

    def create_lidar(self) -> None:
        if self.cfg_lidar.prim_path is None:
            self.cfg_lidar.prim_path = (
                f"{self.cfg_robot.path_prim_robot}/lidar/{self.cfg_lidar.name}"
            )

        _, self.lidar = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path=self.cfg_lidar.prim_path,
            parent=None,
            config=self.cfg_lidar.config_file_name,
            translation=self.cfg_lidar.translation,
            orientation=Gf.Quatd(*self.cfg_lidar.quat),  # wxyz
        )
        self.render_product = omni.replicator.core.create.render_product(
            self.lidar.GetPath(), [1, 1]
        )
        self.annotator = omni.replicator.core.AnnotatorRegistry.get_annotator(
            "RtxSensorCpuIsaacReadRTXLidarData"
        )
        self.annotator.attach(self.render_product)
        logger.info(
            f"Lidar Omni sensor created or encapsulated at path: {self.cfg_lidar.prim_path}"
        )
        return None

    @carb.profiler.profile
    def get_depth(self):
        """直接获取深度数据"""

        self._depth.fill(self.cfg_lidar.max_depth)
        data = self.annotator.get_data()
        lidar_depths = data["distances"]
        emitter_ids = data["emitterIds"]

        depths_flat = self._depth.reshape(-1)
        depths_flat[emitter_ids] = lidar_depths

        self._depth = np.minimum(self._depth, self.cfg_lidar.max_depth)
        return self._depth

    @carb.profiler.profile
    def get_pointcloud(self):
        """从深度图生成点云，使用缓存的LUT"""
        if self._depth2pc_lut is None:
            self._depth2pc_lut = self.create_depth2pc_lut()
        self.get_depth()

        point_cloud = self._depth.reshape((self._depth.shape[0], self._depth.shape[1], 1)) * self._depth2pc_lut
        return point_cloud

    def create_depth2pc_lut(self):
        """Create lookup table for depth to pointcloud conversion."""
        erp_width = self.cfg_lidar.erp_width
        erp_height = self.cfg_lidar.erp_height
        erp_width_fov = self.cfg_lidar.erp_width_fov
        erp_height_fov = self.cfg_lidar.erp_height_fov

        fx_erp = erp_width / np.deg2rad(erp_width_fov)
        fy_erp = erp_height / np.deg2rad(erp_height_fov)
        cx_erp = (erp_width - 1) / 2
        cy_erp = (erp_height - 1) / 2

        grid = np.mgrid[0:erp_height, 0:erp_width]
        v, u = grid[0], grid[1]
        theta_l_map = -(u - cx_erp) / fx_erp  # elevation
        phi_l_map = -(v - cy_erp) / fy_erp  # azimuth

        sin_el = np.sin(theta_l_map)
        cos_el = np.cos(theta_l_map)
        sin_az = np.sin(phi_l_map)
        cos_az = np.cos(phi_l_map)
        X = cos_az * cos_el
        Y = sin_az * cos_el
        Z = -sin_el
        point_cloud = np.stack([X, Y, Z], axis=-1).astype(np.float32)

        return point_cloud


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
    # render_product_lfr = omni.replicator.core.create.render_product(
    #     sensor_lfr.GetPath(), [1, 1]
    # )

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


def wrapper_lidar(lidar_annotators):
    """Create a wrapper function for lidar simulation step that captures all required arguments."""

    depths = np.empty((2, 352, 120), dtype=np.float32)

    @carb.profiler.profile
    def lidar_step_wrapper():
        nonlocal depths

        depths.fill(1000)
        # Process lidar data
        for i, annotator in enumerate(lidar_annotators):
            data = annotator.get_data()
            # for key in data.keys():
            #     print(key)
            #     try:
            #         print(data[key].shape)
            #     except Exception as e:
            #         print(data[key])
            # print("***********************")
            # print(f"Lidar {i}\n{data}")
            lidar_depths = data["distances"]
            emitter_ids = data["emitterIds"]
            depths_flat = depths[i].reshape((depths.shape[1] * depths.shape[2]))
            depths_flat[emitter_ids] = lidar_depths
        return depths

    return lidar_step_wrapper
