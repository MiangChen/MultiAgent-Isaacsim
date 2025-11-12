# Third library imports
import carb
import numpy as np

# Local project imports
from log.log_manager import LogManager
from physics_engine.omni_utils import omni
from physics_engine.pxr_utils import Gf
from simulation.robot.cfg.cfg_robot import CfgRobot
from simulation.sensor.lidar.cfg_lidar import CfgLidar

logger = LogManager.get_logger(__name__)


class LidarOmni:
    """
    一个高层级的封装器，用于在 Isaac Sim 中创建、管理和读取 RTX Lidar 传感器数据。
    这个方法创建的Lidar是使用的 Omni 的API
    """

    def __init__(self, cfg_lidar: CfgLidar, cfg_robot: CfgRobot = None):
        self.cfg_lidar = cfg_lidar
        self.cfg_robot = cfg_robot
        self.lidar = None  # 用于持有 LidarRtx 实例
        self.render_product = None
        self.annotator = None
        self._depth2pc_lut = None
        self._depth = np.empty(
            self.cfg_lidar.output_size, dtype=np.float32
        )  # 存储lidar 的原始深度信息

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
            translation=Gf.Vec3d(*self.cfg_lidar.translation),
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
        """
        从深度图生成点云，使用缓存的LUT
        
        返回的点云已经应用了 LiDAR 的局部旋转（相对于父对象）
        shape: width * height * 3
        """
        if self._depth2pc_lut is None:
            self._depth2pc_lut = self.create_depth2pc_lut()
        self.get_depth()

        # 生成局部坐标系下的点云
        point_cloud = (
            self._depth.reshape((self._depth.shape[0], self._depth.shape[1], 1))
            * self._depth2pc_lut
        )
        
        # 应用 LiDAR 的局部旋转（相对于父对象的旋转）
        # 这样点云就在父对象（无人机）的坐标系下了
        point_cloud = self._apply_local_rotation(point_cloud)
        
        return point_cloud
    
    def _apply_local_rotation(self, point_cloud: np.ndarray) -> np.ndarray:
        """
        应用 LiDAR 的局部旋转到点云
        
        Args:
            point_cloud: 点云 [H, W, 3]
            
        Returns:
            旋转后的点云 [H, W, 3]
        """
        # 获取 LiDAR 的局部旋转（相对于父对象）
        quat = self.cfg_lidar.quat  # (w, x, y, z)
        
        # 如果是单位四元数，不需要旋转
        if np.allclose(quat, [1, 0, 0, 0]):
            return point_cloud
        
        # 转换为旋转矩阵
        from scipy.spatial.transform import Rotation as R
        # scipy 使用 (x, y, z, w) 格式
        rotation = R.from_quat([quat[1], quat[2], quat[3], quat[0]])
        rotation_matrix = rotation.as_matrix()
        
        # 保存原始形状
        original_shape = point_cloud.shape
        
        # Reshape 为 [N, 3] 进行旋转
        points_flat = point_cloud.reshape(-1, 3)
        
        # 应用旋转: p_rotated = R * p
        points_rotated = (rotation_matrix @ points_flat.T).T
        
        # 恢复原始形状
        return points_rotated.reshape(original_shape)
