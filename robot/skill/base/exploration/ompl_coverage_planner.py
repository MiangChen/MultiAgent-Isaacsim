# =============================================================================
# OMPL Coverage Path Planning Implementation
# =============================================================================
#
# This module provides coverage path planning using OMPL (Open Motion Planning Library)
# algorithms for systematic area exploration.
#
# =============================================================================

import numpy as np
from typing import List, Tuple, Optional
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from log.log_manager import LogManager

logger = LogManager.get_logger(__name__)

try:
    from ompl import base as ob
    from ompl import geometric as og

    OMPL_AVAILABLE = True
except ImportError:
    logger.warning("OMPL not available. Falling back to custom implementation.")
    OMPL_AVAILABLE = False


class OMPLCoveragePlanner:
    """基于OMPL的覆盖路径规划器"""

    def __init__(self, robot_radius: float = 0.2, resolution: float = 0.1):
        self.robot_radius = robot_radius
        self.resolution = resolution
        self.space = None
        self.space_info = None

    def setup_state_space(self, bounds: List[Tuple[float, float]]):
        """设置状态空间

        Args:
            bounds: [(min_x, max_x), (min_y, max_y)] 边界
        """
        if not OMPL_AVAILABLE:
            raise ImportError("OMPL is not available")

        # 创建2D实数向量空间
        self.space = ob.RealVectorStateSpace(2)

        # 设置边界
        bounds_obj = ob.RealVectorBounds(2)
        bounds_obj.setLow(0, bounds[0][0])  # x_min
        bounds_obj.setHigh(0, bounds[0][1])  # x_max
        bounds_obj.setLow(1, bounds[1][0])  # y_min
        bounds_obj.setHigh(1, bounds[1][1])  # y_max

        self.space.setBounds(bounds_obj)

        # 创建空间信息
        self.space_info = ob.SpaceInformation(self.space)

        # 设置分辨率
        self.space_info.setStateValidityCheckingResolution(self.resolution)

    def set_validity_checker(
        self,
        polygon_coords: List[Tuple[float, float]],
        holes: Optional[List[List[Tuple[float, float]]]] = None,
    ):
        """设置有效性检查器（障碍物检测）

        Args:
            polygon_coords: 外边界多边形坐标
            holes: 内部障碍物列表
        """
        from shapely.geometry import Polygon, Point

        # 创建多边形
        if holes:
            self.polygon = Polygon(polygon_coords, holes=holes)
        else:
            self.polygon = Polygon(polygon_coords)

        # 收缩多边形以考虑机器人半径
        self.safe_polygon = self.polygon.buffer(-self.robot_radius)

        def is_state_valid(state):
            """检查状态是否有效"""
            x = state[0]
            y = state[1]
            point = Point(x, y)
            return self.safe_polygon.contains(point)

        # 设置有效性检查器
        validity_checker = ob.StateValidityCheckerFn(is_state_valid)
        self.space_info.setStateValidityChecker(validity_checker)
        self.space_info.setup()

    def plan_stc_coverage(
        self, start_pos: Tuple[float, float], coverage_resolution: float = 0.5
    ) -> Optional[List[Tuple[float, float]]]:
        """使用STC (Spanning Tree Coverage) 算法规划覆盖路径

        Args:
            start_pos: 起始位置 (x, y)
            coverage_resolution: 覆盖分辨率

        Returns:
            路径点列表或None
        """
        if not OMPL_AVAILABLE:
            logger.error("OMPL not available for STC coverage planning")
            return None

        try:
            # 创建问题定义
            pdef = ob.ProblemDefinition(self.space_info)

            # 设置起始状态
            start = ob.State(self.space)
            start[0] = start_pos[0]
            start[1] = start_pos[1]
            pdef.setStartState(start)

            # 创建STC规划器
            planner = og.STC(self.space_info)
            planner.setProblemDefinition(pdef)
            planner.setup()

            # 设置覆盖参数
            planner.setRange(coverage_resolution)

            # 规划
            solved = planner.solve(10.0)  # 10秒超时

            if solved:
                # 获取解路径
                path = pdef.getSolutionPath()
                path.interpolate()  # 插值平滑

                # 转换为坐标列表
                waypoints = []
                for i in range(path.getStateCount()):
                    state = path.getState(i)
                    waypoints.append((float(state[0]), float(state[1])))

                logger.info(
                    f"STC coverage planning succeeded with {len(waypoints)} waypoints"
                )
                return waypoints
            else:
                logger.error("STC coverage planning failed")
                return None

        except Exception as e:
            logger.error(f"STC coverage planning error: {e}")
            return None

    def plan_bstar_coverage(
        self, start_pos: Tuple[float, float], grid_resolution: float = 0.5
    ) -> Optional[List[Tuple[float, float]]]:
        """使用B* 算法规划覆盖路径

        Args:
            start_pos: 起始位置 (x, y)
            grid_resolution: 网格分辨率

        Returns:
            路径点列表或None
        """
        if not OMPL_AVAILABLE:
            logger.error("OMPL not available for B* coverage planning")
            return None

        try:
            # 创建问题定义
            pdef = ob.ProblemDefinition(self.space_info)

            # 设置起始状态
            start = ob.State(self.space)
            start[0] = start_pos[0]
            start[1] = start_pos[1]
            pdef.setStartState(start)

            # 创建B*规划器
            planner = og.BStar(self.space_info)
            planner.setProblemDefinition(pdef)
            planner.setup()

            # 设置网格分辨率
            planner.setGridResolution(grid_resolution)

            # 规划
            solved = planner.solve(15.0)  # 15秒超时

            if solved:
                path = pdef.getSolutionPath()
                path.interpolate()

                waypoints = []
                for i in range(path.getStateCount()):
                    state = path.getState(i)
                    waypoints.append((float(state[0]), float(state[1])))

                logger.info(
                    f"B* coverage planning succeeded with {len(waypoints)} waypoints"
                )
                return waypoints
            else:
                logger.error("B* coverage planning failed")
                return None

        except Exception as e:
            logger.error(f"B* coverage planning error: {e}")
            return None

    def plan_grid_coverage(
        self,
        bounds: List[Tuple[float, float]],
        spacing: float = 1.0,
        angle: float = 0.0,
    ) -> List[Tuple[float, float]]:
        """网格覆盖算法（备用方案，不依赖OMPL）

        Args:
            bounds: [(min_x, max_x), (min_y, max_y)]
            spacing: 网格间距
            angle: 扫掠角度（弧度）

        Returns:
            路径点列表
        """
        from shapely.geometry import LineString, Point
        from shapely.affinity import rotate
        import math

        # 创建网格点
        min_x, max_x = bounds[0]
        min_y, max_y = bounds[1]

        # 生成网格
        x_coords = np.arange(min_x, max_x + spacing, spacing)
        y_coords = np.arange(min_y, max_y + spacing, spacing)

        waypoints = []

        # 蛇形扫掠
        for i, x in enumerate(x_coords):
            if i % 2 == 0:  # 偶数列：从下到上
                y_list = y_coords
            else:  # 奇数列：从上到下
                y_list = y_coords[::-1]

            for y in y_list:
                point = Point(x, y)
                # 检查点是否在安全区域内
                if hasattr(self, "safe_polygon") and self.safe_polygon.contains(point):
                    waypoints.append((x, y))

        # 应用旋转
        if angle != 0:
            rotated_waypoints = []
            center_x = (min_x + max_x) / 2
            center_y = (min_y + max_y) / 2

            cos_a = math.cos(angle)
            sin_a = math.sin(angle)

            for x, y in waypoints:
                # 平移到原点
                tx = x - center_x
                ty = y - center_y
                # 旋转
                rx = tx * cos_a - ty * sin_a
                ry = tx * sin_a + ty * cos_a
                # 平移回去
                rotated_waypoints.append((rx + center_x, ry + center_y))

            waypoints = rotated_waypoints

        logger.info(f"Grid coverage planning generated {len(waypoints)} waypoints")
        return waypoints


def plan_ompl_coverage_waypoints(**kwargs) -> Optional[Path]:
    """使用OMPL规划覆盖路径的主函数

    Args:
        robot: 机器人实例
        polygon_coords: 边界多边形坐标
        holes: 障碍物列表
        robot_radius: 机器人半径
        algorithm: 算法类型 ('stc', 'bstar', 'grid')
        coverage_resolution: 覆盖分辨率
        frame_id: 坐标系ID

    Returns:
        ROS Path消息或None
    """
    robot = kwargs.get("robot")
    polygon_coords = kwargs.get("polygon_coords")
    holes = kwargs.get("holes", [])
    robot_radius = kwargs.get("robot_radius", 0.2)
    algorithm = kwargs.get("algorithm", "stc")
    coverage_resolution = kwargs.get("coverage_resolution", 0.5)
    frame_id = kwargs.get("frame_id", "map")
    z_out = kwargs.get("z_out", 0.0)

    try:
        # 创建规划器
        planner = OMPLCoveragePlanner(robot_radius=robot_radius)

        # 计算边界
        coords_array = np.array(polygon_coords)
        bounds = [
            (coords_array[:, 0].min() - 1.0, coords_array[:, 0].max() + 1.0),
            (coords_array[:, 1].min() - 1.0, coords_array[:, 1].max() + 1.0),
        ]

        # 设置状态空间
        planner.setup_state_space(bounds)
        planner.set_validity_checker(polygon_coords, holes)

        # 获取起始位置
        if hasattr(robot, "body") and robot.body:
            start_pos_tensor, _ = robot.body.get_world_pose()
            start_pos = (float(start_pos_tensor[0]), float(start_pos_tensor[1]))
        else:
            # 使用多边形中心作为起始点
            start_pos = (np.mean(coords_array[:, 0]), np.mean(coords_array[:, 1]))

        # 根据算法类型规划路径
        waypoints = None
        if algorithm == "stc" and OMPL_AVAILABLE:
            waypoints = planner.plan_stc_coverage(start_pos, coverage_resolution)
        elif algorithm == "bstar" and OMPL_AVAILABLE:
            waypoints = planner.plan_bstar_coverage(start_pos, coverage_resolution)
        else:
            # 备用网格算法
            waypoints = planner.plan_grid_coverage(bounds, coverage_resolution)

        if not waypoints:
            logger.error("Coverage path planning failed")
            return None

        # 转换为ROS Path消息
        path_msg = Path()
        path_msg.header.frame_id = frame_id
        if hasattr(robot, "get_clock"):
            path_msg.header.stamp = robot.get_clock().now().to_msg()

        for x, y in waypoints:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = frame_id
            if hasattr(robot, "get_clock"):
                pose_stamped.header.stamp = robot.get_clock().now().to_msg()

            pose_stamped.pose.position.x = float(x)
            pose_stamped.pose.position.y = float(y)
            pose_stamped.pose.position.z = float(z_out)

            # 默认朝向
            pose_stamped.pose.orientation.x = 0.0
            pose_stamped.pose.orientation.y = 0.0
            pose_stamped.pose.orientation.z = 0.0
            pose_stamped.pose.orientation.w = 1.0

            path_msg.poses.append(pose_stamped)

        logger.info(
            f"OMPL coverage planning completed with {len(waypoints)} waypoints using {algorithm} algorithm"
        )
        return path_msg

    except Exception as e:
        logger.error(f"OMPL coverage planning failed: {e}")
        return None
