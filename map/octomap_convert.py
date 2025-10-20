# =============================================================================
# Octomap Convert Module - Grid Map to OctoMap Conversion
# =============================================================================
#
# This module provides functionality to convert Isaac Sim's dense 3D GridMap
# to ROS2 OctoMap messages for efficient spatial representation.
#
# =============================================================================

# Third-party library imports
import numpy as np
import octomap

# ROS2 imports
from octomap_msgs.msg import Octomap
from std_msgs.msg import Header

# from map_grid_map import GridMap # 假设 GridMap 类已定义


def convert_grid_to_octomap_msg(grid_map: GridMap, frame_id: str = "map") -> Octomap:
    """
    将 Isaac Sim 的稠密 3D GridMap 高效转换为 ROS2 OctoMap 消息。
    """
    if grid_map.value_map is None:
        raise ValueError("地图未生成 (grid_map.value_map is None)。")

    # 1. 初始化与 GridMap 分辨率相同的八叉树
    map_info = grid_map.get_map_info()
    resolution = map_info["cell_size"]
    tree = octomap.OcTree(resolution)

    # 2. 批量插入所有被占用的点
    min_bounds = np.array(map_info["min_bound"])
    occupied_indices = np.argwhere(grid_map.value_map == map_info["occupied_value"])
    if len(occupied_indices) > 0:
        # 将索引转换为世界坐标 (体素中心)
        occupied_coords = min_bounds + (occupied_indices + 0.5) * resolution
        tree.insertPointCloud(
            pointcloud=occupied_coords.astype(np.float32),
            origin=np.array([0, 0, 0], dtype=np.float32),
        )

    # 3. 将所有空闲空间节点的状态设置为空闲
    free_indices = np.argwhere(grid_map.value_map == map_info["free_value"])
    if len(free_indices) > 0:
        free_coords = min_bounds + (free_indices + 0.5) * resolution
        for coord in free_coords:
            tree.updateNode(coord.astype(np.float32), False)  # False 表示空闲

    # 4. 将八叉树序列化并创建 ROS2 消息
    success, binary_data = tree.writeBinary(full=True)
    if not success:
        raise RuntimeError("八叉树序列化失败。")

    msg = Octomap()
    msg.header.frame_id = frame_id
    # msg.header.stamp = your_node.get_clock().now().to_msg() # 在节点中设置时间戳
    msg.binary = True
    msg.id = "OcTree"
    msg.resolution = resolution
    msg.data = list(binary_data)  # bytes 转换为 list[int]

    return msg


# --- 简洁的示例用法 ---
if __name__ == "__main__":
    # 假设你有一个已经生成了地图的 grid_map 对象
    # from map_grid_map import GridMap
    class MockGridMap:  # 使用一个模拟的 GridMap 类进行演示
        def __init__(self):
            self.cell_size = 0.5
            self.min_bounds = np.array([-10, -10, 0])
            self.free_value = 0
            self.occupied_value = 100
            # 创建一个 40x40x10 的地图
            self.value_map = np.full((40, 40, 10), self.free_value, dtype=np.uint8)
            # 在中间创建一个障碍物
            self.value_map[18:22, 18:22, 0:5] = self.occupied_value

        def get_map_info(self):
            return {
                "cell_size": self.cell_size,
                "min_bound": self.min_bounds,
                "free_value": self.free_value,
                "occupied_value": self.occupied_value,
            }

    grid_map_instance = MockGridMap()

    # 调用转换函数
    octomap_message = convert_grid_to_octomap_msg(grid_map_instance, frame_id="world")

    print(f"ROS2 OctoMap 消息已创建:")
    print(f"  - Frame ID: {octomap_message.header.frame_id}")
    print(f"  - 分辨率: {octomap_message.resolution}")
    print(f"  - 数据大小: {len(octomap_message.data)} bytes")
