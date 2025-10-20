# =============================================================================
# Robot Trajectory Module - Robot Trajectory Tracking and Visualization
# =============================================================================
#
# This module manages and displays the historical trajectory of robots,
# providing efficient trajectory tracking with visual representation in
# the Isaac Sim environment.
#
# =============================================================================

# Third-party library imports
import numpy as np

# Local project imports
from physics_engine.isaacsim_utils import Scene, VisualSphere


class Trajectory:
    """
    Manages and displays the historical trajectory of a specified robot prim.
    """

    def __init__(
        self,
        robot_prim_path: str,
        max_points: int = 500,
        id: int = 0,
        color: tuple = (0.0, 1.0, 0.0),
        thickness: float = 2.0,
        scene: Scene = None,
        radius: float = 0.05,
    ):
        """
        Initializes the trajectory tracker.
        为什么这里使用了list,而不是一个queue记录历史轨迹?
        因为每一次加入一个历史轨迹, 就需要刷新一次scene中的历史轨迹, 如果是用的是queue, 那么每一次更新点的时候, queue中所有点的下标都会改变, 场景中每一个历史轨迹点重新更新, 次数是 N
        如果是list, 用一个指针指向list中最新加入的历史轨迹点, 同时这个指针和scene中特定的点保持一致, 只需要更新特定的那一个点即可 次数是1
        """

        self.robot_prim_path = robot_prim_path
        self.max_points = max_points
        self.color = color  # RGB
        self.trajectory = [[0, 0, 0] for i in range(self.max_points)]  # 历史轨迹
        self.index = 0  # 用于表示当前指针指向新的轨迹要被插入的位置

        self.scene = scene
        self.id = id
        # 先提前把历史轨迹的点都加载好, 但是颜色都不可见

        self.visual_sphere = []
        # max_points = 1
        for i in range(max_points):
            # prim_path = f"{robot_prim_path}/traj/pos_{i}"
            # if stage.GetPrimAtPath(prim_path).IsValid():
            #     print(f"路径 {prim_path} 已存在！")
            # else:
            #     print(f"路径 {prim_path} 可用")
            self.visual_sphere.append(
                VisualSphere(
                    prim_path=f"{robot_prim_path}/traj/pos_{i}",
                    name=f"robot_{id}pos_{i}",
                    position=np.array([0.0, 0.0, 0.0], dtype=np.float32),
                    radius=radius,
                    color=np.array(color),
                    visible=False,
                )
            )

            # print(self.visual_sphere[i].prim_path)
            # self.scene.add(self.visual_sphere[i])
            self.scene.add(self.visual_sphere[i])

    def add_trajectory(self, point: list = None):
        if point is not None:
            if self.index == self.max_points:  # 达到最大的记录数量了 从头开始重新记录
                self.index = 0
            self.trajectory[self.index] = point
            self.render()  # 立即渲染 复杂度只有 1
            self.index += 1
        else:
            print("未输入point")
        return

    def render(self):
        self.visual_sphere[self.index].set_world_pose(self.trajectory[self.index])  #
        self.visual_sphere[self.index].set_visibility(True)  # 可见
