import logging
from typing import List

import carb
import numpy as np

import omni
from physics_engine.isaacsim_utils import _omap, VisualCuboid, stage_utils


class GridMap:
    def __init__(
        self,
        cell_size: float = 1,
        start_point: list = [0, 0, 0],
        min_bounds: list = [-20, -20, 0],
        max_bounds: list = [20, 20, 5],
        occupied_cell: int = 1,
        empty_cell: int = 0,
        invisible_cell: int = 2,
    ):
        """
        用于将一个xyz范围内的连续地图变成一个gridmap
        可以直接获取gridmap, / 快速获取障碍物的点位置
        https://docs.isaacsim.omniverse.nvidia.com/latest/py/source/extensions/isaacsim.asset.gen.omap/docs/index.html#isaacsim.asset.gen.omap.bindings._omap.Generator
        检测的原理应该是在根据某个点有没有发现 碰撞 collision单元, 仅返回该点是否有碰撞, 需要用户再根据这些点重建方格, 无法直接用于A*
        """

        self.method = 1
        # 因为地面是有collision属性的, 直接重建会把地面也当做障碍物, 导致整个地图都是障碍物, 为了解决这个问题, 有两个方式
        # 方式1: 把重建的z轴高度提高一些, 经过测试, 最少要提高 cell_size /2
        self.start_point = start_point
        if self.method == 1:
            if min_bounds[-1] < cell_size / 2:
                min_bounds[-1] = cell_size / 2
                start_point[-1] = cell_size / 2
                print(
                    "当地面的高度低于cell size的时候, 算法会把地面全部当成障碍物, 自动将z轴的最低范围设置为cell_size"
                )
            if max_bounds[-1] <= min_bounds[-1]:
                max_bounds[-1] = min_bounds[-1] + cell_size / 2
                print("z轴范围不足, 自动调高cell size")

        self.min_bounds = np.array(min_bounds)
        self.max_bounds = np.array(max_bounds)
        self.cell_size = cell_size
        self.flag_generate2d = False  # 寄存器,用于判断有没有生成过结果
        self.flag_generate3d = False
        self.occupied_cell = occupied_cell
        self.empty_cell = empty_cell
        self.invisible_cell = invisible_cell
        self.path_robot = (
            "/World/robot"  # 记录机器人的统一路径,后续要先deactivate再建立gridmap
        )
        self.path_ground = "/World/GroundPlane"
        self.occupied_color = carb.Int4(128, 128, 128, 255)  # 灰色，表示障碍物
        self.unoccupied_color = carb.Int4(255, 255, 255, 255)  # 白色，表示可行走区域
        self.unknown_color = carb.Int4(0, 0, 255, 255)  # 蓝色，表示不可见区域

    def initialize(self):
        """
        这个函数必须再world reset后使用
        Returns:

        """
        physx = omni.physx.acquire_physx_interface()
        stage_id = (
            omni.usd.get_context().get_stage_id()
        )  # 这里要改进一下, 只对静态的场景进行grid world建模, 对于动态的机器人建立grid map是在别的地方来处理的
        self.generator = _omap.Generator(physx, stage_id)
        self.reset()

        # Set location to map from and the min and max bounds to map
        self.generator.set_transform(self.start_point, self.min_bounds, self.max_bounds)

    def generate_grid_map(self, dimension: str = "2d"):
        """
        首先重建地图, 2d和3d只能选择一个状态
        :param dimension:
        :return:
        """
        if (
            dimension == "2d"
            and self.flag_generate2d == False
            or dimension == "3d"
            and self.flag_generate3d == False
        ):
            # 给静态场景建图
            if dimension == "2d":
                self.generator.generate2d()
                self.flag_generate2d = True
                self.flag_generate3d = False
                print("generate 2d")
            elif dimension == "3d":
                self.generator.generate3d()
                self.flag_generate3d = True
                self.flag_generate2d = False
                print("generate 3d")
            else:
                print(f"Invalid dimension: {dimension}")

        """
        通过2d/3d矩阵数值来表示每个点是障碍物还是空地
        同时给出一个对应的2d矩阵, 用于表示上面的矩阵的点对应的现实坐标
        :return:
        """
        obs_position = (
            self.generator.get_occupied_positions()
        )  # 只是一个list,表示obs的坐标
        free_position = (
            self.generator.get_free_positions()
        )  # 只是一个list, 表示空地的坐标
        # value_map = self.generator.get_buffer()  # list, 表示各个点是障碍物还是空地, 但是没有坐标信息
        print(
            "len obs",
            len(obs_position),
            "len free ",
            len(free_position),
            "all",
            len(obs_position) + len(free_position),
        )
        # print("len value map", len(value_map))
        x, y, z = self.generator.get_dimensions()
        self.pos_map = np.empty(
            (x, y, z, 3), dtype=np.float16
        )  # 2d map会自动z=1, 无关紧要
        self.value_map = np.empty((x, y, z), dtype=np.uint8)
        # self.semantic_map = np.full((x, y, z), fill_value=-1, dtype=np.int32)   # 语义地图
        # self.value_map = np.array(value_map).reshape((x, y, z))
        # index_obs = 0
        # index_free = 0
        # max_b = np.array(max_b, dtype=np.float32)
        # 计算索引
        # obs_indices = (obs_position / self.cell_size - min_b / self.cell_size).astype(int)

        # free_indices = (free_position / self.cell_size - min_b / self.cell_size).astype(int)
        obs_indices = self.compute_index(obs_position)
        free_indices = self.compute_index(free_position)
        # 使用高级索引更新 self.value_map 和 self.pos_map
        if obs_indices is not None:
            self.value_map[tuple(obs_indices.T)] = self.occupied_cell
            self.pos_map[tuple(obs_indices.T)] = obs_position
        if free_indices is not None:
            self.value_map[tuple(free_indices.T)] = self.empty_cell
            self.pos_map[tuple(free_indices.T)] = free_position
        # for p in obs_position:
        #     x = int((p[0] / scale - min_b[0] / scale))
        #     y = int((p[1] / scale - min_b[1] / scale))
        #     z = int((p[2] / scale - min_b[2] / scale))
        #     self.value_map[x, y, z] = self.occupied_cell
        #     self.pos_map[x, y, z] = p
        # for p in free_position:
        #     x = int((p[0] / scale - min_b[0] / scale))
        #     y = int((p[1] / scale - min_b[1] / scale))
        #     z = int((p[2] / scale - min_b[2] / scale))
        #     self.value_map[x, y, z] = self.empty_cell
        #     self.pos_map[x, y, z] = p

        # int(p[1] / scale - min_b[1] / scale) * int(size[0] / scale) + int(p[0] / scale - min_b[0] / scale)
        # print(min_corner)
        # for i in range(0, x):
        #     for j in range(0, y):
        #         for k in range(0, z):
        #             if self.value_map[i][j][k] == self.occupied_cell:  # 障碍物
        #                 self.pos_map[i][j][k] = obs_position[index_obs]
        #                 index_obs += 1
        #             elif self.value_map[i][j][k] == self.empty_cell or self.value_map[i][j][k] == self.invisible_cell:
        #                 self.pos_map[i][j][k] = free_position[index_free]
        #                 index_free += 1
        #             else:
        #                 print("special occasion, check manually")

        return self.pos_map, self.value_map

    def continuous_to_grid(self, continuous_pos: List[float]):
        """
        这里将地图里的真实坐标, 变成grid map生成的matrix里的坐标, 并不是说把原来世界给变成gridmap后的坐标;
        同一个真实坐标, 同一个cell size 返回的坐标可能因为min bound而不同

        """
        # 先检测可行性
        if any(
            continuous_pos[i] < self.min_bounds[i]
            or continuous_pos[i] > self.max_bounds[i]
            for i in range(3)
        ):
            print(
                "输入的坐标范围超过了 min bounds和 max bounds, 没有实用意义, 需要重新设置"
            )

    def reset(self):
        self.generator.update_settings(
            self.cell_size,
            self.occupied_cell,  # occupied cells
            self.empty_cell,  # unoccupied
            self.invisible_cell,  # cannot be seen
        )
        return

    def get_image(self):
        colored_buffer = self.generator.get_colored_byte_buffer(
            self.occupied_color, self.unoccupied_color, self.unknown_color
        )
        import numpy as np

        # 将缓冲区转换为 numpy 数组
        buffer_np = np.array([ord(byte) for byte in colored_buffer], dtype=np.uint8)
        # 假设图像的宽度和高度由占据图的尺寸（dims）决定
        x, y, z = self.generator.get_dimensions()

        from PIL import Image

        # 将一维的缓冲区转换为二维图像
        buffer_np = buffer_np.reshape((x, y, 4))  # 每个像素有 RGBA 值
        image = Image.fromarray(buffer_np)
        return image

    def compute_index(self, position: List = None) -> np.ndarray:
        if (
            position is None
            or (isinstance(position, np.ndarray) and position.size == 0)
            or isinstance(position, List)
            and len(position) == 0
        ):
            logging.warning("compute index的输入position = None")
            return None
        # 获取角点
        max_b = self.generator.get_max_bound()
        min_b = self.generator.get_min_bound()  # 返回的是grid map 存在的 最小的角落点
        # 获取三个维度上的长度情况
        size = [0, 0, 0]
        size[0] = max_b[0] - min_b[0]
        size[1] = max_b[1] - min_b[1]
        size[2] = max_b[2] - min_b[2]

        # 处理为numpy, 加速计算
        position = np.array(position, dtype=np.float32)  # 确保数据类型一致，提高效率
        min_b = np.array(min_b, dtype=np.float32)

        return (position / self.cell_size - min_b / self.cell_size).astype(int)


if __name__ == "__main__":
    """
    记得 先 Play 运行环境, 才能跑起来
    """
    cell_size = 0.2
    grid_map = GridMap(
        min_bounds=[-10, -10, 0], max_bounds=[10, 10, 10], cell_size=cell_size
    )

    grid_map.generate_grid_map("2d")
    point = grid_map.generator.get_occupied_positions()
    point2 = grid_map.generator.get_free_positions()
    print(point[:10])
    print(point2[:10])
    print(len(point) + len(point2))
    # print(point)
    # print("len point", len(point))

    # buffer = grid_map.generator.get_buffer()
    # print(buffer)
    # 创建并保存图像
    # 创建并保存图像
    # image = grid_map.get_image()
    # image.save("occupancy_map.png")

    # image.save("/home/ubuntu/Pictures/occupancy_map_3d_2.png")
    # grid_map.map_2d()

    index = 0
    # 检查一下是否匹配, 障碍物再value map中的index和pos map中的是否一致

    # 下面的这个比较耗时间 , 3d的0.25精度, 渲染时间2分钟
    k = 0.99
    for x in range(grid_map.value_map.shape[0]):
        for y in range(grid_map.value_map.shape[1]):
            for z in range(grid_map.value_map.shape[2]):
                if grid_map.value_map[x][y][z] == grid_map.occupied_cell:
                    # print(
                    #     f"坐标 {x, y, z}, value map = {grid_map.value_map[x][y][z]}, pos map = {grid_map.pos_map[x][y][z]}")
                    # 在对应的pos位置创建一个方块, 边长就是cell
                    index += 1
                    cube_path = f"/World/grid/cube{index}"
                    pos = grid_map.pos_map[x][y][z]
                    pos[1] -= 6
                    cube = VisualCuboid(
                        prim_path=cube_path,
                        name=f"cube{index}",
                        position=pos,
                        size=cell_size,
                        color=np.array([0.0, 0.5 * k, 0.0], dtype=np.float32),
                    )
                    k *= 0.99

    stage = stage_utils.get_current_stage()
    prim_robot = stage.GetPrimAtPath(grid_map.path_robot)
    # print(dir(prim_robot))
    # print(prim_robot.get_world_pose())
    # property = stage.GetProperty(grid_map.path_robot)
    # print(prim_robot.GetAttribute())

    from pxr import UsdGeom

    # 获取 prim
    prim_robot = stage.GetPrimAtPath(grid_map.path_robot)

    # 检查是否是 Xformable（可变换对象）
    if prim_robot.IsA(UsdGeom.Xformable):
        xform = UsdGeom.Xformable(prim_robot)
        local_transform = xform.GetLocalTransformation()  # 返回 Gf.Matrix4d
        local_position = local_transform.ExtractTranslation()  # 提取平移部分
        print("Local Position:", local_position)
        index = grid_map.compute_index(local_position)
        print(index)
        print(grid_map.pos_map.shape)
        pos_grid_map = grid_map.pos_map[47, 50, 0, :]
        print("pos_grid_map:", pos_grid_map)
        # print(pos_grid_map[index])
    else:
        print("Prim is not transformable (not a UsdGeom.Xformable)")
