import omni
import carb
from isaacsim.asset.gen.omap.bindings import _omap
import numpy as np


class GridMap():
    def __init__(self,
                 start_point: list = [0, 0, 0],
                 min_bounds: list = [-10, -10, 0],
                 max_bounds: list = [10, 10, 10],
                 cell_size: float = 0.1,
                 occupied_cell: int = 1,
                 empty_cell: int = 0,
                 invisible_cell: int = 2
                 ):
        """
        用于将一个xyz范围内的连续地图变成一个gridmap
        可以直接获取gridmap, / 快速获取障碍物的点位置
        https://docs.isaacsim.omniverse.nvidia.com/latest/py/source/extensions/isaacsim.asset.gen.omap/docs/index.html#isaacsim.asset.gen.omap.bindings._omap.Generator
        检测的原理应该是在根据某个点有没有发现 碰撞 collision单元, 仅返回该点是否有碰撞, 需要用户再根据这些点重建方格, 无法直接用于A*
        """

        self.start_point = start_point
        if min_bounds[-1] < cell_size / 2:
            min_bounds[-1] = cell_size / 2
            start_point[-1] = cell_size / 2
            print("当地面的高度低于cell size的时候, 算法会把地面全部当成障碍物, 自动将z轴的最低范围设置为cell_size")
        if max_bounds[-1] <= min_bounds[-1]:
            max_bounds[-1] = min_bounds[-1] + cell_size / 2
            print("z轴范围不足, 自动调高cell size")

        self.min_bounds = min_bounds
        self.max_bounds = max_bounds
        self.cell_size = cell_size
        self.flag_generate2d = False  # 寄存器,用于判断有没有生成过结果
        self.flag_generate3d = False
        self.occupied_cell = occupied_cell
        self.empty_cell = empty_cell
        self.invisible_cell = invisible_cell
        physx = omni.physx.acquire_physx_interface()
        if physx is None:
            print("Failed to acquire PhysX interface.")
        else:
            print("PhysX interface acquired successfully.")
        stage_id = omni.usd.get_context().get_stage_id()
        if stage_id is None:
            print("Failed to get stage ID.")
        else:
            print("Stage ID acquired successfully.")

        import carb
        self.occupied_color = carb.Int4(128, 128, 128, 255)  # 灰色，表示障碍物
        self.unoccupied_color = carb.Int4(255, 255, 255, 255)  # 白色，表示可行走区域
        self.unknown_color = carb.Int4(0, 0, 255, 255)  # 蓝色，表示不可见区域

        self.generator = _omap.Generator(physx, stage_id)
        self.reset()

        # Set location to map from and the min and max bounds to map to
        self.generator.set_transform(self.start_point, self.min_bounds, self.max_bounds)

    def generate2d(self):
        if self.flag_generate2d == True:
            return
        else:
            self.flag_generate2d = True
            self.generator.generate2d()

    def generate3d(self):
        if self.flag_generate3d == False:
            self.flag_generate3d = True
            self.generator.generate3d()
        return

    def reset(self):
        self.generator.update_settings(
            self.cell_size,
            self.occupied_cell,  # occupied cells
            self.empty_cell,  # unoccupied
            self.invisible_cell,  # cannot be seen
        )
        return

    def map_2d(self):
        """
        通过2d矩阵数值来表示每个点是障碍物还是空地
        同时给出一个对应的2d矩阵, 用于表示上面的矩阵的点对应的现实坐标
        :return:
        """
        if self.flag_generate2d == True:
            obs_position = self.generator.get_occupied_positions()  # 只是一个list,表示obs的坐标
            free_position = self.generator.get_free_positions()  # 只是一个list, 表示空地的坐标
            value_map = self.generator.get_buffer()  # list, 表示各个点是障碍物还是空地, 但是没有坐标信息

        x, y, z = self.generator.get_dimensions()
        self.pos_map = np.empty((x, y, 3), dtype=np.float32)

        self.value_map = np.array(value_map).reshape((x, y))
        index_obs = 0
        index_free = 0
        for i in range(0, x):
            for j in range(0, y):

                if self.value_map[i][j] == self.occupied_cell:  # 障碍物
                    self.pos_map[i][j] = obs_position[index_obs]
                    index_obs += 1
                elif self.value_map[i][j] == self.empty_cell or self.value_map[i][j] == self.invisible_cell:
                    self.pos_map[i][j] = free_position[index_free]
                    index_free += 1
                else:
                    print("special occasion, check manually")

        return self.pos_map, self.value_map

    def get_image(self):
        colored_buffer = self.generator.get_colored_byte_buffer(
            self.occupied_color,
            self.unoccupied_color,
            self.unknown_color
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


if __name__ == "__main__":
    grid_map = GridMap(min_bounds=[-10, -10, 0], max_bounds=[10, 10, 10], cell_size=0.5)

    grid_map.generate2d()
    point = grid_map.generator.get_occupied_positions()
    point2 = grid_map.generator.get_free_positions()
    print(point[:10])
    print(point2[:10])
    print(len(point)+len(point2))
    #print(point)
    #print("len point", len(point))

    buffer = grid_map.generator.get_buffer()
    print(buffer)
    # 创建并保存图像
    # 创建并保存图像
    #image = grid_map.get_image()
    #image.save("occupancy_map.png")

    #image.save("/home/ubuntu/Pictures/occupancy_map_3d_2.png")
    grid_map.map_2d()