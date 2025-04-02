import omni
from isaacsim.asset.gen.omap.bindings import _omap


class GridMap(omni):
    def __init__(self, start_point, min_bounds, max_bounds, cell_size: float = 0.1):
        """
        用于将一个xyz范围内的连续地图变成一个gridmap
        可以直接获取gridmap, / 快速获取障碍物的点位置
        """

        self.start_point = start_point
        self.min_bounds = min_bounds
        self.max_bounds = max_bounds
        self.cell_size = cell_size
        self.flag_generate2d = False  # 寄存器,用于判断有没有生成过结果
        self.flag_generate3d = False
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

        self.generator = _omap.Generator(physx, stage_id)
        # 0.05m cell size, output buffer will have 1 for occupied cells, 0 for unoccupied, and 2 for cells that cannot be seen
        # this assumes your usd stage units are in m, and not cm
        self.generator.update_settings(
            self.cell_size, 1, 0, 2)
        # Set location to map from and the min and max bounds to map to
        self.generator.set_transform(self.start_point, self.min_bounds, self.max_bounds)

        # Get locations of the occupied cells in the stage
        points = self.generator.get_occupied_positions()

        buffer = self.generator.get_buffer()
        print(buffer)
        # Get dimensions for 2d buffer
        dims = self.generator.get_dimensions()
        print(dims)

    def generate2d(self):
        if self.flag_generate2d == True:
            return
        else:
            self.flag_generate2d = True
            self.generator.generate2d()

    def generate3d(self):
        self.generator.generate3d()
