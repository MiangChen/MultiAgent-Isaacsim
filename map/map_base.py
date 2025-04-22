from typing import List
import numpy as np


class BaseMap:
    def __init__(self,
                 min_bounds: List[float],
                 max_bounds: List[float],
                 cell_size: float):
        self.min_bounds = np.array(min_bounds, dtype=np.float32)
        self.max_bounds = np.array(max_bounds, dtype=np.float32)
        self.cell_size = cell_size

        # 计算网格尺寸，xyz方向格子数量
        self.grid_size = ((self.max_bounds - self.min_bounds) / self.cell_size).astype(int)

        # 生成网格中每个体素中心坐标
        # 形状 (X, Y, Z, 3)
        x_lin = np.linspace(self.min_bounds[0] + cell_size / 2,
                            self.max_bounds[0] - cell_size / 2,
                            self.grid_size[0])
        y_lin = np.linspace(self.min_bounds[1] + cell_size / 2,
                            self.max_bounds[1] - cell_size / 2,
                            self.grid_size[1])
        z_lin = np.linspace(self.min_bounds[2] + cell_size / 2,
                            self.max_bounds[2] - cell_size / 2,
                            self.grid_size[2])
        self.pos_map = np.stack(np.meshgrid(x_lin, y_lin, z_lin, indexing='ij'), axis=-1).astype(np.float32)

    def get_voxel_index(self, point):
        """
        将世界坐标 point 转换为栅格索引 [ix, iy, iz]
        """
        idx_f = (np.array(point) - self.min_bounds) / self.cell_size
        idx = np.floor(idx_f).astype(int)
        if np.any(idx < 0) or np.any(idx >= self.grid_size):
            raise ValueError("Point outside grid bounds")
        return idx

    def _reset_generator(self):
        """子类需重写此方法初始化生成器"""
        raise NotImplementedError

    def get_cell_index(self, point: List[float]) -> tuple:
        """将世界坐标转换为栅格索引"""
        x = int((point[0] - self.min_bounds[0]) / self.cell_size)
        y = int((point[1] - self.min_bounds[1]) / self.cell_size)
        z = int((point[2] - self.min_bounds[2]) / self.cell_size)
        return (x, y, z)

    def get_world_position(self, cell_idx: tuple) -> List[float]:
        """将栅格索引转换为世界坐标"""
        x = self.min_bounds[0] + cell_idx[0] * self.cell_size
        y = self.min_bounds[1] + cell_idx[1] * self.cell_size
        z = self.min_bounds[2] + cell_idx[2] * self.cell_size
        return [x, y, z]

if __name__ == '__main__':
    semantic_map = BaseMap(min_bounds=[0,0,0], max_bounds=[1,1,1])
