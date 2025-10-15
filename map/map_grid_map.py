from typing import List, Optional, Tuple

import numpy as np

from isaacsim.asset.gen.omap.bindings import _omap
import omni


class GridMap:
    """
    Grid map generator for OMPL path planning.
    Converts continuous 3D space into discrete grid representation.
    """
    
    def __init__(self,
                 cell_size: float = 1.0,
                 start_point: list = [0, 0, 0],
                 min_bounds: List[float] = [-20, -20, 0],
                 max_bounds: List[float] = [20, 20, 5],
                 occupied_value: int = 100,
                 free_value: int = 0,
                 unknown_value: int = -1,):
        """
        Initialize GridMap for path planning.
        
        Args:
            start_point: Should be empty
            cell_size: Size of each grid cell in meters
            min_bounds: [x_min, y_min, z_min] bounds of the map
            max_bounds: [x_max, y_max, z_max] bounds of the map  
            occupied_value: Value for occupied cells (obstacles)
            free_value: Value for free cells (navigable space)
        """
        self.cell_size = cell_size
        self.start_point = np.array(start_point, dtype=np.float32)
        self.min_bounds = np.array(min_bounds, dtype=np.float32)
        self.max_bounds = np.array(max_bounds, dtype=np.float32)
            
        self.occupied_value = occupied_value
        self.free_value = free_value
        self.unknown_value = unknown_value
        
        # Internal state
        self._is_2d_generated = False
        self._is_3d_generated = False
        self.value_map: Optional[np.ndarray] = None
        self.generator: Optional[_omap.Generator] = None

    def initialize(self) -> None:
        """
        Initialize the grid map generator. Must be called after world reset.
        """
        physx = omni.physx.acquire_physx_interface()
        stage_id = omni.usd.get_context().get_stage_id()
        self.generator = _omap.Generator(physx, stage_id)

        self.generator.update_settings(
            self.cell_size,
            self.occupied_value,
            self.free_value,
            self.unknown_value
        )

        self.generator.set_transform(self.start_point, self.min_bounds, self.max_bounds)


    def generate(self, dimension: str = '2d') -> np.ndarray:
        """
        Generate grid map from the current scene.
        
        Args:
            dimension: '2d' or '3d' map generation
            
        Returns:
            Tuple of (position_map, value_map)
            - position_map: Real-world coordinates for each grid cell
            - value_map: Occupancy values (0=free, 1=occupied)
        """
        if not self.generator:
            raise RuntimeError("GridMap not initialized. Call initialize() first.")
            
        # Generate map if not already done
        if dimension == '2d' and not self._is_2d_generated:
            self.generator.generate2d()
            self._is_2d_generated = True
            self._is_3d_generated = False
        elif dimension == '3d' and not self._is_3d_generated:
            self.generator.generate3d()
            self._is_3d_generated = True
            self._is_2d_generated = False
        elif dimension not in ['2d', '3d']:
            raise ValueError(f"Invalid dimension: {dimension}. Use '2d' or '3d'.")
            
        # Get occupied and free positions
        occupied_positions = self.generator.get_occupied_positions()
        
        # Initialize maps
        x, y, z = self.generator.get_dimensions()
        self.value_map = np.full((x, y, z), self.free_value, dtype=np.uint8)
        
        # Convert positions to grid indices and populate maps
        if occupied_positions:
            occupied_indices = self.compute_index(occupied_positions)
            if occupied_indices is not None:
                self.value_map[tuple(occupied_indices.T)] = self.occupied_value

                
        return self.value_map

    def is_valid_position(self, position: List[float]) -> bool:
        """
        Check if a position is within map bounds.
        
        Args:
            position: [x, y, z] coordinates
            
        Returns:
            True if position is within bounds
        """
        return all(self.min_bounds[i] <= position[i] <= self.max_bounds[i] 
                  for i in range(len(position)))
    
    def is_occupied(self, position: List[float]) -> bool:
        """
        Check if a position is occupied (obstacle).
        
        Args:
            position: [x, y, z] coordinates
            
        Returns:
            True if position is occupied
        """
        if not self.is_valid_position(position):
            return True  # Out of bounds considered occupied
            
        indices = self.compute_index([position])
        if indices is None:
            return True
            
        idx = tuple(indices[0])
        return self.value_map[idx] == self.occupied_value
    
    def get_map_info(self) -> dict:
        """
        Get map metadata for OMPL planning.
        
        Returns:
            Dictionary with map information
        """
        if self.value_map is None:
            raise RuntimeError("Map not generated. Call generate() first.")
            
        return {
            'dimensions': self.value_map.shape,
            'cell_size': self.cell_size,
            'min_bounds': self.min_bounds.tolist(),
            'max_bounds': self.max_bounds.tolist(),
            'occupied_value': self.occupied_value,
            'free_value': self.free_value
        }

    def compute_index(self, positions) -> Optional[np.ndarray]:
        """
        Convert world coordinates to grid indices.
        
        Args:
            positions: List of [x, y, z] coordinates or single position
            
        Returns:
            Array of grid indices or None if invalid input
        """
        if positions is None or len(positions) == 0:
            return None
            
        if not self.generator:
            raise RuntimeError("GridMap not initialized.")
            
        # Get grid bounds
        min_bound = np.array(self.generator.get_min_bound(), dtype=np.float32)
        positions = np.array(positions, dtype=np.float32)
        
        # Convert to grid indices
        indices = ((positions - min_bound) / self.cell_size).astype(int)
        
        return indices



# Example usage for OMPL path planning
if __name__ == "__main__":
    # Create grid map
    grid_map = GridMap(
        cell_size=0.5,
        min_bounds=[-10, -10, 0],
        max_bounds=[10, 10, 5]
    )
    
    # Initialize (must be called after world reset)
    grid_map.initialize()
    
    # Generate 2D map for ground robots
    pos_map, value_map = grid_map.generate('2d')
    
    # Check if position is valid for planning
    start_pos = [0, 0, 0]
    goal_pos = [5, 5, 0]
    
    print(f"Start position valid: {grid_map.is_valid_position(start_pos)}")
    print(f"Goal position occupied: {grid_map.is_occupied(goal_pos)}")
    print(f"Map info: {grid_map.get_map_info()}")