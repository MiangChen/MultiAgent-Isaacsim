from typing import List, Optional, Tuple

import numpy as np

from isaacsim.asset.gen.omap.bindings import _omap
import omni


class GridMap:
    """
    Grid map generator for OMPL path planning.
    Converts continuous 3D space into discrete grid representation.
    Includes height filtering to remove ground obstacles.
    """
    
    def __init__(self,
                 cell_size: float = 1.0,
                 start_point: list = [0, 0, 0],
                 min_bounds: List[float] = [-20, -20, 0],
                 max_bounds: List[float] = [20, 20, 5],
                 occupied_value: int = 100,
                 free_value: int = 0,
                 unknown_value: int = -1):
        """
        Initialize GridMap for path planning.
        
        Args:
            start_point: Reference point for map generation
            cell_size: Size of each grid cell in meters
            min_bounds: [x_min, y_min, z_min] bounds of the map
            max_bounds: [x_max, y_max, z_max] bounds of the map  
            occupied_value: Value for occupied cells (obstacles)
            free_value: Value for free cells (navigable space)
            unknown_value: Value for unknown cells
        """
        self.cell_size = cell_size
        self.start_point = np.array(start_point, dtype=np.float32)
        self.min_bounds = np.array(min_bounds, dtype=np.float32)
        self.max_bounds = np.array(max_bounds, dtype=np.float32)
            
        self.occupied_value = occupied_value
        self.free_value = free_value
        self.unknown_value = unknown_value
        
        # Internal state
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
            float(self.occupied_value),
            float(self.free_value),
            float(self.unknown_value)
        )

        self.generator.set_transform(self.start_point, self.min_bounds, self.max_bounds)


    def generate(self, ground_height: float = 0.0, ground_tolerance: float = 0.2) -> np.ndarray:
        """
        Generate 3D grid map with height-based ground filtering.
        
        Args:
            ground_height: Expected ground level height
            ground_tolerance: Height tolerance for ground detection
            
        Returns:
            The 3D value_map (occupancy grid) as a numpy array
        """
        if not self.generator:
            raise RuntimeError("GridMap not initialized. Call initialize() first.")
            
        # Generate 3D map
        self.generator.generate3d()
        
        # Get all occupied positions in 3D world coordinates
        occupied_positions = self.generator.get_occupied_positions()
        x_dim, y_dim, z_dim = self.generator.get_dimensions()
        
        # Initialize 3D map
        self.value_map = np.full((x_dim, y_dim, z_dim), self.free_value, dtype=np.uint8)
        
        # Populate map with all obstacles first
        if occupied_positions is not None and len(occupied_positions) > 0:
            occupied_indices = self.compute_index(occupied_positions)
            if occupied_indices is not None:
                self.value_map[tuple(occupied_indices.T)] = self.occupied_value
        
        # Apply height-based ground filtering
        self._remove_ground_obstacles(ground_height, ground_tolerance)
            
        return self.value_map

    def _remove_ground_obstacles(self, ground_height: float, tolerance: float) -> None:
        """
        Remove ground obstacles while preserving building bases.

        Args:
            ground_height: Expected ground level height
            tolerance: Height tolerance for ground detection
        """
        if self.value_map is None:
            return

        # Ensure parameters are float
        ground_height = float(ground_height)
        tolerance = float(tolerance)
        effective_tolerance = max(tolerance, self.cell_size / 2)

        x_dim, y_dim, z_dim = self.value_map.shape

        # Calculate how many z layers to check based on cell_size
        # Smaller cell_size -> check more layers
        max_ground_layers = max(1, int((effective_tolerance * 2) / self.cell_size))
        max_ground_layers = min(max_ground_layers, z_dim)

        for x in range(x_dim):
            for y in range(y_dim):
                for z in range(max_ground_layers):
                    if self.value_map[x, y, z] == self.occupied_value:
                        # Get world coordinates for this grid cell
                        min_bound = np.array(self.generator.get_min_bound(), dtype=np.float32)
                        world_pos = min_bound + np.array([x, y, z], dtype=np.float32) * self.cell_size
                        actual_height = float(world_pos[2])

                        # Check if this cell is within ground height range
                        if abs(actual_height - ground_height) <= effective_tolerance:
                            # Check if there are obstacles above this position
                            # If the layer above (z+1) is empty, this is likely ground
                            check_z = z + 1
                            if check_z < z_dim:
                                is_empty_above = (self.value_map[x, y, check_z] == self.free_value)
                            else:
                                # If we're at the top layer, consider it empty above
                                is_empty_above = True

                            # Only remove if it's empty above (indicating ground, not building base)
                            if is_empty_above:
                                self.value_map[x, y, z] = self.free_value

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
    
    # Generate 2D map for ground robots with height filtering
    value_map = grid_map.generate('2d', min_height=0.1, max_height=2.0)
    
    # Generate 3D map for aerial robots
    # value_map_3d = grid_map.generate('3d')
    
    # Check if position is valid for planning
    start_pos = [0, 0, 0]
    goal_pos = [5, 5, 0]
    
    print(f"Start position valid: {grid_map.is_valid_position(start_pos)}")
    print(f"Goal position occupied: {grid_map.is_occupied(goal_pos)}")
    print(f"Map info: {grid_map.get_map_info()}")
    print(f"Map shape: {value_map.shape}")