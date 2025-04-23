import numpy as np
import heapq
from typing import Tuple, List, Optional


class AStar:
    def __init__(self,
                 map_3d: np.ndarray,
                 obs_value: float = 1.0,
                 free_value: float = 0.0,
                 directions: str = 'eight'):
        """
        Initialize the A* path planner.

        Args:
            map_3d: 3D numpy array representing the environment
            obs_value: Value that represents obstacles in the map
            free_value: Value that represents free space in the map
            directions: List of possible movement directions [dx, dy, dz]
        """
        self.map = map_3d
        self.obs_value = obs_value
        self.free_value = free_value

        self.x_size, self.y_size, self.z_size = map_3d.shape

        if directions == 'eight':
            self.directions = [
                [1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0],
                [1, 1, 0], [1, -1, 0], [-1, 1, 0], [-1, -1, 0],
            ]
        elif directions == 'four':
            self.directions = [
                [1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0]
            ]
        else:
            raise ValueError("not support directions")

        # Check if directions are valid (non-zero)
        for direction in self.directions:
            if direction == [0, 0, 0]:
                raise ValueError("Direction [0, 0, 0] is not allowed")

    def _heuristic(self, a: Tuple[int, int, int], b: Tuple[int, int, int]) -> float:
        """
        Calculate the heuristic cost between two points (Euclidean distance).

        Args:
            a: First point (x, y, z)
            b: Second point (x, y, z)

        Returns:
            Euclidean distance between the points
        """
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        dz = abs(a[2] - b[2])
        return np.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

    def _is_valid(self, point: Tuple[int, int, int]) -> bool:
        """
        Check if a point is within map bounds and not an obstacle.

        Args:
            point: (x, y, z) coordinates to check

        Returns:
            True if valid, False otherwise
        """
        x, y, z = point
        return (0 <= x < self.x_size and
                0 <= y < self.y_size and
                0 <= z < self.z_size and
                self.map[x, y, z] == self.free_value)

    def find_path(self,
                  start: Tuple[int, int, int],
                  goal: Tuple[int, int, int]) -> Optional[List[Tuple[int, int, int]]]:
        """
        Find a path from start to goal using A* algorithm.

        Args:
            start: Starting point (x, y, z)
            goal: Goal point (x, y, z)

        Returns:
            List of points representing the path from start to goal, or None if no path exists
        """
        # Check if start and goal are valid
        if not self._is_valid(start):
            raise ValueError("Start point is invalid (out of bounds or obstacle)")
        if not self._is_valid(goal):
            raise ValueError("Goal point is invalid (out of bounds or obstacle)")

        # Priority queue: (f_score, g_score, current_node)
        open_set = []
        heapq.heappush(open_set, (0, 0, start))

        # Dictionaries to keep track of the path and costs
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self._heuristic(start, goal)}

        # Nodes already evaluated
        closed_set = set()

        while open_set:
            _, current_g, current = heapq.heappop(open_set)

            if current == goal:
                # Reconstruct path
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return np.array(path)

            if current in closed_set:
                continue

            closed_set.add(current)

            for direction in self.directions:
                neighbor = (current[0] + direction[0],
                            current[1] + direction[1],
                            current[2] + direction[2])

                if not self._is_valid(neighbor):
                    continue

                # Calculate tentative g_score
                # For simplicity, we use Euclidean distance for step cost
                step_cost = np.sqrt(direction[0] ** 2 + direction[1] ** 2 + direction[2] ** 2)
                tentative_g = current_g + step_cost

                if neighbor in closed_set and tentative_g >= g_score.get(neighbor, float('inf')):
                    continue

                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self._heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], g_score[neighbor], neighbor))

        # No path found
        return None


# Example usage:
if __name__ == "__main__":
    # Create a simple 3D map (10x10x10)
    map_3d = np.zeros((10, 10, 10))

    # Add some obstacles
    map_3d[5, 4:6, 0] = 1  # Vertical obstacle
    map_3d[2:8, 3, 4] = 1  # Horizontal obstacle

    # 2d movement for car
    directions = [
        [1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0],
        [1, 1, 0], [1, -1, 0], [-1, 1, 0], [-1, -1, 0],
    ]
    # 3d movement for drone
    # directions = [
    #     [1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1],
    #     [1, 1, 0], [1, -1, 0], [-1, 1, 0], [-1, -1, 0],
    #     [1, 0, 1], [1, 0, -1], [-1, 0, 1], [-1, 0, -1],
    #     [0, 1, 1], [0, 1, -1], [0, -1, 1], [0, -1, -1],
    #     [1, 1, 1], [1, 1, -1], [1, -1, 1], [1, -1, -1],
    #     [-1, 1, 1], [-1, 1, -1], [-1, -1, 1], [-1, -1, -1]
    # ]

    # Initialize A* planner
    planner = AStar(map_3d, obs_value=1.0, free_value=0.0, directions=directions)

    # Define start and goal points
    start = (0, 0, 0)
    goal = (9, 9, 0)

    # Find path
    path = planner.find_path(start, goal)

    if path is not None:
        print("Path found with", len(path), "steps:")
        for i, point in enumerate(path):
            print(f"Step {i}: {point}")
    else:
        print("No path found")
