import heapq
import numpy as np
from typing import Tuple, List, Optional
import collections # For BFS queue

class AStar:
    def __init__(self,
                 map_3d: np.ndarray,
                 penalty_factor: float = 100.0, # 新增：惩罚系数，控制远离障碍物的倾向
                 obs_value: float = 1.0,
                 free_value: float = 0.0,
                 directions: str = 'eight'):
        """
        Initialize the A* path planner with cost based on distance to obstacles.

        Args:
            map_3d: 3D numpy array representing the environment
            penalty_factor: Multiplier for the obstacle avoidance cost. Higher value means stronger avoidance.
            obs_value: Value that represents obstacles in the map
            free_value: Value that represents free space in the map
            directions: 'four', 'eight' (2D, Z=0), '6' (3D axis-aligned), or '26' (3D full)
        """
        self.map = map_3d # Store the original map
        self.obs_value = obs_value
        self.free_value = free_value
        self.penalty_factor = penalty_factor

        self.x_size, self.y_size, self.z_size = map_3d.shape

        # Define movement directions
        if directions == 'eight':
            self.directions = [
                (1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0),
                (1, 1, 0), (1, -1, 0), (-1, 1, 0), (-1, -1, 0),
            ]
        elif directions == 'four':
            self.directions = [
                (1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0)
            ]
        elif directions == '26': # Example for full 3D directions
             self.directions = [
                (dx, dy, dz) for dx in [-1, 0, 1] for dy in [-1, 0, 1] for dz in [-1, 0, 1] if not (dx == 0 and dy == 0 and dz == 0)
             ]
        elif directions == '6': # Example for axis-aligned 3D directions
             self.directions = [
                 (1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)
             ]
        else:
            raise ValueError(f"Unsupported directions: {directions}. Choose 'four', 'eight', '6', or '26'.")

        # Ensure directions are valid (non-zero) - already handled by the logic above

        # Pre-calculate the distance map from each free cell to the nearest obstacle
        self.distance_map = self._calculate_distance_map()


    def _calculate_distance_map(self) -> np.ndarray:
        """
        Calculates the grid distance (Chebyshev distance) from each free cell
        to the nearest obstacle cell using BFS.
        """
        print("Calculating distance map to nearest obstacles...")
        x_size, y_size, z_size = self.map.shape

        # Distance map initialized to infinity, 0 for obstacles
        distance_map = np.full_like(self.map, fill_value=np.inf, dtype=float)
        queue = collections.deque()

        # Initialize queue and distance map with obstacle locations (distance 0)
        obstacle_locations = np.argwhere(self.map == self.obs_value)
        # Use 26 directions for distance calculation propagation
        dist_directions = [(dx, dy, dz) for dx in [-1, 0, 1] for dy in [-1, 0, 1] for dz in [-1, 0, 1] if not (dx == 0 and dy == 0 and dz == 0)]

        for obs_loc in obstacle_locations:
            x, y, z = tuple(obs_loc)
            if distance_map[x, y, z] == np.inf: # Avoid processing if already added
                 distance_map[x, y, z] = 0
                 # Add all neighbors to the queue to start the BFS propagation
                 for d in dist_directions:
                     nx, ny, nz = x + d[0], y + d[1], z + d[2]
                     if 0 <= nx < x_size and 0 <= ny < y_size and 0 <= nz < z_size:
                         # Only add neighbors if their distance is still inf
                         if distance_map[nx, ny, nz] == np.inf:
                            # Step cost is 1 for grid distance (Chebyshev) between adjacent cells
                            # distance_map[nx, ny, nz] = 1 # Distance is 1 from obstacle
                            queue.append((nx, ny, nz))
                            # No, add neighbor to queue *first* with distance inf, process when popped
                            # Better: Initialize non-obstacle neighbors distance as 1 and queue them initially
                            # Let's re-do the BFS initialization slightly for clarity:

        # Re-initialize queue with *free neighbors* of obstacles
        queue = collections.deque()
        # Use a visited set for the BFS itself if needed, but updating distance_map is sufficient

        # Find initial cells for BFS: free cells adjacent to obstacles
        for obs_loc in obstacle_locations:
             x, y, z = tuple(obs_loc)
             for d in dist_directions:
                 nx, ny, nz = x + d[0], y + d[1], z + d[2]
                 if (0 <= nx < x_size and 0 <= ny < y_size and 0 <= nz < z_size and
                     self.map[nx, ny, nz] == self.free_value and # Ensure it's free space
                     distance_map[nx, ny, nz] == np.inf): # Ensure not already added/processed
                     distance_map[nx, ny, nz] = 1 # Distance is 1 step from the obstacle
                     queue.append((nx, ny, nz))

        # Perform BFS
        while queue:
            x, y, z = queue.popleft()
            current_dist = distance_map[x, y, z]

            # Propagate distance to its free neighbors
            for d in dist_directions:
                nx, ny, nz = x + d[0], y + d[1], z + d[2]

                if (0 <= nx < x_size and 0 <= ny < y_size and 0 <= nz < z_size and
                    self.map[nx, ny, nz] == self.free_value): # Only propagate through free space

                    step_cost = 1 # Chebyshev distance step cost

                    if distance_map[nx, ny, nz] > current_dist + step_cost:
                         distance_map[nx, ny, nz] = current_dist + step_cost
                         queue.append((nx, ny, nz))

        print("Distance map calculation finished.")
        # For free cells far from obstacles, distance_map might remain inf. This is okay.
        return distance_map


    def _heuristic(self, a: Tuple[int, int, int], b: Tuple[int, int, int]) -> float:
        """
        Calculate the heuristic cost between two points (Euclidean distance).
        """
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        dz = abs(a[2] - b[2])
        # return np.sqrt(dx ** 2 + dy ** 2 + dz ** 2) # Euclidean
        # Often, using the same distance metric for heuristic as for step cost (excluding penalty)
        # makes the heuristic admissible. Here step cost is Euclidean.
        # The heuristic must be admissible (never overestimate) to guarantee optimality
        # of the *path in terms of g-score*. With added penalty, the "optimal" path
        # minimizes g+penalty+h, not just g+h. Euclidean is admissible for Euclidean step cost.
        # Let's keep Euclidean heuristic as it's generally better unless step cost is pure grid.
        return np.sqrt(dx ** 2 + dy ** 2 + dz ** 2)


    def _is_valid(self, point: Tuple[int, int, int]) -> bool:
        """
        Check if a point is within map bounds and is free space in the *original* map.
        Safety margin is handled by the cost function, not here.
        """
        x, y, z = point
        return (0 <= x < self.x_size and
                0 <= y < self.y_size and
                0 <= z < self.z_size and
                self.map[x, y, z] == self.free_value) # Check against original map


    def _calculate_penalty(self, point: Tuple[int, int, int]) -> float:
        """
        Calculate the penalty cost for being close to an obstacle at this point.
        """
        x, y, z = point
        # Get distance from the pre-calculated distance map
        distance = self.distance_map[x, y, z]

        if distance == 0: # Should not happen for valid free cells checked by _is_valid
             # If somehow a point on an obstacle got here, penalize infinitely
             return float('inf')
        elif distance == np.inf:
             # Very far from obstacles, no penalty
             return 0.0
        else:
            # Simple inverse penalty: penalty decreases as distance increases
            # Add a small value (like 1.0) to the distance in denominator to avoid
            # very large penalties right next to obstacles and make penalty finite.
            # Using distance**2 + 1 makes penalty drop faster.
            # The minimum distance for a free cell next to an obstacle is 1 in the distance_map.
            # Penalty at dist=1: penalty_factor / (1*1 + 1) = penalty_factor / 2
            # Penalty at dist=2: penalty_factor / (2*2 + 1) = penalty_factor / 5
            # Penalty at dist=3: penalty_factor / (3*3 + 1) = penalty_factor / 10
            return self.penalty_factor / (distance**2 + 1.0) # Or penalty_factor / distance, etc.


    def find_path(self,
                  start: Tuple[int, int, int],
                  goal: Tuple[int, int, int]) -> Optional[List[Tuple[int, int, int]]]:
        """
        Find a path from start to goal using A* algorithm with obstacle avoidance cost.
        """
        # Check if start and goal are valid in the original map
        if not self._is_valid(start):
            raise ValueError(f"Start point {start} is invalid (out of bounds or obstacle in original map)")
        if not self._is_valid(goal):
            raise ValueError(f"Goal point {goal} is invalid (out of bounds or obstacle in original map)")

        # Also check if start/goal are in areas infinitely far from obstacles (shouldn't happen in a connected map)
        # or if distance map calculation failed for them.
        if self.distance_map[start] == np.inf or self.distance_map[goal] == np.inf:
             print(f"Warning: Start ({start}) or Goal ({goal}) seem infinitely far from any obstacle. Check map connectivity.")
             # Although they are valid free cells, they might be in an isolated free region.
             # A* will still work if a path exists.

        # Priority queue: (f_score, g_score, current_node)
        # Use f_score for primary sort, g_score for tie-breaking (prefer shorter paths so far)
        open_set = []
        # Initial g_score includes the penalty for the start node itself
        initial_g = self._calculate_penalty(start) # Cost to "reach" start from start, including its penalty
        heapq.heappush(open_set, (initial_g + self._heuristic(start, goal), initial_g, start))


        # Dictionaries to keep track of the path and costs
        came_from = {}
        g_score = {start: initial_g}
        # f_score = {start: initial_g + self._heuristic(start, goal)} # f_score can be calculated on the fly or stored

        closed_set = set()

        while open_set:
            # Pop node with lowest f_score (then lowest g_score for tie-breaking)
            current_f, current_g, current = heapq.heappop(open_set)

            if current == goal:
                print("Path found.")
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

                # Check validity using the *original* map
                if not self._is_valid(neighbor):
                    continue

                # Calculate step cost (movement cost + penalty cost)
                movement_cost = np.sqrt(direction[0] ** 2 + direction[1] ** 2 + direction[2] ** 2)
                penalty_cost = self._calculate_penalty(neighbor)

                # Total cost to reach neighbor through current
                tentative_g = current_g + movement_cost + penalty_cost

                # If the neighbor is in the closed set and the new path is not better, skip
                if neighbor in closed_set and tentative_g >= g_score.get(neighbor, float('inf')):
                     continue

                # If this is a better path than previously found, or node not visited yet
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score_neighbor = tentative_g + self._heuristic(neighbor, goal)
                    # Add or update node in the open set
                    heapq.heappush(open_set, (f_score_neighbor, tentative_g, neighbor))

        # No path found
        print("No path found.")
        return None


# Example Usage:
if __name__ == "__main__":
    # Create a sample 3D map (10x10x5)
    # 0.0 represents free space, 1.0 represents obstacles
    map_shape = (10, 10, 5)
    sample_map = np.zeros(map_shape)

    # Add some obstacles to create a narrow passage
    sample_map[3:7, 3:7, 1:4] = 1.0 # A block obstacle
    sample_map[1, 5, :] = 1.0        # A vertical line obstacle near start
    sample_map[8, :, 2] = 1.0        # A horizontal line obstacle near goal

    # Scenario: Obstacles create a narrow passage
    # Path might need to go through (5, y, 2) where y is between 2 and 8
    sample_map[4, 0:8, 2] = 1.0 # Wall from y=0 to 7
    sample_map[6, 2:10, 2] = 1.0 # Wall from y=2 to 9
    # This creates a narrow vertical passage at x=5, y=8,9, x=6, y=0,1
    # And a passage around y=8,9, x between 4 and 6
    # And a passage around y=0,1, x between 4 and 6

    # Define start and goal points
    start_point = (1, 1, 2)
    goal_point = (2, 2, 2)

    print(f"\n--- Running A* with penalty_factor = 0 (standard A*) ---")
    # Use 26 directions for a more flexible search in 3D
    astar_standard = AStar(sample_map, penalty_factor=0, directions='26')
    path_standard = astar_standard.find_path(start_point, goal_point)

    if path_standard is not None:
        print(f"Path found (standard A*). Length: {len(path_standard)}")
        # Calculate minimum distance to obstacle for this path
        obstacle_locations = np.argwhere(sample_map == 1.0)
        min_dist_to_obs = np.inf
        for point in path_standard:
            px, py, pz = point
            min_dist_sq_point = np.inf
            for obs_loc in obstacle_locations:
                 ox, oy, oz = obs_loc
                 dist_sq = (px - ox)**2 + (py - oy)**2 + (pz - oz)**2
                 min_dist_sq_point = min(min_dist_sq_point, dist_sq)
            min_dist_to_obs = min(min_dist_to_obs, np.sqrt(min_dist_sq_point))
        print(f"Minimum Euclidean distance from path to any original obstacle: {min_dist_to_obs:.2f}")

    else:
        print(f"No path found with standard A*.")

    print(f"\n--- Running A* with penalty_factor = 50 ---")
    astar_penalty_moderate = AStar(sample_map, penalty_factor=50, directions='26')
    path_penalty_moderate = astar_penalty_moderate.find_path(start_point, goal_point)

    if path_penalty_moderate is not None:
        print(f"Path found (moderate penalty). Length: {len(path_penalty_moderate)}")
        obstacle_locations = np.argwhere(sample_map == 1.0)
        min_dist_to_obs = np.inf
        for point in path_penalty_moderate:
            px, py, pz = point
            min_dist_sq_point = np.inf
            for obs_loc in obstacle_locations:
                 ox, oy, oz = obs_loc
                 dist_sq = (px - ox)**2 + (py - oy)**2 + (pz - oz)**2
                 min_dist_sq_point = min(min_dist_sq_point, dist_sq)
            min_dist_to_obs = min(min_dist_to_obs, np.sqrt(min_dist_sq_point))
        print(f"Minimum Euclidean distance from path to any original obstacle: {min_dist_to_obs:.2f}")
        # You will likely observe this distance is larger than in the standard A* case (if a path existed).

    else:
        print(f"No path found with moderate penalty.") # Could happen if penalty makes all paths too expensive

    print(f"\n--- Running A* with penalty_factor = 500 (strong penalty) ---")
    astar_penalty_strong = AStar(sample_map, penalty_factor=500, directions='26')
    path_penalty_strong = astar_penalty_strong.find_path(start_point, goal_point)

    if path_penalty_strong is not None:
        print(f"Path found (strong penalty). Length: {len(path_penalty_strong)}")
        obstacle_locations = np.argwhere(sample_map == 1.0)
        min_dist_to_obs = np.inf
        for point in path_penalty_strong:
            px, py, pz = point
            min_dist_sq_point = np.inf
            for obs_loc in obstacle_locations:
                 ox, oy, oz = obs_loc
                 dist_sq = (px - ox)**2 + (py - oy)**2 + (pz - oz)**2
                 min_dist_sq_point = min(min_dist_sq_point, dist_sq)
            min_dist_to_obs = min(min_dist_to_obs, np.sqrt(min_dist_sq_point))
        print(f"Minimum Euclidean distance from path to any original obstacle: {min_dist_to_obs:.2f}")
         # This distance should be even larger, favoring paths further away.
    else:
        print(f"No path found with strong penalty.") # More likely to happen if path is narrow


