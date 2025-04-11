# import isaacsim.core.api.utils.prims as prim_utils
# from isaacsim.core.api.simulation_context import SimulationContext
# from omni.isaac.debug_draw import _debug_draw  # Use the singleton instance
# import omni.usd
# import carb  # For color definitions

import numpy as np
from isaacsim.core.api.scenes import Scene


class Trajectory:
    """
    Manages and displays the historical trajectory of a specified robot prim.
    """

    def __init__(self, robot_prim_path: str, max_points: int = 500, id: int = 0,
                 color: tuple = (0.0, 1.0, 0.0), thickness: float = 2.0, scene: Scene = None, radius: float = 0.05):
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
        from isaacsim.core.api.objects import VisualSphere
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
# def
#
# def _update_trajectory(self, step: float):
#     """
#     Callback function executed at each physics step.
#     Records the robot's current position and triggers drawing.
#     """
#     if not self._is_tracking:
#         return
#
#     # Ensure prim is still valid (it might get deleted)
#     if not self.robot_prim or not self.robot_prim.IsValid():
#         self.robot_prim = self._get_robot_prim()  # Try to re-acquire
#         if not self.robot_prim:
#             # print(f"Warning: Robot prim at {self.robot_prim_path} is no longer valid. Stopping tracking update for this step.")
#             return  # Skip update if prim is gone
#
#     try:
#         # Get current world position of the robot prim
#         current_pos, _ = prim_utils.get_world_pose(self.robot_prim)
#
#         # Add the new position (convert numpy array to list or tuple if needed by deque)
#         self.trajectory_points.append(tuple(current_pos))
#
#         # Draw the trajectory if enough points exist
#         self._draw_trajectory()
#
#     except Exception as e:
#         print(f"Error updating/drawing trajectory: {e}")
#         # Optionally stop tracking if errors persist
#         # self.stop_tracking()
#
# def _draw_trajectory(self):
#     """Draws the trajectory using DebugDraw."""
#     if len(self.trajectory_points) >= 2:
#         # Convert deque of tuples/lists to a flat list of floats for drawing
#         # points_list = [coord for point in self.trajectory_points for coord in point]
#         # Note: draw_polyline expects a list of Vec3 points (or similar)
#         points_list = list(self.trajectory_points)  # Convert deque to list of tuples/lists
#
#         # Use draw_polyline for connecting lines
#         self.draw.draw_polyline(points_list, color=self.color, thickness=self.thickness)
#
#         # Alternatively, draw individual points:
#         # self.draw.draw_points(points_list, [(self.color)] * len(points_list), [self.thickness] * len(points_list))
#
# def start_tracking(self):
#     """Starts tracking the robot's trajectory."""
#     if self._is_tracking:
#         print("Tracking is already active.")
#         return
#
#     # Check if prim exists before starting
#     self.robot_prim = self._get_robot_prim()
#     if not self.robot_prim:
#         print(f"Error: Cannot start tracking. Robot prim not found at path: {self.robot_prim_path}")
#         return
#
#     # if self.simulation_context:
#     #     # Register the callback to the physics step event
#     #     self._physics_callback_id = self.simulation_context.add_physics_callback(
#     #         "robot_trajectory_update", self._update_trajectory
#     #     )
#     #     self._is_tracking = True
#     #     print(f"Started tracking trajectory for: {self.robot_prim_path}")
#     # else:
#     #     print("Error: SimulationContext not available to start tracking.")
#
# def stop_tracking(self):
#     """Stops tracking the robot's trajectory."""
#     if not self._is_tracking:
#         # print("Tracking is not active.")
#         return
#
#     if self.simulation_context and self._physics_callback_id is not None:
#         # Remove the callback
#         removed = self.simulation_context.remove_physics_callback(self._physics_callback_id)
#         if removed:
#             print(f"Stopped tracking trajectory for: {self.robot_prim_path}")
#         else:
#             print(f"Warning: Could not remove physics callback for trajectory tracking.")
#
#         self._physics_callback_id = None
#         self._is_tracking = False
#
#     # Clear existing drawings (optional, they might disappear next frame anyway)
#     # self.clear_drawing() # Usually not needed as DebugDraw clears frame by frame
#
# def clear_trajectory(self):
#     """Clears the stored trajectory points."""
#     self.trajectory_points.clear()
#     print(f"Cleared trajectory points for: {self.robot_prim_path}")
#     # Note: This doesn't immediately clear the drawing from the screen
#     # until the next frame where _draw_trajectory doesn't draw anything.
#
# def set_robot_prim_path(self, robot_prim_path: str):
#     """Changes the prim being tracked."""
#     was_tracking = self._is_tracking
#     if was_tracking:
#         self.stop_tracking()
#
#     self.robot_prim_path = robot_prim_path
#     self.robot_prim = self._get_robot_prim()  # Update prim reference
#     self.clear_trajectory()  # Clear old trajectory
#
#     if not self.robot_prim:
#         print(f"Warning: New robot prim not found at path: {self.robot_prim_path}")
#
#     if was_tracking:
#         self.start_tracking()  # Restart tracking with the new prim if it was active
#
# def __del__(self):
#     """Ensures callback is removed when the object is deleted."""
#     self.stop_tracking()
#     # Release debug draw interface if necessary (depends on how singleton is managed)
#     # _debug_draw.release_debug_draw_interface(self.draw) # Usually not needed for singleton

# --- Example Usage ---
#
# async def setup_and_run_trajectory():
#     # 1. Get Simulation Context (Assuming it's already running or you start it here)
#     simulation_context = SimulationContext.instance()
#     if simulation_context is None:
#         simulation_context = SimulationContext()  # Start one if needed for standalone script
#
#     # --- A. Load your robot first! ---
#     # Example: Load a Franka robot (replace with your robot loading code)
#     # Make sure the prim path used below ('/World/Franka/panda_link0') exists after loading.
#     from omni.isaac.core.robots import Robot
#     # prim_path = "/World/Franka" # Adjust if needed
#     # usd_path = "/Isaac/Robots/Franka/franka_alt_fingers.usd"
#     # prim_utils.create_prim(prim_path, "Xform", usd_path=usd_path, position=np.array([0,0,0]))
#
#     # --- B. Define the path to the prim you want to track ---
#     # IMPORTANT: Replace this with the actual path to your robot's base or relevant link
#     robot_base_prim_path = "/World/Franka/panda_link0"
#     # Or for a different robot, e.g., "/World/Carter_v2/chassis_link"
#
#     # 2. Create the Trajectory Tracker instance
#     trajectory_tracker = RobotTrajectory(robot_prim_path=robot_base_prim_path,
#                                          max_points=1000,
#                                          color=(1.0, 0.0, 0.0, 1.0),  # Red color
#                                          thickness=3.0)
#
#     # 3. Start Tracking when simulation starts playing
#     # We can register a callback for the play event, or simply call start_tracking()
#     # if we know the simulation is about to play.
#
#     # Example: Start tracking immediately if simulation is already playing or will play soon
#     # Check if prim exists before starting
#     if trajectory_tracker.robot_prim:
#         trajectory_tracker.start_tracking()
#     else:
#         print(f"Robot prim {robot_base_prim_path} not found. Trajectory tracking not started.")
#         # You might want to add logic here to wait for the robot to be loaded
#
#     # 4. Simulation Loop (Example structure)
#     # This part would typically be within your main Isaac Sim application structure
#     # For a standalone script, it might look like this:
#
#     # Make sure simulation is playing
#     if not simulation_context.is_playing():
#         simulation_context.play()
#
#     # Run simulation steps (example for a few seconds)
#     for i in range(300):  # Simulate for ~5 seconds if physics DT is ~1/60s
#         simulation_context.step(render=True)  # Must render=True to see debug drawings
#         if not simulation_context.is_playing():  # Stop if user presses stop
#             break
#
#             # 5. Stop Tracking (e.g., when simulation stops or app closes)
#     print("Simulation loop finished.")
#     trajectory_tracker.stop_tracking()
#
#     # Optional: Clear points after stopping
#     # trajectory_tracker.clear_trajectory()
#
#     # Clean up simulation context if started here
#     # simulation_context.stop() # Stop simulation
#     # simulation_context.clear_instance() # If you created it here

# --- How to Run ---
# 1. Open Isaac Sim.
# 2. Load your robot into the stage (e.g., Franka, Carter, or your custom robot).
# 3. Open the Script Editor window (Window -> Script Editor).
# 4. Paste the *entire* code (RobotTrajectory class and the example usage part) into the Script Editor.
# 5. **Crucially, modify `robot_base_prim_path` in the example usage section to match the correct USD path of your robot's base link (or the link you want to track).** Use the Stage window to find the exact path.
# 6. Click the "Run" button (play icon) in the Script Editor.
# 7. Press the main "Play" button in the Isaac Sim UI to start the simulation.
# 8. Move your robot around (e.g., using MoveIt, RMPFlow, or keyboard teleop). You should see a colored line tracing its path.

# To run as a standalone application (more advanced):
# import asyncio
# asyncio.ensure_future(setup_and_run_trajectory())
# Make sure your environment is set up correctly for standalone execution.
