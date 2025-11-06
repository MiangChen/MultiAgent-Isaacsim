# =============================================================================
# Node Trajectory Generator Module - Time-Optimal Trajectory Generation
# =============================================================================
#
# This module provides ROS2 node implementation for generating time-optimal
# trajectories from geometric paths using the TOPPRA library.
#
# =============================================================================

# Standard library imports
from typing import Dict, Any

# Third-party library imports
import numpy as np
from scipy.spatial.transform import Rotation as R
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo

# ROS2 imports
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from nav_msgs.msg import Path
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class NodeTrajectoryGenerator(Node):
    """
    Generates a time-optimal trajectory from a geometric path.

    This node subscribes to a `nav_msgs/Path`, processes it using the toppra
    library to calculate a time-parameterized trajectory, and then publishes
    the result as a `trajectory_msgs/JointTrajectory`.
    """

    def __init__(self, namespace: str):
        super().__init__(node_name="node_trajectory_generator", namespace=namespace)

        self.declare_parameters(
            namespace="",
            parameters=[
                ("max_velocity", 1.5),
                ("max_acceleration", 0.8),
                ("max_omega", 1.0),
                ("max_alpha", 1.5),
                ("sample_rate", 50),
            ],
        )

        self.robot_dynamics = {
            "max_velocity": self.get_parameter("max_velocity").value,
            "max_acceleration": self.get_parameter("max_acceleration").value,
            "max_omega": self.get_parameter("max_omega").value,
            "max_alpha": self.get_parameter("max_alpha").value,
        }
        self.sample_rate = self.get_parameter("sample_rate").value

        # Subscribe to the geometric path from the planner
        self.path_subscription = self.create_subscription(
            Path, "planned_path", self.path_callback, 10
        )

        # Publish the timed trajectory for the controller
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory, "timed_trajectory", 10
        )
        self.get_logger().info("Trajectory Generator Node has started.")

    def path_callback(self, msg: Path):
        """Callback function for the /planned_path topic."""
        # 1. Extract the 4D path (x, y, z, yaw) from the Path message
        path_4d = self._extract_4d_path_from_msg(msg)
        if path_4d is None or len(path_4d) < 2:
            self.get_logger().warn("Path has too few points to generate a trajectory.")
            return

        # 2. Generate the time-optimal trajectory using toppra
        trajectory_data = self._generate_trajectory_with_toppra(path_4d)
        if not trajectory_data["success"]:
            self.get_logger().error(
                f"Toppra trajectory generation failed: {trajectory_data['message']}"
            )
            return

        # 3. Convert the toppra output to a JointTrajectory message
        trajectory_msg = self._convert_to_joint_trajectory_msg(
            trajectory_data, msg.header.frame_id
        )

        # 4. Publish the final trajectory message
        self.trajectory_publisher.publish(trajectory_msg)
        self.get_logger().info(
            f"Successfully published trajectory of {trajectory_data['timestamps'][-1]:.2f}s to /timed_trajectory"
        )

    def _extract_4d_path_from_msg(self, msg: Path) -> np.ndarray:
        """Extracts an (x, y, z, yaw) path from a nav_msgs/Path message."""
        positions = []
        quaternions = []
        for pose_stamped in msg.poses:
            p = pose_stamped.pose.position
            o = pose_stamped.pose.orientation
            positions.append([p.x, p.y, p.z])
            quaternions.append([o.x, o.y, o.z, o.w])

        if not quaternions:
            return None

        # Batch convert quaternions to continuous yaw angles
        rotations = R.from_quat(np.array(quaternions))
        euler_angles = rotations.as_euler("zyx", degrees=False)
        raw_yaw_angles = euler_angles[:, 0]

        # np.unwrap handles angle wrapping (-pi to pi jumps) for smooth angular velocity
        unwrapped_yaw_angles = np.unwrap(raw_yaw_angles)

        # Stack positions and yaw to create the final (N, 4) path
        return np.column_stack((np.array(positions), unwrapped_yaw_angles))

    def _generate_trajectory_with_toppra(
            self, path_points: np.ndarray
    ) -> Dict[str, Any]:
        """Generates a trajectory using toppra for a given 4D path."""
        ss = np.zeros(len(path_points))
        ss[1:] = np.cumsum(np.linalg.norm(np.diff(path_points, axis=0), axis=1))

        path = ta.SplineInterpolator(ss, path_points)

        v_lims = np.array(
            [
                [
                    -self.robot_dynamics["max_velocity"],
                    self.robot_dynamics["max_velocity"],
                ],
                [
                    -self.robot_dynamics["max_velocity"],
                    self.robot_dynamics["max_velocity"],
                ],
                [
                    -self.robot_dynamics["max_velocity"],
                    self.robot_dynamics["max_velocity"],
                ],
                [-self.robot_dynamics["max_omega"], self.robot_dynamics["max_omega"]],
            ]
        )
        a_lims = np.array(
            [
                [
                    -self.robot_dynamics["max_acceleration"],
                    self.robot_dynamics["max_acceleration"],
                ],
                [
                    -self.robot_dynamics["max_acceleration"],
                    self.robot_dynamics["max_acceleration"],
                ],
                [
                    -self.robot_dynamics["max_acceleration"],
                    self.robot_dynamics["max_acceleration"],
                ],
                [-self.robot_dynamics["max_alpha"], self.robot_dynamics["max_alpha"]],
            ]
        )

        con_vel = constraint.JointVelocityConstraint(v_lims)
        con_acc = constraint.JointAccelerationConstraint(a_lims)
        con_acc.set_discretization_type(constraint.DiscretizationType.Interpolation)

        instance = algo.TOPPRA([con_vel, con_acc], path)
        traj = instance.compute_trajectory(0, 0)

        if traj is None:
            return {
                "success": False,
                "message": "Toppra failed to compute a valid trajectory.",
            }

        duration = traj.duration
        ts_sample = np.linspace(0, duration, int(duration * self.sample_rate))
        positions = traj(ts_sample, 0)
        velocities = traj(ts_sample, 1)
        accelerations = traj(ts_sample, 2)

        return {
            "success": True,
            "timestamps": ts_sample,
            "positions": positions,
            "velocities": velocities,
            "accelerations": accelerations,
        }

    def _convert_to_joint_trajectory_msg(
            self, traj_data: Dict, frame_id: str
    ) -> JointTrajectory:
        """Converts the toppra output dictionary into a JointTrajectory message."""
        traj_msg = JointTrajectory()
        traj_msg.header.stamp = self.get_clock().now().to_msg()
        traj_msg.header.frame_id = frame_id
        traj_msg.joint_names = ["x", "y", "z", "yaw"]

        timestamps = traj_data["timestamps"]
        positions = traj_data["positions"]
        velocities = traj_data["velocities"]
        accelerations = traj_data["accelerations"]

        for i in range(len(timestamps)):
            point = JointTrajectoryPoint()

            time_from_start_sec = timestamps[i]
            point.time_from_start = Duration(
                sec=int(time_from_start_sec),
                nanosec=int((time_from_start_sec - int(time_from_start_sec)) * 1e9),
            )

            point.positions = positions[i].tolist()
            point.velocities = velocities[i].tolist()
            point.accelerations = accelerations[i].tolist()

            traj_msg.points.append(point)

        return traj_msg
