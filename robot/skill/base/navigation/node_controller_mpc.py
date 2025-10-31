# =============================================================================
# Node Controller MPC Module - Model Predictive Control for Robot Navigation
# =============================================================================
#
# This module provides MPC-based trajectory tracking control for robot
# navigation, using CasADi for optimization and trajectory interpolation.
#
# =============================================================================

# Standard library imports
import threading
from typing import Dict, Any, Tuple, Optional

# Third-party library imports
import numpy as np
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation as R
import casadi as ca

# from utils.threading_event import FeedbackEvent

class SimpleEvent:
    """简单的事件状态管理，不需要线程安全"""
    def __init__(self):
        self._is_set = False
        self._value = None
    
    def set(self, value=None):
        self._is_set = True
        self._value = value
    
    def clear(self):
        self._is_set = False
        self._value = None
    
    def is_set(self):
        return self._is_set
    
    def get_value(self):
        return self._value

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Duration
from rosgraph_msgs.msg import Clock


class TrajectoryManager:
    """
    Manages and interpolates discrete trajectory data from toppra.
    Provides a continuous, queryable reference signal for the MPC.
    """

    def __init__(self, trajectory_data: Dict[str, Any]):
        if not trajectory_data.get("success", False):
            raise ValueError(
                "Cannot initialize TrajectoryManager with a failed trajectory."
            )

        self.timestamps = np.array(trajectory_data["timestamps"])
        self.positions = np.array(
            trajectory_data["positions"]
        )  # Shape: (N, 4) -> x,y,z,yaw
        self.velocities = np.array(
            trajectory_data["velocities"]
        )  # Shape: (N, 4) -> vx,vy,vz,omega

        if self.timestamps.size == 0:
            raise ValueError("Trajectory data cannot be empty.")

        self.duration = self.timestamps[-1]
        self.dof = self.positions.shape[1]

        self.pos_interpolators = [
            interp1d(
                self.timestamps,
                self.positions[:, i],
                kind="linear",
                bounds_error=False,
                fill_value=self.positions[-1, i],
            )
            for i in range(self.dof)
        ]
        self.vel_interpolators = [
            interp1d(
                self.timestamps,
                self.velocities[:, i],
                kind="linear",
                bounds_error=False,
                fill_value=self.velocities[-1, i],
            )
            for i in range(self.dof)
        ]

    def get_reference_slice(
        self, t_start: float, horizon_N: int, dt: float
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Gets the reference trajectory slice for the MPC prediction horizon."""
        future_times = np.linspace(t_start, t_start + horizon_N * dt, horizon_N + 1)

        ref_states = np.array(
            [interp(future_times) for interp in self.pos_interpolators]
        ).T
        ref_controls = np.array(
            [interp(future_times[:-1]) for interp in self.vel_interpolators]
        ).T

        return ref_states, ref_controls


class CasadiMpcController:
    """
    Implements the CasADi-based Model Predictive Controller.
    """

    def __init__(self, robot_dynamics: Dict, N: int, dt: float):
        self.N = N
        self.dt = dt
        self.dof_x = 4  # State: [px, py, pz, yaw]
        self.dof_u = 4  # Control: [vx, vy, vz, omega]

        self.v_max = robot_dynamics["max_velocity"]
        self.omega_max = robot_dynamics["max_omega"]

        self._build_mpc_problem()

    def _build_mpc_problem(self):
        # Define symbolic variables
        x = ca.SX.sym("x", self.dof_x)
        u = ca.SX.sym("u", self.dof_u)

        # Define the system model (integrator)
        x_dot = ca.vertcat(u[0], u[1], u[2], u[3])
        self.f = ca.Function("f", [x, u], [x + self.dt * x_dot])

        # Define optimization variables and parameters
        U = ca.SX.sym("U", self.dof_u, self.N)
        X = ca.SX.sym("X", self.dof_x, self.N + 1)
        num_params = self.dof_x + (self.N + 1) * self.dof_x + self.N * self.dof_u
        P = ca.SX.sym("P", num_params)

        # Define cost function
        obj = 0
        Q = ca.diag([10.0, 10.0, 10.0, 1.0])  # State tracking weights
        R = ca.diag([0.5, 0.5, 0.5, 0.1])  # Control effort weights

        x_ref_slice = ca.reshape(
            P[self.dof_x : self.dof_x + (self.N + 1) * self.dof_x],
            self.dof_x,
            self.N + 1,
        )
        u_ref_slice = ca.reshape(
            P[self.dof_x + (self.N + 1) * self.dof_x :], self.dof_u, self.N
        )

        for k in range(self.N):
            state_error = X[:, k] - x_ref_slice[:, k]
            control_error = U[:, k] - u_ref_slice[:, k]
            obj += ca.mtimes([state_error.T, Q, state_error]) + ca.mtimes(
                [control_error.T, R, control_error]
            )

        final_state_error = X[:, self.N] - x_ref_slice[:, self.N]
        obj += ca.mtimes([final_state_error.T, Q, final_state_error])

        # Define constraints
        g = []
        g.append(X[:, 0] - P[: self.dof_x])  # Initial state constraint
        for k in range(self.N):
            g.append(X[:, k + 1] - self.f(X[:, k], U[:, k]))  # Dynamics constraint

        # Create NLP solver
        OPT_variables = ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))
        nlp_prob = {"f": obj, "x": OPT_variables, "g": ca.vertcat(*g), "p": P}
        opts = {"ipopt": {"print_level": 0}, "print_time": 0}
        self.solver = ca.nlpsol("solver", "ipopt", nlp_prob, opts)

        # Define bounds
        self.lbg = [0.0] * (self.dof_x * (self.N + 1))
        self.ubg = [0.0] * (self.dof_x * (self.N + 1))

        self.lbx = [-ca.inf] * (self.dof_x * (self.N + 1))
        self.ubx = [ca.inf] * (self.dof_x * (self.N + 1))
        for _ in range(self.N):
            self.lbx.extend([-self.v_max, -self.v_max, -self.v_max, -self.omega_max])
            self.ubx.extend([self.v_max, self.v_max, self.v_max, self.omega_max])

    def solve(
        self,
        current_state: np.ndarray,
        ref_states: np.ndarray,
        ref_controls: np.ndarray,
    ) -> np.ndarray:
        p_vec = np.concatenate(
            [current_state, ref_states.flatten(), ref_controls.flatten()]
        )
        sol = self.solver(
            x0=0, lbx=self.lbx, ubx=self.ubx, lbg=self.lbg, ubg=self.ubg, p=p_vec
        )
        optimal_U_flat = sol["x"][self.dof_x * (self.N + 1) :].full().flatten()
        return optimal_U_flat[: self.dof_u]


class NodeMpcController(Node):
    """
    Receives a timed trajectory and uses an MPC to track it.

    - Subscribes to `/timed_trajectory` to receive the mission.
    - Subscribes to `/odom` to get the current robot state.
    - Publishes velocity commands to `/cmd_vel`.
    """

    def __init__(self, namespace: str):
        super().__init__(node_name="node_mpc_controller", namespace=namespace)

        # Load parameters
        self.declare_parameters(
            namespace="",
            parameters=[
                ("mpc.N", 20),
                ("mpc.dt", 0.01),
                ("robot.max_velocity", 1.5),
                ("robot.max_acceleration", 0.8),
                ("robot.max_omega", 1.0),
                ("robot.max_alpha", 1.5),
                ("goal.position_tolerance", 0.1),
                ("goal.velocity_tolerance", 0.1),
                ("goal.completion_timeout", 5.0),
                ("time.use_sim_time", True),  # 是否使用仿真时间
            ],
        )

        self.mpc_horizon = self.get_parameter("mpc.N").value
        self.mpc_dt = self.get_parameter("mpc.dt").value
        robot_dynamics = {
            "max_velocity": self.get_parameter("robot.max_velocity").value,
            "max_acceleration": self.get_parameter("robot.max_acceleration").value,
            "max_omega": self.get_parameter("robot.max_omega").value,
            "max_alpha": self.get_parameter("robot.max_alpha").value,
        }
        self.goal_pos_tolerance = self.get_parameter("goal.position_tolerance").value
        self.goal_vel_tolerance = self.get_parameter("goal.velocity_tolerance").value
        self.completion_timeout = self.get_parameter("goal.completion_timeout").value

        # Instantiate core components
        self.mpc_controller = CasadiMpcController(
            robot_dynamics, N=self.mpc_horizon, dt=self.mpc_dt
        )
        self.trajectory_manager = None
        self.trajectory_start_time = None
        self.current_state = None
        self.has_reached_goal = True

        self.move_event = SimpleEvent()
        self.latest_sim_time = 0

        # ROS 2 Communications
        self.trajectory_sub = self.create_subscription(
            JointTrajectory, "timed_trajectory", self.trajectory_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, "odom", self.odom_callback, 10
        )
        # self.control_timer = self.create_timer(self.mpc_dt, self.control_loop)
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.clock_sub = self.create_subscription(
            Clock, "/isaacsim_simulation_clock", self.clock_callback, 10
        )
        self.get_logger().info("MPC Controller Node has started.")

    def clock_callback(self, msg: Clock):
        sim_time_float = msg.clock.sec + msg.clock.nanosec / 1e9
        self.latest_sim_time = sim_time_float

    def trajectory_callback(self, msg: JointTrajectory):
        """Handles incoming trajectory messages."""
        self.get_logger().info(
            f"New trajectory received with {len(msg.points)} points."
        )
        try:
            trajectory_data = self._convert_traj_msg_to_dict(msg)
            self.trajectory_manager = TrajectoryManager(trajectory_data)
            self.trajectory_start_time = self.latest_sim_time
            self.has_reached_goal = False
            self.move_event.clear()  # 重置事件状态，开始新的移动任务
        except ValueError as e:
            self.get_logger().error(f"Failed to process trajectory: {e}")
            self.move_event.set(value=False)  # 轨迹处理失败

    def odom_callback(self, msg: Odometry):
        """Updates the current state of the robot from odometry data."""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        # Convert quaternion to yaw
        rotation = R.from_quat([q.x, q.y, q.z, q.w])
        yaw = rotation.as_euler("zyx", degrees=False)[0]

        self.current_state = np.array([p.x, p.y, p.z, yaw])

    def control_loop(self):
        """Main MPC control loop, running at a fixed frequency."""
        if (
            self.move_event.is_set()
            or self.trajectory_manager is None
            or self.current_state is None
        ):
            return

        elapsed_time = self.latest_sim_time - self.trajectory_start_time
        # Get reference slice from the trajectory manager
        ref_states, ref_controls = self.trajectory_manager.get_reference_slice(
            t_start=elapsed_time, horizon_N=self.mpc_horizon, dt=self.mpc_dt
        )

        # Solve the MPC problem
        optimal_command = self.mpc_controller.solve(
            self.current_state, ref_states, ref_controls
        )

        # NOTE: This assumes the robot base controller accepts world-frame velocity commands.
        # If it expects body-frame (base_link), a coordinate transformation is needed here.
        cmd_msg = Twist()
        cmd_msg.linear.x = optimal_command[0]
        cmd_msg.linear.y = optimal_command[1]
        cmd_msg.linear.z = optimal_command[2]
        cmd_msg.angular.z = optimal_command[3]
        self.cmd_vel_pub.publish(cmd_msg)

        if elapsed_time > self.trajectory_manager.duration:
            final_goal_state = self.trajectory_manager.positions[-1]

            # Condition 1: Have we reached the goal?
            position_error = np.linalg.norm(
                self.current_state[:3] - final_goal_state[:3]
            )
            if position_error < self.goal_pos_tolerance:
                self.get_logger().info(
                    "Goal reached within tolerance. Mission successful!"
                )
                self.has_reached_goal = True
                self.move_event.set(value=True)
                self.stop_robot()
                return

            # Condition 2: Have we run out of extra time?
            if (
                elapsed_time
                > self.trajectory_manager.duration + self.completion_timeout
            ):
                self.get_logger().warn(
                    f"Goal not reached within timeout ({self.completion_timeout}s extra time). "
                    f"Stopping at final distance of {position_error:.2f}m."
                )
                self.has_reached_goal = True
                self.move_event.set(value=False)  # 超时失败，也要设置事件
                self.stop_robot()
                return

    def _convert_traj_msg_to_dict(self, msg: JointTrajectory) -> Dict:
        """Converts a JointTrajectory message to the dictionary format for TrajectoryManager."""
        timestamps, positions, velocities = [], [], []
        for point in msg.points:
            time_sec = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
            timestamps.append(time_sec)
            positions.append(point.positions)
            velocities.append(point.velocities)

        return {
            "success": True,
            "timestamps": timestamps,
            "positions": positions,
            "velocities": velocities,
        }

    def stop_robot(self):
        cmd_msg = Twist()
        self.cmd_vel_pub.publish(cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    node = NodeMpcController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
