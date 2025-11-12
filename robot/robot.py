# =============================================================================
# Robot Module - Core Robot Implementation
# =============================================================================
#
# This module provides the base Robot class and related functionality for
# robotic simulation and control within the Isaac Sim environment.
#
# =============================================================================

# Standard library imports
from typing import Any, Dict

# Third-party library imports
import numpy as np
import torch

# Local project imports
from log.log_manager import LogManager
from physics_engine.isaacsim_utils import Scene, prims_utils
from robot.body import BodyRobot

# ROS2 message imports
from gsi_msgs.gsi_msgs_helper import (
    RobotFeedback,
    SkillExecution,
    SkillInfo,
    Parameter,
    VelTwistPose,
)

logger = LogManager.get_logger(__name__)


class Robot:
    def __init__(self):
        self.cfg_robot.path_prim_robot = (
            self.cfg_robot.path_prim_swarm
            + f"/{self.cfg_robot.type}"
            + f"/{self.cfg_robot.type}_{self.cfg_robot.id}"
        )
        self.cfg_robot.namespace = self.cfg_robot.type + f"_{self.cfg_robot.id}"
        self.namespace = self.cfg_robot.namespace

        self._body: BodyRobot = None  # Private: Isaac Sim layer only

        # 机器人的控制器
        self.controllers: dict = {}  # 用于存储多个控制器, 'controller name': function
        self.control_mode: str = (
            ""  # 'joint_efforts', 'joint_velocities', 'joint_positions', 'joint_indices', 'joint_names'
        )
        self.action: np.ndarray = None

        # Robot state (cached from Isaac Sim, updated in on_physics_step)
        self._position = torch.tensor([0.0, 0.0, 0.0])
        self._quat = torch.tensor([0.0, 0.0, 0.0, 1.0])
        self._linear_velocity = torch.tensor(
            [0.0, 0.0, 0.0]
        )  # Actual linear velocity (state)
        self._angular_velocity = torch.tensor(
            [0.0, 0.0, 0.0]
        )  # Actual angular velocity (state)

        # Robot control commands (set by controllers, applied in controller_simplified)
        # Following CARLA naming: these are target/command values, not actual state
        self.target_linear_velocity = torch.tensor(
            [0.0, 0.0, 0.0]
        )  # Target linear velocity (command)
        self.target_angular_velocity = torch.tensor(
            [0.0, 0.0, 0.0]
        )  # Target angular velocity (command)

        # Manipulation control (set by skills, applied in on_physics_step)
        self._manipulation_control = None
        self._manipulation_result = None

        self.sim_time = 0.0

        self.view_angle: float = 2 * np.pi / 3  # 感知视野 弧度
        self.view_radius: float = 2  # 感知半径 米

        # ROS manager (optional, injected from outside)
        self.ros_manager = None

        self.is_detecting = False
        self.target_prim = None

        self.track_waypoint_list = []
        self.track_waypoint_index = 0
        self.is_tracking = False
        self.track_waypoint_sub = None

        self.track_counter = 0
        self.track_period = 300

    ########################## Public Interface (Application Layer) ############################

    def get_world_pose(self):
        """Get robot world pose (position, quaternion) - Public interface"""
        return self._position, self._quat

    def set_world_pose(self, position, orientation=None):
        """
        Set robot world pose - Public interface
        This only updates the cached state. The actual Isaac Sim update happens in on_physics_step.
        """
        self._position = (
            position if isinstance(position, torch.Tensor) else torch.tensor(position)
        )
        if orientation is not None:
            self._quat = (
                orientation
                if isinstance(orientation, torch.Tensor)
                else torch.tensor(orientation)
            )

    def get_linear_velocity(self):
        """
        Get robot linear velocity (CARLA style) - Public interface

        Returns the actual velocity (state), not the target/command velocity.
        This is safe to call from Application layer as it returns cached values.
        """
        return self._linear_velocity

    def get_angular_velocity(self):
        """
        Get robot angular velocity (CARLA style) - Public interface

        Returns the actual angular velocity (state), not the target/command velocity.
        This is safe to call from Application layer as it returns cached values.
        """
        return self._angular_velocity

    def get_world_velocity(self):
        """
        Get robot world velocity (linear, angular) - Public interface
        Alias for compatibility, returns both velocities.
        """
        return self._linear_velocity, self._angular_velocity

    def set_target_velocity(self, linear_velocity=None, angular_velocity=None):
        """
        Set robot target velocity (CARLA style) - Public interface

        This sets the TARGET velocity (command), not the actual state.
        The actual Isaac Sim update happens in controller_simplified during on_physics_step.

        Args:
            linear_velocity: Target linear velocity [x, y, z]
            angular_velocity: Target angular velocity [x, y, z] (optional)
        """
        if linear_velocity is not None:
            self.target_linear_velocity = (
                linear_velocity
                if isinstance(linear_velocity, torch.Tensor)
                else torch.tensor(linear_velocity)
            )
        if angular_velocity is not None:
            self.target_angular_velocity = (
                angular_velocity
                if isinstance(angular_velocity, torch.Tensor)
                else torch.tensor(angular_velocity)
            )

    def get_config(self):
        """Get robot configuration - Public interface"""
        if self._body is None:
            raise RuntimeError("Robot body not initialized")
        return self._body.cfg_robot

    def get_topics(self):
        """Get ROS topics configuration - Public interface"""
        cfg = self.get_config()
        return cfg.topics if hasattr(cfg, "topics") else {}

    def get_detection_radius(self):
        """Get robot detection radius - Public interface"""
        cfg = self.get_config()
        return cfg.detection_radius if hasattr(cfg, "detection_radius") else 1.0

    def get_robot_radius(self):
        """Get robot physical radius - Public interface"""
        cfg = self.get_config()
        return cfg.robot_radius if hasattr(cfg, "robot_radius") else 0.5

    def is_physics_valid(self):
        """
        Check if physics handle is valid - Public interface
        Note: This is a simple check and doesn't call Isaac Sim API directly.
        """
        return self._body is not None

    @property
    def body(self):
        """
        Deprecated: Direct access to body is discouraged.
        Use public methods like get_world_pose(), get_config(), etc.
        This property exists for backward compatibility only.
        """
        import warnings

        warnings.warn(
            "Direct access to robot.body is deprecated and may cause issues "
            "when called during Isaac Sim rendering. "
            "Use robot.get_world_pose(), robot.get_config(), etc. instead.",
            DeprecationWarning,
            stacklevel=2,
        )
        return self._body

    ########################## Publisher Odom  ############################
    def publish_robot_state(self):
        """
        Update robot state from Isaac Sim and publish to ROS (CARLA style).
        This method is called in on_physics_step, so it's safe to call Isaac Sim API here.

        Updates:
        - _position, _quat: Current pose (state)
        - _linear_velocity, _angular_velocity: Current actual velocity (state)

        Does NOT update:
        - target_linear_velocity, target_angular_velocity: These are commands set by controllers
        """
        # Read state from Isaac Sim API (safe in physics step)
        pos, quat = self._body.get_world_pose()
        linear_vel, angular_vel = self._body.get_world_vel()

        # Update cached state for Application layer
        self._position = pos
        self._quat = quat
        self._linear_velocity = linear_vel  # Actual velocity (state)
        self._angular_velocity = angular_vel  # Actual angular velocity (state)

        # DO NOT update target_linear_velocity/target_angular_velocity here!
        # They are command variables set by MPC/controllers.

        # Publish to ROS if available (use actual velocity for odometry)
        if self.has_ros():
            pos_np = pos.detach().cpu().numpy()
            quat_np = quat.detach().cpu().numpy()
            vel_linear_np = linear_vel.detach().cpu().numpy()
            vel_angular_np = angular_vel.detach().cpu().numpy()

            self.ros_manager.publish_odometry(
                pos_np, quat_np, vel_linear_np, vel_angular_np
            )

    ########################## ROS Manager Interface ############################

    def set_ros_manager(self, ros_manager):
        """Set ROS manager (dependency injection)"""
        self.ros_manager = ros_manager

    def get_ros_manager(self):
        """Get ROS manager"""
        return self.ros_manager

    def has_ros(self):
        """Check if ROS is enabled"""
        return self.ros_manager is not None

    def cleanup(self):
        """Cleanup robot resources"""
        if self.has_ros():
            self.ros_manager.stop()

    def initialize(self) -> None:
        """Initialize robot (sensors are created separately via Blueprint system)"""
        pass
        return

    def form_feedback(
        self, status: str = "processing", message: str = "none", progress: int = 100
    ) -> Dict[str, Any]:
        return dict(
            status=str(status),
            message=str(message),
            progress=progress,
        )

    def apply_control(self, control):
        """Apply CARLA-style control: Control object -> velocity command"""
        from simulation.control import RobotControl

        if isinstance(control, RobotControl):
            self._current_control = control  # Cache for get_control()
            # Set target velocity (command), not actual velocity (state)
            self.target_linear_velocity = torch.tensor(control.linear_velocity)
            self.target_angular_velocity = torch.tensor(control.angular_velocity)

    def get_control(self):
        """Get current control (CARLA-style)"""
        from simulation.control import RobotControl

        if not hasattr(self, "_current_control"):
            # Return default control if none applied yet
            control = RobotControl()
            control.linear_velocity = self.target_linear_velocity.tolist()
            control.angular_velocity = self.target_angular_velocity.tolist()
            return control
        return self._current_control

    def apply_manipulation_control(self, control):
        """
        Apply manipulation control (CARLA-style) - Public interface

        Application layer creates Control object and applies it.
        Robot layer executes it in on_physics_step.

        Args:
            control: GraspControl, ReleaseControl, or PlaceControl object
        """
        self._manipulation_control = control
        self._manipulation_result = None  # Clear previous result

    def get_manipulation_result(self):
        """
        Get manipulation result (CARLA-style) - Public interface

        Returns:
            dict: Result with 'success', 'message', 'data' keys
            None: If no result available yet
        """
        return self._manipulation_result

    def on_physics_step(self, step_size) -> None:
        """
        Physics step callback - Robot layer only

        Args:
            step_size: dt 时间间隔
        """
        # 1. Update robot state from Isaac Sim
        self.publish_robot_state()

        # 2. Apply target velocity to Isaac Sim
        # Note: target_linear_velocity is set by MPC (Application layer) via clock callback
        self.controller_simplified()

        # 3. Execute manipulation control if any
        if self._manipulation_control is not None:
            self._execute_manipulation_control()

        return

    def controller_simplified(self) -> None:
        """
        Apply target velocity commands to Isaac Sim (CARLA style).
        This is called in on_physics_step, so it's safe to call Isaac Sim API here.
        """
        if self._body and self._body.robot_articulation.is_physics_handle_valid():
            self._body.robot_articulation.set_linear_velocities(
                self.target_linear_velocity
            )
            self._body.robot_articulation.set_angular_velocities(
                self.target_angular_velocity
            )
            logger.debug(
                f"Robot Articulation target vel: {self.target_linear_velocity}"
            )

    def _execute_manipulation_control(self):
        """
        Execute manipulation control in physics step (Robot layer only).
        This is called in on_physics_step, so it's safe to call Isaac Sim API here.

        Architecture:
        - DiscreteControl: One-shot action with state management
        - Skip if already completed or failed
        - Mark as completed after execution to prevent repeated execution
        """
        from simulation.control import (
            GraspControl,
            ReleaseControl,
            PlaceControl,
            DiscreteControl,
        )
        from physics_engine.isaacsim_utils import RigidPrim

        control = self._manipulation_control

        # Skip if discrete control is already completed or failed
        if isinstance(control, DiscreteControl):
            if control.is_completed() or control.is_failed():
                return

        try:
            if isinstance(control, GraspControl):
                self._execute_grasp_control(control)
            elif isinstance(control, ReleaseControl):
                self._execute_release_control(control)
            elif isinstance(control, PlaceControl):
                self._execute_place_control(control)
            else:
                self._manipulation_result = {
                    "success": False,
                    "message": f"Unknown control type: {type(control)}",
                    "data": {},
                }
        except Exception as e:
            logger.error(f"Manipulation control execution failed: {e}")
            self._manipulation_result = {
                "success": False,
                "message": f"Execution error: {str(e)}",
                "data": {},
            }

    def _execute_grasp_control(self, control):
        """
        Execute grasp control (check distance or attach)

        Multi-stage action:
        1. CHECK_DISTANCE: Check if object is within grasp distance
        2. ATTACH: Attach object to hand (create joint)
        """
        from physics_engine.isaacsim_utils import RigidPrim
        from physics_engine.pxr_utils import UsdPhysics, Gf
        from containers import get_container
        from simulation.control import ControlAction

        if control.action == ControlAction.CHECK_DISTANCE:
            # Initialize rigid prims
            hand_prim = RigidPrim(prim_paths_expr=control.hand_prim_path)
            object_prim = RigidPrim(prim_paths_expr=control.object_prim_path)

            # Calculate distance
            hand_pos, _ = hand_prim.get_world_poses()
            object_pos, _ = object_prim.get_world_poses()
            distance = torch.norm(hand_pos - object_pos).item()

            # Check if within threshold
            if distance <= control.distance_threshold:
                self._manipulation_result = {
                    "success": True,
                    "message": "Within grasp distance",
                    "data": {
                        "distance": distance,
                        "ready_to_grasp": True,
                        "hand_prim": hand_prim,
                        "object_prim": object_prim,
                    },
                }
            else:
                self._manipulation_result = {
                    "success": False,
                    "message": f"Too far: {distance:.2f}m > {control.distance_threshold}m",
                    "data": {
                        "distance": distance,
                        "ready_to_grasp": False,
                    },
                }

        elif control.action == ControlAction.ATTACH:
            # Get world from container
            container = get_container()
            world = container.world_configured()

            # Initialize rigid prims
            hand_prim = RigidPrim(prim_paths_expr=control.hand_prim_path)
            object_prim = RigidPrim(prim_paths_expr=control.object_prim_path)

            # Stop object motion
            object_prim.set_linear_velocities(torch.zeros(3, dtype=torch.float32))
            object_prim.set_angular_velocities(torch.zeros(3, dtype=torch.float32))

            # Move object to hand position
            hand_pos, _ = hand_prim.get_world_poses()
            object_prim.set_world_poses(positions=hand_pos)

            # Disable collision
            world.set_collision_enabled(
                prim_path=control.object_prim_path, enabled=False
            )

            # Create or update joint
            joint_path = f"/World/grasp_joint_{object_prim.name}"
            stage = world.get_stage()
            joint_prim = stage.GetPrimAtPath(joint_path)

            if not joint_prim.IsValid():
                world.create_joint(
                    joint_path=joint_path,
                    joint_type="fixed",
                    body0=control.hand_prim_path,
                    body1=control.object_prim_path,
                    local_pos_0=control.local_pos_hand,
                    local_pos_1=control.local_pos_object,
                    axis=control.axis,
                )
                joint_prim = stage.GetPrimAtPath(joint_path)

            # Enable joint
            joint = UsdPhysics.Joint(joint_prim)
            joint.GetLocalPos0Attr().Set(Gf.Vec3f(control.local_pos_hand))
            joint.GetLocalPos1Attr().Set(Gf.Vec3f(control.local_pos_object))
            joint.GetJointEnabledAttr().Set(True)

            # Mark as completed to prevent repeated execution
            control.mark_completed()
            self._manipulation_result = {
                "success": True,
                "message": "Object attached",
                "data": {
                    "joint_path": joint_path,
                },
            }

    def _execute_release_control(self, control):
        """
        Execute release control (detach object)

        One-shot action:
        - RELEASE: Disable joint and re-enable collision
        """
        from physics_engine.pxr_utils import UsdPhysics
        from containers import get_container
        from simulation.control import ControlAction

        if control.action != ControlAction.RELEASE:
            return

        # Get world from container
        container = get_container()
        world = container.world_configured()

        # Disable joint
        stage = world.get_stage()
        joint_prim = stage.GetPrimAtPath(control.joint_path)

        if joint_prim.IsValid():
            joint = UsdPhysics.Joint(joint_prim)
            joint.GetJointEnabledAttr().Set(False)

            # Re-enable collision
            world.set_collision_enabled(
                prim_path=control.object_prim_path, enabled=True
            )

            # Mark as completed to prevent repeated execution
            control.mark_completed()

            self._manipulation_result = {
                "success": True,
                "message": "Object released",
                "data": {},
            }
        else:
            control.mark_failed()
            self._manipulation_result = {
                "success": False,
                "message": f"Joint not found: {control.joint_path}",
                "data": {},
            }

    def _execute_place_control(self, control):
        """
        Execute place control (place object at target location)

        One-shot action:
        - PLACE: Set object position and disable joint
        """
        from physics_engine.isaacsim_utils import RigidPrim
        from physics_engine.pxr_utils import UsdPhysics
        from containers import get_container
        from simulation.control import ControlAction

        if control.action != ControlAction.PLACE:
            return

        try:
            # Get world from container
            container = get_container()
            world = container.world_configured()

            # Get object prim
            object_prim = RigidPrim(prim_paths_expr=control.object_prim_path)

            # Disable joint first
            stage = world.get_stage()
            joint_prim = stage.GetPrimAtPath(control.joint_path)

            if joint_prim.IsValid():
                joint = UsdPhysics.Joint(joint_prim)
                joint.GetJointEnabledAttr().Set(False)

            # Set object position
            position = torch.tensor(control.target_location, dtype=torch.float32)
            object_prim.set_world_poses(positions=position)

            # Set velocity to zero (gentle placement)
            if control.gentle:
                object_prim.set_linear_velocities(torch.zeros(3, dtype=torch.float32))
                object_prim.set_angular_velocities(torch.zeros(3, dtype=torch.float32))

            # Re-enable collision
            world.set_collision_enabled(
                prim_path=control.object_prim_path, enabled=True
            )

            # Mark as completed
            control.mark_completed()

            self._manipulation_result = {
                "success": True,
                "message": "Object placed",
                "data": {
                    "position": control.target_location,
                },
            }
        except Exception as e:
            control.mark_failed()
            self._manipulation_result = {
                "success": False,
                "message": f"Place failed: {str(e)}",
                "data": {},
            }

    def update_sim_time(self, sim_time):
        """更新仿真时间"""
        self.sim_time = sim_time
