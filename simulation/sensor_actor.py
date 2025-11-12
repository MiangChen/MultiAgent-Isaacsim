"""
Sensor Actor Module - CARLA Style Sensor Implementation

Provides SensorActor wrapper for sensors, following the same pattern as RobotActor.
All sensors are wrapped in SensorActor and support listen/stop callbacks.
"""

from abc import ABC, abstractmethod
from typing import Callable, Any, Optional
from simulation.actor import Actor
from simulation.transform import Transform, Location, Vector3D


class SensorActor(Actor, ABC):
    """
    Sensor Actor base class (CARLA style)

    Wraps Isaac Sim sensor implementations (Camera, LidarIsaac, etc.)
    Follows the same pattern as RobotActor:
    - RobotActor wraps Robot
    - SensorActor wraps Sensor implementation

    All sensors inherit from Actor and can be spawned/destroyed.
    Provides unified listen/stop interface for data callbacks.
    """

    def __init__(self, sensor_impl: Any, world, parent: Actor = None):
        """
        Initialize sensor actor

        Args:
            sensor_impl: Isaac Sim sensor implementation (Camera, LidarIsaac, etc.)
            world: World instance
            parent: Parent actor (sensor is attached to)
        """
        # Store sensor implementation (like RobotActor stores robot)
        self.sensor = sensor_impl
        self._parent = parent

        # Get prim path from sensor implementation
        self._prim_path = self._get_prim_path_from_impl(sensor_impl)

        # Initialize Actor base class (like RobotActor does)
        self._world = world
        self._actor_id = world.register_actor(self) if world else None

        # Callback management (CARLA only supports one callback)
        self._callback = None
        self._is_listening = False

        # Bidirectional reference (like Robot <-> RobotActor)
        if hasattr(sensor_impl, "actor"):
            sensor_impl.actor = self

    def _get_prim_path_from_impl(self, sensor_impl) -> str:
        """Extract prim path from sensor implementation"""
        if hasattr(sensor_impl, "cfg_camera"):
            return sensor_impl.cfg_camera.path_prim_absolute
        elif hasattr(sensor_impl, "cfg_lidar"):
            return sensor_impl.cfg_lidar.prim_path
        else:
            raise ValueError("Unknown sensor implementation type")

    def listen(self, callback: Callable):
        """
        Register callback function (CARLA style)

        Args:
            callback: Callback function that receives sensor data

        Note: CARLA only supports one callback. New callback replaces old one.
        """
        self._callback = callback
        self._is_listening = True

    def stop(self):
        """Stop listening (CARLA style)"""
        self._is_listening = False

    def is_listening(self) -> bool:
        """Check if sensor is listening"""
        return self._is_listening

    def get_parent(self) -> Optional[Actor]:
        """Get parent actor"""
        return self._parent

    @abstractmethod
    def tick(self, step_size: float):
        """
        Physics step callback (internal method)

        Subclasses implement: fetch data and trigger callback

        Args:
            step_size: Physics time step
        """
        pass

    def _trigger_callback(self, data: Any):
        """
        Trigger callback with sensor data

        Args:
            data: Sensor data object
        """
        if not self._is_listening or self._callback is None:
            return

        try:
            self._callback(data)
        except Exception as e:
            from log.log_manager import LogManager

            logger = LogManager.get_logger(__name__)
            logger.error(f"Sensor callback error: {e}")

    def destroy(self):
        """Destroy sensor (CARLA style)"""
        self.stop()
        self._callback = None
        super().destroy()

    def get_transform(self) -> Transform:
        """Get sensor world transform (like RobotActor.get_transform)"""
        # Get from sensor implementation
        if hasattr(self.sensor, "get_world_pose"):
            pos, quat = self.sensor.get_world_pose()
            if hasattr(pos, "__iter__"):
                return Transform(location=Location(pos[0], pos[1], pos[2]))
            else:
                return Transform(location=Location(pos.x, pos.y, pos.z))
        return Transform()

    def set_transform(self, transform: Transform):
        """Set sensor transform (not supported for attached sensors)"""
        raise NotImplementedError("Cannot set transform for attached sensors")

    def get_velocity(self) -> Vector3D:
        """Get sensor velocity (follows parent, like RobotActor.get_velocity)"""
        if self._parent:
            return self._parent.get_velocity()
        return Vector3D()
