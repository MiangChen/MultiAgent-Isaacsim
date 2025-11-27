from typing import List, Optional, Dict, TYPE_CHECKING
from physics_engine.isaacsim_utils import World as IsaacWorld

if TYPE_CHECKING:
    from simulation.actor import Actor


class World:

    def __init__(
        self,
        simulation_app,
        physics_dt: float = 1.0 / 60.0,
        rendering_dt: float = 1.0 / 60.0,
        stage_units_in_meters: float = 1.0,
        sim_params: dict = None,
        backend: str = "torch",
    ):
        self._simulation_app = simulation_app
        self._isaac_world = IsaacWorld(
            physics_dt=physics_dt,
            rendering_dt=rendering_dt,
            stage_units_in_meters=stage_units_in_meters,
            sim_params=sim_params,
            backend=backend,
        )
        self._actors: Dict[int, "Actor"] = {}
        self._next_actor_id = 1
        self._blueprint_library = None
        self._scene_manager = None
        self._semantic_map = None

    def tick(self):
        self._isaac_world.step(render=True)

    def reset(self):
        self._isaac_world.reset()

    def get_actors(self) -> List["Actor"]:
        """Get all actors in the world"""
        return list(self._actors.values())

    def get_actor(self, actor_id: int) -> Optional["Actor"]:
        """Get actor by ID"""
        return self._actors.get(actor_id)

    def find_actor_by_robot(self, robot) -> Optional["Actor"]:
        """Find actor by Robot instance"""
        return getattr(robot, "actor", None)

    def find_sensors_by_parent(self, parent_actor) -> List["Actor"]:
        """
        Find all sensors attached to a parent actor (CARLA style)

        Args:
            parent_actor: Parent actor (RobotActor or other)

        Returns:
            List of sensor actors attached to the parent
        """
        from simulation.sensor_actor import SensorActor

        sensors = []
        for actor in self._actors.values():
            if isinstance(actor, SensorActor):
                if actor.get_parent() == parent_actor:
                    sensors.append(actor)
        return sensors

    def find_sensor_by_type(self, parent_actor, sensor_type: str) -> Optional["Actor"]:
        """
        Find sensor by type attached to a parent actor

        Args:
            parent_actor: Parent actor
            sensor_type: Sensor type ID (e.g., 'sensor.camera.rgb', 'sensor.lidar.ray_cast')

        Returns:
            First matching sensor actor or None
        """
        sensors = self.find_sensors_by_parent(parent_actor)
        for sensor in sensors:
            if sensor.get_type_id() == sensor_type:
                return sensor
        return None

    def spawn_actor(self, blueprint, transform=None, attach_to=None):
        """
        Spawn an actor from blueprint (CARLA style)

        Args:
            blueprint: Blueprint object (robot, static prop, vehicle, sensor, etc.)
            transform: Transform object for initial position/rotation
            attach_to: Parent actor to attach to (required for sensor)

        Returns:
            Actor: RobotActor, StaticActor, or Sensor
        """
        # Sensors must be attached to a parent
        if blueprint.has_tag("sensor"):
            if attach_to is None:
                raise ValueError(
                    "Sensors must be attached to a parent actor. "
                    "Please provide 'attach_to' parameter."
                )
            return self._spawn_sensor(blueprint, transform, attach_to)

        # Use tags to determine actor type (more flexible than checking robot_class)
        if blueprint.has_tag("static"):
            return self._spawn_static_prop(blueprint, transform)

        # Dynamic actors (robots, vehicles, drones, etc.)
        if blueprint.has_tag("robot"):
            return self._spawn_robot(blueprint, transform)

        # Fallback: if no tags match, treat as static
        return self._spawn_static_prop(blueprint, transform)

    def _spawn_robot(self, blueprint, transform):
        """创建机器人 Spawn robot actor"""
        actor_config = blueprint.get_all_attributes()

        if transform is not None:
            if transform.location is not None:
                actor_config["position"] = transform.location.to_list()
            if transform.rotation is not None:
                actor_config["orientation"] = transform.rotation.to_quaternion()

        # Instantiate the robot class
        if blueprint.robot_class is None:
            raise ValueError(
                f"Blueprint {blueprint.id} has 'robot' tag but no robot_class defined"
            )

        robot_instance = blueprint.robot_class(cfg_robot=actor_config)

        self.scene.add(robot_instance._body.robot_articulation)

        if self._semantic_map:
            self._semantic_map.dict_map_semantic[robot_instance.cfg_robot.namespace] = (
                robot_instance.cfg_robot.path_prim_robot
            )
            self._semantic_map.add_semantic(
                prim_path=robot_instance.cfg_robot.path_prim_robot,
                semantic_label="robot",
            )

        from simulation.robot_actor import RobotActor

        actor = RobotActor(robot_instance, world=self)

        return actor

    def _spawn_static_prop(self, blueprint, transform):
        """创建静态物体 Spawn static prop (CARLA style)"""
        attrs = blueprint.get_all_attributes()

        # Generate prim_path if not provided
        if "prim_path" not in attrs:
            name = attrs.get("name", f"prop_{id(blueprint)}")
            # Sanitize name for USD prim path (replace invalid characters)
            name = name.replace("-", "_").replace(" ", "_")
            attrs["prim_path"] = f"/World/{name}"

        if transform:
            if transform.location is not None:
                attrs["position"] = [
                    transform.location.x,
                    transform.location.y,
                    transform.location.z,
                ]
            if transform.rotation is not None:
                attrs["orientation"] = transform.rotation.to_quaternion()

        # Extract semantic_label before passing to create_shape_unified
        semantic_label = attrs.pop("semantic_label", None)

        result = self._scene_manager.create_shape_unified(**attrs)

        if result.get("status") == "success":
            prim_path = result.get("prim_path")
            if self._semantic_map and semantic_label:
                self._semantic_map.add_semantic(
                    prim_path=prim_path, semantic_label=semantic_label
                )
            # Create StaticActor wrapper
            from simulation.static_actor import StaticActor

            actor = StaticActor(prim_path, world=self, semantic_label=semantic_label)
            return actor

        return None

    def get_blueprint_library(self):
        if self._blueprint_library is None:
            from simulation.actor_blueprint import BlueprintLibrary

            self._blueprint_library = BlueprintLibrary()
        return self._blueprint_library

    def load_actors_from_config(self, config_path: str) -> List:
        import yaml

        with open(config_path, "r") as file:
            config = yaml.safe_load(file)

        actors = []
        blueprint_library = self.get_blueprint_library()

        for robot_type, robot_configs in config.items():
            bp = blueprint_library.find(f"robot.{robot_type}")
            if not bp:
                raise ValueError(f"Unknown robot type: {robot_type}")

            for cfg in robot_configs:
                for key, value in cfg.items():
                    bp.set_attribute(key, value)
                actor = self.spawn_actor(bp)
                actors.append(actor)

        return actors

    def register_actor(self, actor: "Actor") -> int:
        actor_id = self._next_actor_id
        self._next_actor_id += 1
        self._actors[actor_id] = actor
        return actor_id

    def unregister_actor(self, actor_id: int):
        if actor_id in self._actors:
            del self._actors[actor_id]

    def get_isaac_world(self):
        return self._isaac_world

    @property
    def scene(self):
        return self._isaac_world.scene

    def get_rendering_dt(self) -> float:
        return self._isaac_world.get_rendering_dt()

    def get_physics_dt(self) -> float:
        return self._isaac_world.get_physics_dt()

    def is_playing(self) -> bool:
        return self._isaac_world.is_playing()

    def play(self):
        self._isaac_world.play()

    def pause(self):
        self._isaac_world.pause()

    def stop(self):
        self._isaac_world.stop()

    def set_scene_manager(self, scene_manager):
        self._scene_manager = scene_manager

    def set_semantic_map(self, semantic_map):
        self._semantic_map = semantic_map


    def get_scene_manager(self):
        return self._scene_manager

    def get_semantic_map(self):
        return self._semantic_map

    def initialize_robots(self):
        for actor in self.get_actors():
            if hasattr(actor, "robot") and hasattr(actor.robot, "initialize"):
                actor.robot.initialize()


    # ============================================================================
    # Physics Operations (for Application Layer)
    # ============================================================================

    def get_stage(self):
        """
        Get USD stage (CARLA style)

        Returns:
            Usd.Stage: USD stage object
        """
        if self._scene_manager:
            return self._scene_manager.stage

        # Fallback: get stage directly
        import omni.usd

        return omni.usd.get_context().get_stage()

    def create_joint(
        self, joint_path, joint_type, body0, body1, local_pos_0, local_pos_1, axis
    ):
        """
        Create physics joint (CARLA style)

        Args:
            joint_path: Path for the joint prim
            joint_type: Type of joint ('fixed', 'revolute', etc.)
            body0: First body prim path
            body1: Second body prim path
            **kwargs: Additional joint parameters

        Returns:
            Result dictionary
        """
        if not self._scene_manager:
            raise RuntimeError("Scene manager not available")

        return self._scene_manager.create_joint(
            joint_path=joint_path,
            joint_type=joint_type,
            body0=body0,
            body1=body1,
            local_pos0=local_pos_0,
            local_pos1=local_pos_1,
            axis=axis,
        )

    def remove_joint(self, joint_path):
        """
        Remove physics joint (CARLA style)

        Args:
            joint_path: Path of the joint prim to remove
        """
        stage = self.get_stage()
        if stage:
            joint_prim = stage.GetPrimAtPath(joint_path)
            if joint_prim.IsValid():
                stage.RemovePrim(joint_path)

    def set_collision_enabled(self, prim_path, enabled=True):
        """
        Enable/disable collision for a prim (CARLA style)

        Args:
            prim_path: Path of the prim
            enabled: True to enable collision, False to disable

        Returns:
            Result dictionary
        """
        if not self._scene_manager:
            raise RuntimeError("Scene manager not available")

        return self._scene_manager.set_collision_enabled(
            prim_path=prim_path, collision_enabled=enabled
        )

    def overlap_test(self, prim_path):
        """
        Test for overlapping objects (CARLA style)

        Args:
            prim_path: Path of the prim to test

        Returns:
            Overlap test result
        """
        if not self._scene_manager:
            return None

        return self._scene_manager.overlap_hits_target_ancestor(prim_path)

    # ============================================================================
    # Sensor Operations (CARLA Style)
    # ============================================================================

    def _spawn_sensor(self, blueprint, transform, parent_actor):
        """
        Spawn sensor and attach to parent (CARLA style)

        Args:
            blueprint: Sensor blueprint
            transform: Relative transform
            parent_actor: Parent actor (robot)

        Returns:
            Sensor actor
        """
        # 1. Auto-construct sensor path
        sensor_type = blueprint.id.replace(".", "_")
        sensor_name = f"{sensor_type}_{self._next_actor_id}"
        sensor_path_relative = f"/sensor/{sensor_name}"

        # 2. Extract relative transform
        translation, quaternion = self._extract_transform(transform)

        # 3. Create sensor implementation (Isaac Sim layer)
        sensor_impl = self._create_sensor_impl(sensor_path_relative, translation, quaternion, blueprint, parent_actor)

        # 4. Create sensor actor (abstraction layer)
        sensor_actor = blueprint.sensor_class(sensor_impl, self, parent_actor)

        # 5. Register physics callback
        self._isaac_world.add_physics_callback(
            f"sensor_{sensor_actor.get_id()}", sensor_actor.tick
        )

        return sensor_actor

    def _extract_transform(self, transform):
        """
        Extract translation and quaternion from Transform

        Args:
            transform: Transform object

        Returns:
            (translation, quaternion) tuple
        """
        if transform is None:
            return [0, 0, 0], [1, 0, 0, 0]

        # 处理 location
        if transform.location is not None:
            translation = [
                transform.location.x,
                transform.location.y,
                transform.location.z,
            ]
        else:
            translation = [0, 0, 0]

        # 处理 rotation
        if transform.rotation is not None:
            quaternion = transform.rotation.to_quaternion()
        else:
            # 默认单位四元数 [x, y, z, w]
            quaternion = [0, 0, 0, 1]

        # Isaac Sim uses [w, x, y, z], but to_quaternion returns [x, y, z, w]
        # Reorder to [w, x, y, z]
        quaternion = [quaternion[3], quaternion[0], quaternion[1], quaternion[2]]

        return translation, quaternion

    def _create_sensor_impl(self, sensor_path_relative, translation, quaternion, blueprint, parent_actor=None):
        """
        Create sensor implementation (Isaac Sim layer)

        Args:
            sensor_path_relative: Auto-generated sensor path
            translation: Relative position
            quaternion: Relative orientation [w, x, y, z]
            blueprint: Sensor blueprint
            parent_actor: Parent actor (for sensors that need it)

        Returns:
            Sensor implementation (Camera, LidarIsaac, or LidarOmni)
        """
        if blueprint.id == "sensor.camera.rgb":
            return self._create_camera_impl(sensor_path_relative, translation, quaternion, blueprint, parent_actor)
        elif blueprint.id == "sensor.lidar.isaac":
            return self._create_lidar_isaac_impl(sensor_path_relative, translation, quaternion, blueprint)
        elif blueprint.id == "sensor.lidar.omni":
            return self._create_lidar_omni_impl(sensor_path_relative, translation, quaternion, blueprint, parent_actor)
        else:
            raise ValueError(f"Unknown sensor type: {blueprint.id}")

    def _create_camera_impl(self, sensor_path_relative, translation, quaternion, blueprint, parent_actor=None):
        """Create camera implementation using new xform + rigid body + fixed joint architecture"""
        from simulation.sensor.camera.camera import Camera
        from simulation.sensor.camera.cfg_camera import CfgCamera

        # Construct config
        cfg = CfgCamera(
            name=sensor_path_relative.split('/')[-1],
            type="rgb",
            id=self._next_actor_id,
            path_prim_relative=sensor_path_relative,
            resolution=(
                blueprint.get_attribute("image_size_x"),
                blueprint.get_attribute("image_size_y"),
            ),
            translation=translation,
            orientation=quaternion,
            focal_length=blueprint.get_attribute("focal_length"),
            enable_semantic_detection=blueprint.get_attribute(
                "enable_semantic_detection"
            ),
            attach_prim_relative_path=blueprint.get_attribute("attach_prim_relative_path", None),
            use_existing_camera=False,  # 创建新 camera，使用 xform + joint 架构
        )

        # Get parent prim path from parent actor
        if parent_actor is not None:
            parent_prim_path = parent_actor.get_prim_path()
        else:
            # Fallback: extract from sensor path
            parent_prim_path = "/".join(sensor_path_relative.split("/")[:-2])

        # Create camera with new architecture
        camera = Camera(path_prim_parent=parent_prim_path, cfg_camera=cfg)
        camera.initialize_on_physics_step()

        return camera

    def _create_lidar_isaac_impl(self, sensor_path_relative, translation, quaternion, blueprint):
        """Create Isaac LiDAR implementation"""
        from simulation.sensor.lidar.lidar_isaac import LidarIsaac
        from simulation.sensor.lidar.cfg_lidar import CfgLidar
        from simulation.robot import CfgRobot

        # Construct LiDAR config
        cfg_lidar = CfgLidar(
            name=sensor_path_relative.split("/")[-1],
            prim_path_relative=sensor_path_relative,
            translation=translation,
            quat=quaternion,
            config_file_name=blueprint.get_attribute("config_file_name"),
            frequency=blueprint.get_attribute("frequency", 10),
        )

        # Create dummy robot config (LidarIsaac requires it)
        cfg_robot = CfgRobot()
        cfg_robot.path_prim_swarm = "/".join(sensor_path_relative.split("/")[:-2])

        # Create LiDAR
        lidar = LidarIsaac(cfg_lidar, cfg_robot)
        lidar.create_lidar(prim_path=sensor_path_relative)
        lidar.initialize()

        return lidar

    def _create_lidar_omni_impl(self, sensor_path_relative, translation, quaternion, blueprint, parent_actor=None):
        """Create Omni LiDAR implementation"""
        from simulation.sensor.lidar.lidar_omni import LidarOmni
        from simulation.sensor.lidar.cfg_lidar import CfgLidar

        # Construct LiDAR config
        cfg_lidar = CfgLidar(
            name=sensor_path_relative.split("/")[-1],
            prim_path_relative=sensor_path_relative,
            translation=translation,
            quat=quaternion,
            config_file_name=blueprint.get_attribute("config_file_name"),
            output_size=blueprint.get_attribute("output_size"),
            max_depth=blueprint.get_attribute("max_depth", 1000.0),
            erp_width=blueprint.get_attribute("erp_width"),
            erp_height=blueprint.get_attribute("erp_height"),
            erp_width_fov=blueprint.get_attribute("erp_width_fov", 90.0),
            erp_height_fov=blueprint.get_attribute("erp_height_fov", 270.0),
            frequency=blueprint.get_attribute("frequency", 10),
            attach_prim_relative_path=blueprint.get_attribute("attach_prim_relative_path", None),
        )

        # Get parent prim path from parent actor
        parent_prim_path = parent_actor.get_prim_path()

        # Create LiDAR with parent_prim_path
        lidar = LidarOmni(cfg_lidar, parent_prim_path)
        lidar.initialize_on_physics_step()

        return lidar

    def get_frame(self) -> int:
        """Get current frame number"""
        return self._isaac_world.current_time_step_index

    def get_simulation_time(self) -> float:
        """Get simulation time in seconds"""
        return self._isaac_world.current_time
