#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Standard library imports
import argparse
import asyncio
import logging
import os
import threading
from typing import Dict, Any
from collections import defaultdict, deque

# Third-party imports
import matplotlib
matplotlib.use('TkAgg')
import yaml

# Isaac Sim related imports
from physics_engine.isaacsim_simulation_app import initialize_simulation_app_from_yaml

# Initialize simulation app
parser = argparse.ArgumentParser(description="Initialize Isaac Sim from a YAML config.")
parser.add_argument("--config", type=str, default="./files/sim_cfg.yaml",
                    help="Path to the configuration physics engine.")
parser.add_argument("--enable", type=str, action='append',
                    help="Enable a feature. Can be used multiple times.")
parser.add_argument("--ros", type=bool, default=True)
args = parser.parse_args()

simulation_app = initialize_simulation_app_from_yaml(args.config)

# Local imports
from environment.env import Env
from files.variables import WORLD_USD_PATH, PATH_PROJECT
from map.map_grid_map import GridMap
from map.map_semantic_map import MapSemantic
from robot.robot_cf2x import RobotCf2x, RobotCfgCf2x
from robot.robot_h1 import RobotH1, RobotCfgH1
from robot.robot_jetbot import RobotCfgJetbot, RobotJetbot
from robot.swarm_manager import SwarmManager
from scene.scene_manager import SceneManager

# ROS 2 imports (optional, only if ROS is available)
try:
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from ros.ros_swarm import BaseNode, SceneMonitorNode
    from plan_msgs.msg import Parameter, SkillInfo, RobotSkill, Plan as PlanMsg, TimestepSkills

    ROS_AVAILABLE = True
except ImportError:
    print("ROS modules not available, running without ROS integration")
    ROS_AVAILABLE = False

# Global variables for ROS integration
_sem_map = None
_skill_queues = defaultdict(deque)
_skill_lock = threading.Lock()


def _param_dict(params):
    """Convert ROS parameter list to dictionary"""
    d = {}
    if params:
        for p in params:
            d[p.key] = p.value
    return d


def _parse_robot_id(robot_id: str):
    """Parse robot ID string to extract robot class and index"""
    import re
    s = robot_id or ""
    m = re.match(r"^([A-Za-z]\w*?)[_-]?(\d+)$", s)
    if not m:
        return "jetbot", 0
    name = m.group(1)
    idx = int(m.group(2))
    return name, idx


# Skill execution functions
def _skill_navigate_to(env, rc, rid, params):
    """Execute navigate-to skill"""
    pos = _sem_map.map_semantic[params["goal"]]
    env.robot_swarm.robot_active[rc][rid].navigate_to(pos)


def _skill_pick_up(env, rc, rid, params):
    """Execute pick-up skill"""
    env.robot_swarm.robot_active[rc][rid].pick_up()


def _skill_put_down(env, rc, rid, params):
    """Execute put-down skill"""
    env.robot_swarm.robot_active[rc][rid].put_down()


_SKILL_TABLE = {
    "navigate-to": _skill_navigate_to,
    "pick-up": _skill_pick_up,
    "put-down": _skill_put_down,
}


def build_ros_nodes():
    """Build ROS nodes for plan receiving and scene monitoring"""
    if not ROS_AVAILABLE:
        return None, None

    qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=50,
    )

    plan_receiver = BaseNode('plan_receiver')

    def _plan_cb(msg: PlanMsg):
        """Plan callback to queue skills for robots"""
        try:
            steps = sorted(msg.steps, key=lambda s: s.timestep)

            with _skill_lock:
                for ts in steps:
                    for rs in ts.robots:
                        rc, rid = _parse_robot_id(rs.robot_id)
                        q = _skill_queues[(rc, rid)]

                        for sk in rs.skill_list:
                            q.append(sk)

        except Exception as e:
            print(f"[PlanCB] Error: {e}")

    plan_receiver.create_subscription(PlanMsg, '/Plan', _plan_cb, qos)
    scene_monitor = SceneMonitorNode()
    return plan_receiver, scene_monitor


def spin_ros_in_background(nodes, stop_evt: threading.Event):
    """Run ROS nodes in background thread"""
    if not ROS_AVAILABLE or not nodes[0]:
        return

    exec_ = MultiThreadedExecutor(num_threads=4)
    for n in nodes:
        if n:
            exec_.add_node(n)
    try:
        while not stop_evt.is_set():
            exec_.spin_once(timeout_sec=0.05)
    finally:
        for n in nodes:
            if n:
                try:
                    exec_.remove_node(n)
                except:
                    pass
                try:
                    n.destroy_node()
                except:
                    pass
        if ROS_AVAILABLE:
            rclpy.shutdown()


async def setup_simulation(simulation_app, swarm_manager, cfg: Dict[str, Any], scene_manager, map_grid) -> Env:
    """
    Setup simulation environment with robot swarm manager.

    Args:
        simulation_app: Isaac Sim simulation application instance
        swarm_manager: Robot swarm manager instance
        cfg: Configuration dictionary loaded from YAML
        scene_manager: Scene manager instance
        map_grid: Grid map instance

    Returns:
        Env: Initialized environment instance
    """
    # Register robot classes to swarm manager
    swarm_manager.register_robot_class(
        robot_class_name="jetbot",
        robot_class=RobotJetbot,
        robot_class_cfg=RobotCfgJetbot,
    )
    swarm_manager.register_robot_class(
        robot_class_name="h1",
        robot_class=RobotH1,
        robot_class_cfg=RobotCfgH1
    )
    swarm_manager.register_robot_class(
        robot_class_name="cf2x",
        robot_class=RobotCf2x,
        robot_class_cfg=RobotCfgCf2x
    )

    # Create environment first
    env = await Env.create(
        simulation_app=simulation_app,
        physics_dt=cfg['world']['physics_dt'],
        swarm_manager=swarm_manager,
        scene_manager=scene_manager,
        grid_map=map_grid,
    )

    # Initialize swarm manager after environment is ready
    try:
        await swarm_manager.initialize_async(
            scene=env.world.scene,
            robot_swarm_cfg_path=f"{PATH_PROJECT}/files/robot_swarm_cfg.yaml",
            robot_active_flag_path=f"{PATH_PROJECT}/files/robot_swarm_active_flag.yaml"
        )
    except Exception as e:
        print(f"Error during swarm manager initialization: {e}")
        raise

    return env


def create_car_objects(scene_manager: SceneManager) -> list:
    """
    Create car objects in the scene with semantic labels.

    Args:
        scene_manager: Scene manager instance for creating objects

    Returns:
        list: List of created prim paths
    """
    scale = [2, 5, 1.0]
    cubes_config = {
        "car0": {
            "shape_type": "cuboid",
            "prim_path": "/World/car0",
            "size": scale,
            "position": [11.6, 3.5, 0],
            "color": [255, 255, 255],
            "make_dynamic": False,
        },
        "car1": {
            "shape_type": "cuboid",
            "prim_path": "/World/car1",
            "size": scale,
            "position": [0.3, 3.5, 0],
            "color": [255, 255, 255],
            "make_dynamic": False,
        },
        "car2": {
            "shape_type": "cuboid",
            "prim_path": "/World/car2",
            "size": scale,
            "position": [-13.2, 3.5, 0],
            "color": [255, 255, 255],
            "make_dynamic": False,
        },
        "car3": {
            "shape_type": "cuboid",
            "prim_path": "/World/car3",
            "size": scale,
            "position": [-7.1, 10, 0],
            "color": [255, 255, 255],
            "make_dynamic": False,
        },
        "car4": {
            "shape_type": "cuboid",
            "prim_path": "/World/car4",
            "size": scale,
            "position": [-0.9, 30, 0],
            "orientation": [0.707, 0, 0, 0.707],
            "color": [255, 255, 255],
            "make_dynamic": False,
        },
    }

    created_prim_paths = []
    print("All semantics in scene:", scene_manager.count_semantics_in_scene().get('result'))

    for cube_name, config in cubes_config.items():
        print(f"--- Processing: {cube_name} ---")

        # Create shape using unpacking
        creation_result = scene_manager.create_shape(**config)

        # Check the result
        if creation_result.get("status") == "success":
            prim_path = creation_result.get("result")
            print(f"  Successfully created prim at: {prim_path}")
            created_prim_paths.append(prim_path)

            # Add semantic label
            semantic_result = scene_manager.add_semantic(
                prim_path=prim_path,
                semantic_label='car'
            )

            if semantic_result.get("status") == "success":
                print(f"  Successfully applied semantic label 'car' to {prim_path}")
            else:
                print(f"  [ERROR] Failed to apply semantic label: {semantic_result.get('message')}")
        else:
            print(f"  [ERROR] Failed to create shape '{cube_name}': {creation_result.get('message')}")

    return created_prim_paths


def save_scenes(scene_manager: SceneManager) -> None:
    """
    Save current scene in both flattened and reference formats.

    Args:
        scene_manager: Scene manager instance for saving scenes
    """
    print("--- Saving current scene ---")

    save_dir = os.path.abspath("./saved_scenes")

    # Save flattened scene (complete with all assets)
    save_result_flat = scene_manager.save_scene(
        scene_name="current_scene_with_cars_flattened",
        save_directory=save_dir,
        flatten_scene=True
    )
    if save_result_flat.get("status") == "success":
        print(f"Flattened scene saved: {save_result_flat.get('message')}")
    else:
        print(f"Failed to save flattened scene: {save_result_flat.get('message')}")

    # Save reference version (smaller file)
    save_result_ref = scene_manager.save_scene(
        scene_name="current_scene_with_cars_references",
        save_directory=save_dir,
        flatten_scene=False
    )
    if save_result_ref.get("status") == "success":
        print(f"Reference scene saved: {save_result_ref.get('message')}")
    else:
        print(f"Failed to save reference scene: {save_result_ref.get('message')}")


def process_semantic_detection(semantic_camera, map_semantic: MapSemantic
                               ) -> None:
    """
    Process semantic detection and car pose extraction.

    Args:
        semantic_camera: Semantic camera instance
        map_semantic: Semantic map instance
        count: Current frame count
        bounding_box_enabled: Whether bounding box detection is enabled
"""
    try:
        current_frame = semantic_camera.get_current_frame()
        if current_frame and 'bounding_box_2d_loose' in current_frame:
            result = current_frame['bounding_box_2d_loose']
            print("get bounding box 2d loose", result)
            if result:
                car_prim, car_pose = map_semantic.get_prim_and_pose_by_semantic(result, 'car')
                if car_prim is not None and car_pose is not None:
                    print("get car prim and pose\n", car_prim, '\n', car_pose)
                else:
                    print("No car detected in current frame")
            else:
                print("No bounding box data available")
        else:
            print("No frame data or bounding box key available")
    except Exception as e:
        print(f"Error getting semantic camera data: {e}")


def process_ros_skills(env):
    """Process ROS skill queue and execute skills"""
    if not ROS_AVAILABLE:
        return

    # Check if all robots have completed their current skills
    state_skill_complete_all = True
    for robot_class in env.robot_swarm.robot_class:
        for robot in env.robot_swarm.robot_active[robot_class]:
            done = getattr(robot, "state_skill_complete", True)
            state_skill_complete_all = state_skill_complete_all and bool(done)

    if state_skill_complete_all:
        # All robots completed -> get next skill for each robot
        with _skill_lock:
            keys = list(_skill_queues.keys())

        for rc, rid in keys:
            with _skill_lock:
                dq = _skill_queues.get((rc, rid))
                next_skill = dq.popleft() if (dq and len(dq) > 0) else None
                if dq is not None and len(dq) == 0:
                    _skill_queues.pop((rc, rid), None)

            if next_skill is not None:
                name = next_skill.skill.strip().lower()
                params = _param_dict(next_skill.params)
                fn = _SKILL_TABLE.get(name)
                if fn is None:
                    print(f"[Scheduler] unsupported skill: {name}")
                else:
                    try:
                        fn(env, rc, rid, params)
                    except Exception as e:
                        print(f"[Scheduler] start skill error: {e}")


def main():
    """Main async function to run the simulation."""
    global _sem_map
    # Initialize ROS if requested and available
    ros_nodes = (None, None)
    stop_evt = None
    t_ros = None

    if args.ros and ROS_AVAILABLE:
        try:
            rclpy.init(args=None)
            ros_nodes = build_ros_nodes()
            stop_evt = threading.Event()
            t_ros = threading.Thread(
                target=spin_ros_in_background,
                args=(ros_nodes, stop_evt),
                daemon=True
            )
            t_ros.start()
            print("ROS integration enabled")
        except Exception as e:
            print(f"Failed to initialize ROS: {e}")
            args.ros = False

    try:
        # Load configuration
        with open('./files/env_cfg.yaml', 'r') as f:
            cfg = yaml.safe_load(f)

        # Create managers and components
        map_grid = GridMap(
            cell_size=cfg['map']['cell_size'],
            start_point=cfg['map']['start_point'],
            min_bounds=cfg['map']['min_bounds'],
            max_bounds=cfg['map']['max_bounds'],
            occupied_cell=cfg['map']['occupied_cell'],
            empty_cell=cfg['map']['empty_cell'],
            invisible_cell=cfg['map']['invisible_cell'],
        )

        map_semantic = MapSemantic()
        _sem_map = map_semantic  # Set global reference for ROS

        swarm_manager = SwarmManager(map_grid)
        scene_manager = SceneManager()

        # Load scene
        scene_manager.load_scene(usd_path=WORLD_USD_PATH)

        # Create car objects
        created_prim_paths = create_car_objects(scene_manager)
        print("All prims with 'car' label:", created_prim_paths)
        print(scene_manager.count_semantics_in_scene().get('result'))

        # Setup simulation with proper error handling
        try:
            # env = await setup_simulation(simulation_app, swarm_manager, cfg, scene_manager, map_grid)
            loop = asyncio.get_event_loop()
            env = loop.run_until_complete(setup_simulation(simulation_app, swarm_manager, cfg, scene_manager, map_grid))

        except Exception as e:
            print(f"Failed to setup simulation: {e}")
            raise

        # Reset environment to ensure all objects are properly initialized
        env.reset()

        # Add physics callbacks for active robots
        for robot_class in swarm_manager.robot_class:
            for i, robot in enumerate(swarm_manager.robot_active[robot_class]):
                callback_name = f"physics_step_{robot_class}_{i}"
                env.world.add_physics_callback(callback_name, callback_fn=robot.on_physics_step)

        # Create and initialize semantic camera
        result = scene_manager.add_semantic_camera(
            prim_path='/World/semantic_camera',
            position=[0, 4, 2],
            quat=scene_manager.euler_to_quaternion(roll=90)
        )
        semantic_camera = result.get('result').get('camera_instance')
        semantic_camera_prim_path = result.get('result').get('prim_path')

        # Initialize camera
        semantic_camera.initialize()

        # Wait for camera and rendering pipeline to fully initialize
        for _ in range(60):
            env.step(action=None)

        # Enable bounding box detection after initialization period
        semantic_camera.add_bounding_box_2d_loose_to_frame()

        # Switch viewport to semantic camera
        scene_manager.change_viewport(prim_path=semantic_camera_prim_path)

        # Build grid map for planning
        map_grid.generate_grid_map('2d')

        # Save current scene
        # save_scenes(scene_manager)

        print("--- Initializing experiment plan and semantic map ---")

        count = 0
        # Main simulation loop
        while simulation_app.is_running():
            # World step
            env.step(action=None)

            if count % 120 == 0 and count > 0:
                process_semantic_detection(semantic_camera, map_semantic)

            # Process ROS skills if ROS is enabled
            if args.ros:
                process_ros_skills(env)

            count += 1

    except Exception as e:
        print(f"Error in main execution: {e}")
        raise
    finally:
        # Clean shutdown
        if stop_evt:
            stop_evt.set()
        if t_ros:
            t_ros.join(timeout=1.0)

        print("--- Simulation finished. Manually closing application. ---")
        if simulation_app:
            simulation_app.__exit__(None, None, None)


if __name__ == "__main__":
    main()
