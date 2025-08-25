# Standard library imports
import argparse
import asyncio
import logging
import os
from itertools import count
from typing import Dict, Any

import os, threading, queue, argparse, yaml
from collections import defaultdict, deque
import numpy as np

# Third-party imports
import yaml

import matplotlib
matplotlib.use('TkAgg')

# Isaac Sim related imports
from physics_engine.isaacsim_simulation_app import initialize_simulation_app_from_yaml

# Initialize simulation app
parser = argparse.ArgumentParser(description="Initialize Isaac Sim from a YAML config.")
parser.add_argument("--config", type=str, default="./files/sim_cfg.yaml",
                    help="Path to the configuration physics engine.")
parser.add_argument("--enable", type=str, action='append',
                    help="Enable a feature. Can be used multiple times.")
args = parser.parse_args()

simulation_app = initialize_simulation_app_from_yaml(args.config)

# Suppress specific Isaac Sim warnings
# logging.getLogger("omni.syntheticdata.plugin").setLevel(logging.ERROR)

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
            robot_swarm_cfg_path=f"{PATH_PROJECT}/files/robot_swarm_cfg_empty.yaml",
            robot_active_flag_path=f"{PATH_PROJECT}/files/robot_swarm_active_flag_empty.yaml"
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
    
    save_dir = os.path.abspath("./saved_scenes")  # Use absolute path
    
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


def process_semantic_detection(semantic_camera, map_semantic: MapSemantic, count: int, bounding_box_enabled: bool) -> None:
    """
    Process semantic detection and car pose extraction.
    
    Args:
        semantic_camera: Semantic camera instance
        map_semantic: Semantic map instance
        count: Current frame count
        bounding_box_enabled: Whether bounding box detection is enabled
    """
    if count % 120 == 0 and bounding_box_enabled:
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


if __name__ == "__main__":


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
    swarm_manager = SwarmManager(map_grid)
    scene_manager = SceneManager()
    
    # Load scene
    scene_manager.load_scene(usd_path=WORLD_USD_PATH)
    
    # Create car objects
    created_prim_paths = create_car_objects(scene_manager)
    print("All prims with 'car' label:", created_prim_paths)
    print(scene_manager.count_semantics_in_scene().get('result'))

    try:
        # Setup simulation environment
        loop = asyncio.get_event_loop()
        env = loop.run_until_complete(setup_simulation(simulation_app, swarm_manager, cfg, scene_manager, map_grid))

        # Reset environment to ensure all objects are properly initialized
        env.reset()
        
        # Create and initialize semantic camera after env.reset()
        result = scene_manager.add_semantic_camera(
            prim_path='/World/semantic_camera', 
            position=[0, 4, 2], 
            quat=scene_manager.euler_to_quaternion(roll=90)
        )
        semantic_camera = result.get('result').get('camera_instance')
        semantic_camera_prim_path = result.get('result').get('prim_path')
        
        # Initialize camera without immediately adding bounding box functionality
        semantic_camera.initialize()
        
        # Wait for frames to ensure camera and render pipeline are fully initialized
        for _ in range(10):
            env.step(action=None)
        
        # Flag for delayed bounding box detection enabling
        bounding_box_enabled = False
        
        # Switch viewport to semantic camera
        scene_manager.change_viewport(prim_path=semantic_camera_prim_path)

        # Build map for future planning
        map_grid.generate_grid_map('2d')

        # Save current scene (after env reset and swarm creation)
        save_scenes(scene_manager)

        print("--- Initializing experiment plan and semantic map ---")

        count = 0
        # Main simulation loop
        while simulation_app.is_running():
            # World step
            env.step(action=None)

            # Enable bounding box detection after simulation runs for a while to avoid initialization warnings
            if count == 60 and not bounding_box_enabled:  # Enable after 60 frames
                try:
                    semantic_camera.add_bounding_box_2d_loose_to_frame()
                    bounding_box_enabled = True
                    print("Semantic camera bounding box detection enabled successfully")
                except Exception as e:
                    print(f"Warning: Failed to enable bounding box detection: {e}")
                    print("Continuing without bounding box detection...")

            # Process semantic detection only after bounding box detection is enabled
            process_semantic_detection(semantic_camera, map_semantic, count, bounding_box_enabled)
            count += 1

    finally:
        # Close simulation safely
        print("--- Simulation finished. Manually closing application. ---")
        if simulation_app:
            simulation_app.__exit__(None, None, None)