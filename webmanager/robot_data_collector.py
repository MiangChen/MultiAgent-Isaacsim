"""
Robot position data collection implementation for the WebManager.
Handles real-time robot position tracking, status monitoring, and data formatting.
"""

import time
import logging
from typing import Dict, List, Optional, Any
import numpy as np
from pathlib import Path
import yaml

from .models import RobotPosition, RobotPositions, RobotStatus
from .data_collector import RobotDataSource

logger = logging.getLogger(__name__)


class SwarmManagerRobotDataSource(RobotDataSource):
    """
    Robot data source that integrates with the existing swarm manager.
    Provides real-time robot position tracking and status monitoring.
    """
    
    def __init__(self, swarm_manager=None, config_path: Optional[str] = None):
        """
        Initialize the robot data source.
        
        Args:
            swarm_manager: Reference to the swarm manager instance
            config_path: Path to robot configuration file
        """
        self.swarm_manager = swarm_manager
        self.config_path = config_path or "config/robot_swarm_cfg.yaml"
        
        # Robot tracking data
        self.robot_positions: RobotPositions = {}
        self.robot_status_cache: Dict[str, RobotStatus] = {}
        self.last_update_time: Dict[str, float] = {}
        
        # Configuration
        self.position_update_threshold = 0.01  # Minimum movement to trigger update
        self.status_timeout = 5.0  # Seconds before marking robot as inactive
        
        # Load robot configuration
        self._load_robot_config()
        
        logger.info(f"SwarmManagerRobotDataSource initialized with {len(self.robot_positions)} robots")
    
    def _load_robot_config(self):
        """Load robot configuration from YAML file."""
        try:
            config_file = Path(self.config_path)
            if config_file.exists():
                with open(config_file, 'r') as f:
                    config = yaml.safe_load(f)
                
                # Initialize robots from config
                robots = config.get('robots', {})
                for robot_id, robot_config in robots.items():
                    initial_pos = robot_config.get('initial_position', [0, 0, 0])
                    
                    self.robot_positions[robot_id] = RobotPosition(
                        x=float(initial_pos[0]),
                        y=float(initial_pos[1]),
                        z=float(initial_pos[2]) if len(initial_pos) > 2 else 0.0,
                        orientation={'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
                        status=RobotStatus.INACTIVE,
                        battery_level=100.0
                    )
                    
                    self.robot_status_cache[robot_id] = RobotStatus.INACTIVE
                    self.last_update_time[robot_id] = 0.0
                
                logger.info(f"Loaded {len(self.robot_positions)} robots from config")
            else:
                logger.warning(f"Robot config file not found: {config_file}")
                
        except Exception as e:
            logger.error(f"Error loading robot config: {e}")
    
    def get_data(self) -> RobotPositions:
        """Get current robot position data."""
        return self.get_robot_positions()
    
    def is_available(self) -> bool:
        """Check if the robot data source is available."""
        return self.swarm_manager is not None or len(self.robot_positions) > 0    

    def get_robot_positions(self) -> RobotPositions:
        """
        Get current robot positions and status.
        Integrates with swarm manager and updates robot status based on activity.
        """
        current_time = time.time()
        
        # Update positions from swarm manager if available
        if self.swarm_manager:
            self._update_from_swarm_manager()
        
        # Update robot status based on last activity
        self._update_robot_status(current_time)
        
        # Return copy of current positions
        return self.robot_positions.copy()
    
    def get_robot_count(self) -> int:
        """Get the number of tracked robots."""
        return len(self.robot_positions)
    
    def _update_from_swarm_manager(self):
        """Update robot positions from the swarm manager."""
        try:
            # Access robots from swarm manager's robot_active dictionary
            if hasattr(self.swarm_manager, 'robot_active'):
                for robot_class_name, robots in self.swarm_manager.robot_active.items():
                    for i, robot in enumerate(robots):
                        # Create unique robot ID
                        robot_id = f"{robot_class_name}_{i}"
                        
                        # Get robot position using get_world_pose method
                        if hasattr(robot, 'get_world_pose'):
                            world_pose = robot.get_world_pose()
                            if world_pose and len(world_pose) >= 1:
                                position = world_pose[0]  # Position is first element
                                orientation_quat = world_pose[1] if len(world_pose) > 1 else None
                                
                                # Convert quaternion to euler angles if available
                                orientation = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
                                if orientation_quat is not None:
                                    # Simple yaw extraction from quaternion (w, x, y, z)
                                    if len(orientation_quat) >= 4:
                                        w, x, y, z = orientation_quat
                                        yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
                                        orientation['yaw'] = float(yaw)
                                
                                # Get robot status
                                status = 'active' if getattr(robot, 'flag_active', False) else 'inactive'
                                
                                # Get battery level if available
                                battery_level = getattr(robot, 'battery_level', 100.0)
                                
                                position_data = {
                                    'position': position,
                                    'orientation': orientation,
                                    'battery_level': battery_level,
                                    'status': status
                                }
                                
                                self._update_robot_position(robot_id, position_data)
            
            # Fallback: try to get individual robot position by name
            elif hasattr(self.swarm_manager, 'get_robot_position'):
                # This would require knowing robot names, which we can get from config
                for robot_id in list(self.robot_positions.keys()):
                    try:
                        position = self.swarm_manager.get_robot_position(robot_id)
                        if position is not None:
                            position_data = {
                                'position': list(position) + [0.0],  # Add z=0 if 2D
                                'orientation': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
                                'battery_level': 100.0,
                                'status': 'active'
                            }
                            self._update_robot_position(robot_id, position_data)
                    except Exception as e:
                        logger.debug(f"Could not get position for robot {robot_id}: {e}")
            
        except Exception as e:
            logger.error(f"Error updating from swarm manager: {e}")
    
    def _update_robot_position(self, robot_id: str, position_data: Dict[str, Any]):
        """
        Update individual robot position and status.
        
        Args:
            robot_id: Unique robot identifier
            position_data: Dictionary containing position and status information
        """
        current_time = time.time()
        
        # Extract position information
        position = position_data.get('position', [0, 0, 0])
        orientation = position_data.get('orientation', {'roll': 0, 'pitch': 0, 'yaw': 0})
        battery_level = position_data.get('battery_level', 100.0)
        status_str = position_data.get('status', 'active')
        
        # Convert position to floats
        if isinstance(position, (list, tuple)) and len(position) >= 2:
            x, y = float(position[0]), float(position[1])
            z = float(position[2]) if len(position) > 2 else 0.0
        else:
            logger.warning(f"Invalid position data for robot {robot_id}: {position}")
            return
        
        # Convert status string to enum
        try:
            status = RobotStatus(status_str.lower())
        except ValueError:
            status = RobotStatus.ACTIVE if status_str.lower() == 'active' else RobotStatus.INACTIVE
        
        # Check if this is a significant position update
        if robot_id in self.robot_positions:
            old_pos = self.robot_positions[robot_id]
            distance_moved = np.sqrt((x - old_pos.x)**2 + (y - old_pos.y)**2 + (z - old_pos.z)**2)
            
            # Only update if robot moved significantly or status changed
            if distance_moved < self.position_update_threshold and status == old_pos.status:
                return
        
        # Create or update robot position
        self.robot_positions[robot_id] = RobotPosition(
            x=x,
            y=y,
            z=z,
            orientation=orientation,
            status=status,
            battery_level=battery_level
        )
        
        # Update tracking data
        self.robot_status_cache[robot_id] = status
        self.last_update_time[robot_id] = current_time
        
        logger.debug(f"Updated robot {robot_id}: pos=({x:.2f}, {y:.2f}, {z:.2f}), status={status.value}")
    
    def _update_robot_status(self, current_time: float):
        """
        Update robot status based on last activity time.
        Mark robots as inactive if they haven't been updated recently.
        """
        for robot_id in list(self.robot_positions.keys()):
            last_update = self.last_update_time.get(robot_id, 0)
            time_since_update = current_time - last_update
            
            # Mark robot as inactive if no recent updates
            if time_since_update > self.status_timeout:
                if self.robot_positions[robot_id].status == RobotStatus.ACTIVE:
                    self.robot_positions[robot_id].status = RobotStatus.INACTIVE
                    self.robot_status_cache[robot_id] = RobotStatus.INACTIVE
                    logger.info(f"Robot {robot_id} marked as inactive (no updates for {time_since_update:.1f}s)")
    
    def add_robot(self, robot_id: str, initial_position: List[float], initial_orientation: Dict[str, float] = None):
        """
        Add a new robot to tracking.
        
        Args:
            robot_id: Unique robot identifier
            initial_position: [x, y, z] initial position
            initial_orientation: {'roll', 'pitch', 'yaw'} initial orientation
        """
        if initial_orientation is None:
            initial_orientation = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        
        self.robot_positions[robot_id] = RobotPosition(
            x=float(initial_position[0]),
            y=float(initial_position[1]),
            z=float(initial_position[2]) if len(initial_position) > 2 else 0.0,
            orientation=initial_orientation,
            status=RobotStatus.INACTIVE,
            battery_level=100.0
        )
        
        self.robot_status_cache[robot_id] = RobotStatus.INACTIVE
        self.last_update_time[robot_id] = 0.0
        
        logger.info(f"Added robot {robot_id} to tracking")
    
    def remove_robot(self, robot_id: str):
        """
        Remove a robot from tracking.
        
        Args:
            robot_id: Robot identifier to remove
        """
        if robot_id in self.robot_positions:
            del self.robot_positions[robot_id]
            del self.robot_status_cache[robot_id]
            del self.last_update_time[robot_id]
            logger.info(f"Removed robot {robot_id} from tracking")
    
    def get_robot_status_summary(self) -> Dict[str, Any]:
        """
        Get summary of robot status.
        
        Returns:
            Dictionary with robot status counts and details
        """
        status_counts = {
            'active': 0,
            'inactive': 0,
            'error': 0,
            'total': len(self.robot_positions)
        }
        
        robot_details = {}
        
        for robot_id, robot_pos in self.robot_positions.items():
            status_counts[robot_pos.status.value] += 1
            
            robot_details[robot_id] = {
                'status': robot_pos.status.value,
                'position': [robot_pos.x, robot_pos.y, robot_pos.z],
                'battery_level': robot_pos.battery_level,
                'last_update': self.last_update_time.get(robot_id, 0)
            }
        
        return {
            'status_counts': status_counts,
            'robot_details': robot_details,
            'timestamp': time.time()
        }
    
    def set_robot_status(self, robot_id: str, status: RobotStatus):
        """
        Manually set robot status.
        
        Args:
            robot_id: Robot identifier
            status: New robot status
        """
        if robot_id in self.robot_positions:
            self.robot_positions[robot_id].status = status
            self.robot_status_cache[robot_id] = status
            self.last_update_time[robot_id] = time.time()
            logger.info(f"Set robot {robot_id} status to {status.value}")
    
    def get_active_robots(self) -> List[str]:
        """Get list of currently active robot IDs."""
        return [
            robot_id for robot_id, robot_pos in self.robot_positions.items()
            if robot_pos.status == RobotStatus.ACTIVE
        ]
    
    def get_robot_trajectories(self, time_window: float = 60.0) -> Dict[str, List[Dict[str, Any]]]:
        """
        Get robot trajectory data for the specified time window.
        This would typically integrate with the data collector's history.
        
        Args:
            time_window: Time window in seconds for trajectory data
            
        Returns:
            Dictionary mapping robot IDs to trajectory points
        """
        # This is a placeholder - actual implementation would use
        # the data collector's history storage
        trajectories = {}
        
        for robot_id in self.robot_positions.keys():
            # Generate sample trajectory for demonstration
            current_pos = self.robot_positions[robot_id]
            trajectories[robot_id] = [
                {
                    'timestamp': time.time(),
                    'x': current_pos.x,
                    'y': current_pos.y,
                    'z': current_pos.z
                }
            ]
        
        return trajectories