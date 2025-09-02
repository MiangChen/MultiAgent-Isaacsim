"""
Chart data generation system for the WebManager.
Handles trajectory and performance trend chart data generation for ECharts rendering.
"""

import time
import logging
from typing import Dict, List, Any, Optional, Tuple
from collections import deque
import numpy as np

from .models import ChartData, ChartSeries, RobotPositions, PerformanceMetrics

logger = logging.getLogger(__name__)


class ChartDataGenerator:
    """
    Chart data generation system that creates ECharts-compatible data structures
    from historical robot and performance data.
    """
    
    # Predefined colors for multi-robot trajectory visualization
    ROBOT_COLORS = [
        '#FF6B6B',  # Red
        '#4ECDC4',  # Teal
        '#45B7D1',  # Blue
        '#96CEB4',  # Green
        '#FFEAA7',  # Yellow
        '#DDA0DD',  # Plum
        '#98D8C8',  # Mint
        '#F7DC6F',  # Light Yellow
        '#BB8FCE',  # Light Purple
        '#85C1E9',  # Light Blue
    ]
    
    def __init__(self, max_trajectory_points: int = 500, max_performance_points: int = 200, 
                 trajectory_window_seconds: float = 300.0):
        """
        Initialize the chart data generator.
        
        Args:
            max_trajectory_points: Maximum number of trajectory points per robot
            max_performance_points: Maximum number of performance data points
            trajectory_window_seconds: Rolling window size for trajectory display in seconds
        """
        self.max_trajectory_points = max_trajectory_points
        self.max_performance_points = max_performance_points
        self.trajectory_window_seconds = trajectory_window_seconds
        
        # Trajectory state tracking for real-time updates
        self.robot_trajectory_cache = {}
        self.last_trajectory_update = {}
        
        logger.info(f"ChartDataGenerator initialized with {max_trajectory_points} trajectory points, "
                   f"{max_performance_points} performance points, and {trajectory_window_seconds}s trajectory window")
    
    def generate_trajectory_chart_data(self, robot_data_history: List[Dict[str, Any]], 
                                     enable_rolling_window: bool = True) -> ChartData:
        """
        Create robot trajectory data extraction from position history with rolling window support.
        Format trajectory data for ECharts line chart rendering.
        Add multi-robot trajectory support with different colors and real-time path updates.
        
        Args:
            robot_data_history: List of historical robot position data entries
            enable_rolling_window: Whether to apply rolling window filtering
            
        Returns:
            ChartData: Formatted trajectory chart data for ECharts
        """
        try:
            # Extract trajectory data for each robot with rolling window
            robot_trajectories = self._extract_robot_trajectories_with_window(
                robot_data_history, enable_rolling_window
            )
            
            # Update trajectory cache for real-time updates
            self._update_trajectory_cache(robot_trajectories)
            
            # Generate chart series for each robot
            series_list = []
            color_index = 0
            current_time = time.time()
            
            for robot_id, trajectory in robot_trajectories.items():
                if not trajectory['x'] or not trajectory['y']:
                    continue
                
                # Create coordinate pairs for ECharts line chart
                trajectory_data = []
                timestamps = trajectory['timestamps']
                
                for i in range(len(trajectory['x'])):
                    trajectory_data.append([trajectory['x'][i], trajectory['y'][i]])
                
                # Apply intelligent point reduction for performance
                trajectory_data = self._optimize_trajectory_points(
                    trajectory_data, timestamps, robot_id
                )
                
                # Create chart series for this robot with enhanced styling
                robot_series = ChartSeries(
                    name=f"Robot {robot_id}",
                    data=trajectory_data,
                    type="line",
                    color=self.ROBOT_COLORS[color_index % len(self.ROBOT_COLORS)]
                )
                
                # Add trajectory metadata for enhanced visualization
                robot_series.metadata = {
                    'robot_id': robot_id,
                    'point_count': len(trajectory_data),
                    'time_span': timestamps[-1] - timestamps[0] if timestamps else 0,
                    'last_update': current_time,
                    'path_length': self._calculate_path_length(trajectory_data)
                }
                
                series_list.append(robot_series)
                color_index += 1
            
            # Create chart data structure with enhanced metadata
            chart_data = ChartData(
                chart_type="line",
                series=series_list,
                title="Robot Trajectories (Real-time)",
                x_axis_label="X Position (m)",
                y_axis_label="Y Position (m)"
            )
            
            # Add trajectory-specific metadata
            chart_data.metadata = {
                'rolling_window_enabled': enable_rolling_window,
                'window_seconds': self.trajectory_window_seconds,
                'total_robots': len(series_list),
                'update_timestamp': current_time,
                'trajectory_cache_size': len(self.robot_trajectory_cache)
            }
            
            logger.debug(f"Generated trajectory chart data for {len(series_list)} robots with rolling window")
            return chart_data
            
        except Exception as e:
            logger.error(f"Error generating trajectory chart data: {e}")
            return self._create_empty_trajectory_chart()
    
    def generate_performance_trend_chart_data(self, performance_data_history: List[Dict[str, Any]]) -> ChartData:
        """
        Extract performance metrics from history for trend analysis.
        Create time-series data formatting for ECharts.
        Implement multiple metric series (FPS, CPU, memory) in single chart.
        
        Args:
            performance_data_history: List of historical performance data entries
            
        Returns:
            ChartData: Formatted performance trend chart data for ECharts
        """
        try:
            # Extract performance trends
            performance_trends = self._extract_performance_trends(performance_data_history)
            
            if not performance_trends['timestamps']:
                return self._create_empty_performance_chart()
            
            # Limit data points to prevent performance issues
            timestamps = performance_trends['timestamps']
            if len(timestamps) > self.max_performance_points:
                # Keep evenly spaced points
                step = len(timestamps) // self.max_performance_points
                timestamps = timestamps[::step]
                for metric in ['fps', 'cpu_usage', 'memory_usage', 'ros_message_rate']:
                    performance_trends[metric] = performance_trends[metric][::step]
            
            # Convert timestamps to relative time in seconds for better readability
            if timestamps:
                start_time = timestamps[0]
                relative_timestamps = [(t - start_time) for t in timestamps]
            else:
                relative_timestamps = []
            
            # Create chart series for each performance metric
            series_list = [
                ChartSeries(
                    name="FPS",
                    data=performance_trends['fps'],
                    type="line",
                    color="#FF6B6B"  # Red
                ),
                ChartSeries(
                    name="CPU Usage (%)",
                    data=performance_trends['cpu_usage'],
                    type="line",
                    color="#4ECDC4"  # Teal
                ),
                ChartSeries(
                    name="Memory Usage (%)",
                    data=performance_trends['memory_usage'],
                    type="line",
                    color="#45B7D1"  # Blue
                ),
                ChartSeries(
                    name="ROS Message Rate (Hz)",
                    data=performance_trends['ros_message_rate'],
                    type="line",
                    color="#96CEB4"  # Green
                )
            ]
            
            # Create chart data structure
            chart_data = ChartData(
                chart_type="line",
                series=series_list,
                x_axis=relative_timestamps,
                title="Performance Trends",
                x_axis_label="Time (seconds)",
                y_axis_label="Value"
            )
            
            logger.debug(f"Generated performance trend chart data with {len(relative_timestamps)} data points")
            return chart_data
            
        except Exception as e:
            logger.error(f"Error generating performance trend chart data: {e}")
            return self._create_empty_performance_chart()
    
    def generate_robot_status_chart_data(self, robot_data_history: List[Dict[str, Any]]) -> ChartData:
        """
        Generate chart data for robot status over time (active/inactive/error counts).
        
        Args:
            robot_data_history: List of historical robot position data entries
            
        Returns:
            ChartData: Formatted robot status chart data for ECharts
        """
        try:
            status_trends = self._extract_robot_status_trends(robot_data_history)
            
            if not status_trends['timestamps']:
                return self._create_empty_status_chart()
            
            # Convert timestamps to relative time
            timestamps = status_trends['timestamps']
            if timestamps:
                start_time = timestamps[0]
                relative_timestamps = [(t - start_time) for t in timestamps]
            else:
                relative_timestamps = []
            
            # Create chart series for each status type
            series_list = [
                ChartSeries(
                    name="Active Robots",
                    data=status_trends['active_count'],
                    type="line",
                    color="#96CEB4"  # Green
                ),
                ChartSeries(
                    name="Inactive Robots",
                    data=status_trends['inactive_count'],
                    type="line",
                    color="#FFEAA7"  # Yellow
                ),
                ChartSeries(
                    name="Error Robots",
                    data=status_trends['error_count'],
                    type="line",
                    color="#FF6B6B"  # Red
                )
            ]
            
            chart_data = ChartData(
                chart_type="line",
                series=series_list,
                x_axis=relative_timestamps,
                title="Robot Status Over Time",
                x_axis_label="Time (seconds)",
                y_axis_label="Number of Robots"
            )
            
            logger.debug(f"Generated robot status chart data with {len(relative_timestamps)} data points")
            return chart_data
            
        except Exception as e:
            logger.error(f"Error generating robot status chart data: {e}")
            return self._create_empty_status_chart()
    
    def _extract_robot_trajectories(self, robot_data_history: List[Dict[str, Any]]) -> Dict[str, Dict[str, List[float]]]:
        """
        Extract robot trajectory data from position history.
        
        Args:
            robot_data_history: List of historical robot position data entries
            
        Returns:
            Dict mapping robot IDs to trajectory data (x, y coordinates and timestamps)
        """
        trajectories = {}
        
        for entry in robot_data_history:
            timestamp = entry.get('timestamp', time.time())
            positions = entry.get('positions', {})
            
            for robot_id, position_data in positions.items():
                if robot_id not in trajectories:
                    trajectories[robot_id] = {
                        'x': [],
                        'y': [],
                        'z': [],
                        'timestamps': []
                    }
                
                # Extract position coordinates
                x = position_data.get('x', 0.0)
                y = position_data.get('y', 0.0)
                z = position_data.get('z', 0.0)
                
                trajectories[robot_id]['x'].append(x)
                trajectories[robot_id]['y'].append(y)
                trajectories[robot_id]['z'].append(z)
                trajectories[robot_id]['timestamps'].append(timestamp)
        
        return trajectories
    
    def _extract_robot_trajectories_with_window(self, robot_data_history: List[Dict[str, Any]], 
                                              enable_rolling_window: bool = True) -> Dict[str, Dict[str, List[float]]]:
        """
        Extract robot trajectory data with rolling window filtering.
        Implements rolling window trajectory display for real-time updates.
        
        Args:
            robot_data_history: List of historical robot position data entries
            enable_rolling_window: Whether to apply rolling window filtering
            
        Returns:
            Dict mapping robot IDs to filtered trajectory data
        """
        # First extract all trajectories
        trajectories = self._extract_robot_trajectories(robot_data_history)
        
        if not enable_rolling_window:
            return trajectories
        
        # Apply rolling window filtering
        current_time = time.time()
        window_start_time = current_time - self.trajectory_window_seconds
        
        filtered_trajectories = {}
        
        for robot_id, trajectory in trajectories.items():
            timestamps = trajectory['timestamps']
            
            if not timestamps:
                filtered_trajectories[robot_id] = trajectory
                continue
            
            # Find indices within the rolling window
            valid_indices = [
                i for i, ts in enumerate(timestamps) 
                if ts >= window_start_time
            ]
            
            if not valid_indices:
                # If no points in window, keep the last point
                if timestamps:
                    valid_indices = [len(timestamps) - 1]
            
            # Filter trajectory data to rolling window
            filtered_trajectories[robot_id] = {
                'x': [trajectory['x'][i] for i in valid_indices],
                'y': [trajectory['y'][i] for i in valid_indices],
                'z': [trajectory['z'][i] for i in valid_indices],
                'timestamps': [trajectory['timestamps'][i] for i in valid_indices]
            }
        
        return filtered_trajectories
    
    def _update_trajectory_cache(self, robot_trajectories: Dict[str, Dict[str, List[float]]]):
        """
        Update trajectory cache for real-time path updates.
        Tracks robot movement data for efficient incremental updates.
        
        Args:
            robot_trajectories: Current robot trajectory data
        """
        current_time = time.time()
        
        for robot_id, trajectory in robot_trajectories.items():
            if not trajectory['timestamps']:
                continue
            
            # Update cache with latest trajectory data
            self.robot_trajectory_cache[robot_id] = {
                'last_position': {
                    'x': trajectory['x'][-1] if trajectory['x'] else 0.0,
                    'y': trajectory['y'][-1] if trajectory['y'] else 0.0,
                    'z': trajectory['z'][-1] if trajectory['z'] else 0.0
                },
                'last_timestamp': trajectory['timestamps'][-1] if trajectory['timestamps'] else current_time,
                'point_count': len(trajectory['x']),
                'path_length': self._calculate_path_length(
                    [[trajectory['x'][i], trajectory['y'][i]] for i in range(len(trajectory['x']))]
                )
            }
            
            self.last_trajectory_update[robot_id] = current_time
    
    def _optimize_trajectory_points(self, trajectory_data: List[List[float]], 
                                  timestamps: List[float], robot_id: str) -> List[List[float]]:
        """
        Apply intelligent point reduction for trajectory performance optimization.
        Implements adaptive sampling based on movement patterns.
        
        Args:
            trajectory_data: List of [x, y] coordinate pairs
            timestamps: Corresponding timestamps
            robot_id: Robot identifier for caching
            
        Returns:
            Optimized trajectory data with reduced points
        """
        if len(trajectory_data) <= self.max_trajectory_points:
            return trajectory_data
        
        # Use adaptive sampling that preserves important trajectory features
        optimized_points = []
        
        if len(trajectory_data) < 3:
            return trajectory_data
        
        # Always include first point
        optimized_points.append(trajectory_data[0])
        
        # Calculate movement significance for each point
        movement_significance = []
        for i in range(1, len(trajectory_data) - 1):
            # Calculate angle change and distance
            prev_point = trajectory_data[i - 1]
            curr_point = trajectory_data[i]
            next_point = trajectory_data[i + 1]
            
            # Distance from previous point
            distance = np.sqrt(
                (curr_point[0] - prev_point[0]) ** 2 + 
                (curr_point[1] - prev_point[1]) ** 2
            )
            
            # Angle change (direction change significance)
            vec1 = [curr_point[0] - prev_point[0], curr_point[1] - prev_point[1]]
            vec2 = [next_point[0] - curr_point[0], next_point[1] - curr_point[1]]
            
            # Calculate angle between vectors
            angle_change = 0.0
            if np.linalg.norm(vec1) > 0 and np.linalg.norm(vec2) > 0:
                cos_angle = np.dot(vec1, vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2))
                cos_angle = np.clip(cos_angle, -1.0, 1.0)
                angle_change = np.arccos(cos_angle)
            
            # Combine distance and angle change for significance score
            significance = distance + angle_change * 0.5
            movement_significance.append((i, significance))
        
        # Sort by significance and select top points
        movement_significance.sort(key=lambda x: x[1], reverse=True)
        
        # Select most significant points up to the limit
        selected_indices = [0]  # Always include first point
        selected_indices.extend([
            idx for idx, _ in movement_significance[:self.max_trajectory_points - 2]
        ])
        selected_indices.append(len(trajectory_data) - 1)  # Always include last point
        
        # Sort indices to maintain chronological order
        selected_indices.sort()
        
        # Build optimized trajectory
        optimized_points = [trajectory_data[i] for i in selected_indices]
        
        return optimized_points
    
    def _calculate_path_length(self, trajectory_data: List[List[float]]) -> float:
        """
        Calculate the total path length of a trajectory.
        
        Args:
            trajectory_data: List of [x, y] coordinate pairs
            
        Returns:
            Total path length in meters
        """
        if len(trajectory_data) < 2:
            return 0.0
        
        total_length = 0.0
        for i in range(1, len(trajectory_data)):
            prev_point = trajectory_data[i - 1]
            curr_point = trajectory_data[i]
            
            distance = np.sqrt(
                (curr_point[0] - prev_point[0]) ** 2 + 
                (curr_point[1] - prev_point[1]) ** 2
            )
            total_length += distance
        
        return total_length
    
    def _extract_performance_trends(self, performance_data_history: List[Dict[str, Any]]) -> Dict[str, List[float]]:
        """
        Extract performance trend data from history.
        
        Args:
            performance_data_history: List of historical performance data entries
            
        Returns:
            Dict containing lists of performance metrics over time
        """
        trends = {
            'timestamps': [],
            'fps': [],
            'cpu_usage': [],
            'memory_usage': [],
            'ros_message_rate': []
        }
        
        for entry in performance_data_history:
            timestamp = entry.get('timestamp', time.time())
            metrics = entry.get('metrics', {})
            
            trends['timestamps'].append(timestamp)
            trends['fps'].append(metrics.get('fps', 0.0))
            trends['cpu_usage'].append(metrics.get('cpu_usage', 0.0))
            trends['memory_usage'].append(metrics.get('memory_usage', 0.0))
            trends['ros_message_rate'].append(metrics.get('ros_message_rate', 0.0))
        
        return trends
    
    def _extract_robot_status_trends(self, robot_data_history: List[Dict[str, Any]]) -> Dict[str, List[float]]:
        """
        Extract robot status trends from history.
        
        Args:
            robot_data_history: List of historical robot position data entries
            
        Returns:
            Dict containing counts of robots by status over time
        """
        trends = {
            'timestamps': [],
            'active_count': [],
            'inactive_count': [],
            'error_count': []
        }
        
        for entry in robot_data_history:
            timestamp = entry.get('timestamp', time.time())
            positions = entry.get('positions', {})
            
            # Count robots by status
            active_count = 0
            inactive_count = 0
            error_count = 0
            
            for robot_id, position_data in positions.items():
                status = position_data.get('status', 'active')
                if status == 'active':
                    active_count += 1
                elif status == 'inactive':
                    inactive_count += 1
                elif status == 'error':
                    error_count += 1
            
            trends['timestamps'].append(timestamp)
            trends['active_count'].append(active_count)
            trends['inactive_count'].append(inactive_count)
            trends['error_count'].append(error_count)
        
        return trends
    
    def _create_empty_trajectory_chart(self) -> ChartData:
        """Create empty trajectory chart data."""
        return ChartData(
            chart_type="line",
            series=[],
            title="Robot Trajectories",
            x_axis_label="X Position (m)",
            y_axis_label="Y Position (m)"
        )
    
    def _create_empty_performance_chart(self) -> ChartData:
        """Create empty performance chart data."""
        return ChartData(
            chart_type="line",
            series=[],
            title="Performance Trends",
            x_axis_label="Time (seconds)",
            y_axis_label="Value"
        )
    
    def _create_empty_status_chart(self) -> ChartData:
        """Create empty robot status chart data."""
        return ChartData(
            chart_type="line",
            series=[],
            title="Robot Status Over Time",
            x_axis_label="Time (seconds)",
            y_axis_label="Number of Robots"
        )
    
    def get_available_chart_types(self) -> List[str]:
        """Get list of available chart types."""
        return [
            "robot_trajectories",
            "performance_trends", 
            "robot_status"
        ]
    
    def get_trajectory_cache_info(self) -> Dict[str, Any]:
        """
        Get information about the trajectory cache for monitoring.
        
        Returns:
            Dict containing trajectory cache statistics
        """
        return {
            'cached_robots': list(self.robot_trajectory_cache.keys()),
            'cache_size': len(self.robot_trajectory_cache),
            'last_updates': self.last_trajectory_update.copy(),
            'window_seconds': self.trajectory_window_seconds,
            'max_points': self.max_trajectory_points
        }
    
    def set_trajectory_window(self, window_seconds: float):
        """
        Set the rolling window size for trajectory display.
        
        Args:
            window_seconds: Rolling window size in seconds
        """
        if window_seconds <= 0:
            raise ValueError("Trajectory window must be positive")
        
        self.trajectory_window_seconds = window_seconds
        logger.info(f"Trajectory window set to {window_seconds} seconds")
    
    def clear_trajectory_cache(self):
        """Clear the trajectory cache for all robots."""
        self.robot_trajectory_cache.clear()
        self.last_trajectory_update.clear()
        logger.info("Trajectory cache cleared")
    
    def get_robot_trajectory_info(self, robot_id: str) -> Optional[Dict[str, Any]]:
        """
        Get trajectory information for a specific robot.
        
        Args:
            robot_id: Robot identifier
            
        Returns:
            Dict containing robot trajectory info or None if not found
        """
        return self.robot_trajectory_cache.get(robot_id)
    
    def has_trajectory_updates(self, since_timestamp: float) -> bool:
        """
        Check if there have been trajectory updates since a given timestamp.
        
        Args:
            since_timestamp: Timestamp to check against
            
        Returns:
            True if there have been updates since the timestamp
        """
        return any(
            update_time > since_timestamp 
            for update_time in self.last_trajectory_update.values()
        )
    
    def generate_chart_data(self, chart_type: str, data_history: Dict[str, List[Dict[str, Any]]]) -> ChartData:
        """
        Generate chart data based on chart type and available data history.
        
        Args:
            chart_type: Type of chart to generate
            data_history: Dictionary containing different types of historical data
            
        Returns:
            ChartData: Generated chart data
        """
        try:
            if chart_type == "robot_trajectories":
                robot_history = data_history.get('robot_data', [])
                return self.generate_trajectory_chart_data(robot_history)
            
            elif chart_type == "performance_trends":
                performance_history = data_history.get('performance_data', [])
                return self.generate_performance_trend_chart_data(performance_history)
            
            elif chart_type == "robot_status":
                robot_history = data_history.get('robot_data', [])
                return self.generate_robot_status_chart_data(robot_history)
            
            else:
                logger.warning(f"Unknown chart type: {chart_type}")
                return ChartData(chart_type="line", series=[], title="Unknown Chart Type")
                
        except Exception as e:
            logger.error(f"Error generating chart data for type {chart_type}: {e}")
            return ChartData(chart_type="line", series=[], title="Error Loading Chart")