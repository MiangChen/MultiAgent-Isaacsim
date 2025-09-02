"""
Data collection system for the WebManager.
Handles real-time data collection from various sources and manages data history.
"""

import asyncio
import threading
import time
import logging
from collections import deque
from typing import Dict, Any, List, Optional, Callable, Protocol
from abc import ABC, abstractmethod
import psutil
import numpy as np

from .models import (
    RobotPosition, RobotPositions, PerformanceMetrics, 
    RosGraphData, RosNode, RosTopic, RosConnection, RobotStatus,
    ChartData
)
from .chart_generator import ChartDataGenerator

logger = logging.getLogger(__name__)


class DataSource(Protocol):
    """Protocol for data source interfaces."""
    
    def get_data(self) -> Any:
        """Get current data from the source."""
        ...
    
    def is_available(self) -> bool:
        """Check if the data source is available."""
        ...


class RobotDataSource(DataSource):
    """Interface for robot position data sources."""
    
    @abstractmethod
    def get_robot_positions(self) -> RobotPositions:
        """Get current robot positions and status."""
        pass
    
    @abstractmethod
    def get_robot_count(self) -> int:
        """Get the number of active robots."""
        pass


class PerformanceDataSource(DataSource):
    """Interface for performance metrics data sources."""
    
    @abstractmethod
    def get_fps(self) -> float:
        """Get current simulation FPS."""
        pass
    
    @abstractmethod
    def get_cpu_usage(self) -> float:
        """Get current CPU usage percentage."""
        pass
    
    @abstractmethod
    def get_memory_usage(self) -> float:
        """Get current memory usage percentage."""
        pass
    
    @abstractmethod
    def get_ros_message_rate(self) -> float:
        """Get current ROS message publishing rate."""
        pass


class RosDataSource(DataSource):
    """Interface for ROS graph data sources."""
    
    @abstractmethod
    def get_ros_nodes(self) -> List[RosNode]:
        """Get list of active ROS nodes."""
        pass
    
    @abstractmethod
    def get_ros_topics(self) -> List[RosTopic]:
        """Get list of active ROS topics."""
        pass
    
    @abstractmethod
    def get_ros_connections(self) -> List[RosConnection]:
        """Get ROS node connections."""
        pass


class CameraDataSource(DataSource):
    """Interface for camera frame data sources."""
    
    @abstractmethod
    def get_camera_frame(self, camera_id: str = "default") -> Optional[np.ndarray]:
        """Get current camera frame."""
        pass
    
    @abstractmethod
    def get_available_cameras(self) -> List[str]:
        """Get list of available camera IDs."""
        pass


class DataCollector:
    """
    Core data collector class that manages async data collection from various sources.
    Implements data history management with deque storage and provides interfaces
    for different data source types.
    """
    
    def __init__(self, web_server=None, max_history: int = 1000, collection_rate: float = 10.0, 
                 chart_update_rate: float = 2.0):
        """
        Initialize the data collector.
        
        Args:
            web_server: WebUIServer instance for broadcasting updates
            max_history: Maximum number of historical data points to store
            collection_rate: Data collection frequency in Hz
            chart_update_rate: Chart data generation frequency in Hz
        """
        self.web_server = web_server
        self.max_history = max_history
        self.collection_rate = collection_rate
        self.collection_interval = 1.0 / collection_rate
        self.chart_update_rate = chart_update_rate
        self.chart_update_interval = 1.0 / chart_update_rate
        
        # Data history storage using deque for efficient operations
        self.robot_data_history = deque(maxlen=max_history)
        self.performance_history = deque(maxlen=max_history)
        self.ros_graph_history = deque(maxlen=max_history)
        
        # Data sources
        self.robot_data_source: Optional[RobotDataSource] = None
        self.performance_data_source: Optional[PerformanceDataSource] = None
        self.ros_data_source: Optional[RosDataSource] = None
        self.camera_data_source: Optional[CameraDataSource] = None
        
        # Collection control
        self.collecting = False
        self.collection_task: Optional[asyncio.Task] = None
        self.chart_generation_task: Optional[asyncio.Task] = None
        self.collection_thread: Optional[threading.Thread] = None
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        
        # Current data cache
        self.current_robot_data: RobotPositions = {}
        self.current_performance_data: Optional[PerformanceMetrics] = None
        self.current_ros_data: Optional[RosGraphData] = None
        
        # Collection statistics
        self.collection_stats = {
            'total_collections': 0,
            'failed_collections': 0,
            'last_collection_time': 0,
            'average_collection_time': 0
        }
        
        # Chart generation control and statistics
        self.chart_generation_enabled = True
        self.chart_generation_stats = {
            'total_generations': 0,
            'failed_generations': 0,
            'last_generation_time': 0,
            'average_generation_time': 0,
            'last_chart_update': 0
        }
        
        # Chart data generator
        self.chart_generator = ChartDataGenerator(
            max_trajectory_points=500,
            max_performance_points=200
        )
        
        # Warning throttling to reduce log spam
        self.last_collection_warning = 0
        self.collection_warning_cooldown = 10.0  # 10 seconds between collection warnings
        
        # Adaptive collection rate to reduce Isaac Sim impact
        self.base_collection_rate = collection_rate
        self.adaptive_collection_enabled = True
        self.performance_history_for_adaptation = deque(maxlen=10)
        self.last_rate_adjustment = 0
        
        logger.info(f"DataCollector initialized with {max_history} max history, {collection_rate}Hz collection rate, and {chart_update_rate}Hz chart update rate")
    
    def register_robot_data_source(self, source: RobotDataSource):
        """Register a robot data source."""
        self.robot_data_source = source
        logger.info("Robot data source registered")
    
    def register_performance_data_source(self, source: PerformanceDataSource):
        """Register a performance metrics data source."""
        self.performance_data_source = source
        logger.info("Performance data source registered")
    
    def register_ros_data_source(self, source: RosDataSource):
        """Register a ROS graph data source."""
        self.ros_data_source = source
        logger.info("ROS data source registered")
    
    def register_camera_data_source(self, source: CameraDataSource):
        """Register a camera data source."""
        self.camera_data_source = source
        logger.info("Camera data source registered")
    
    def start_collection(self):
        """Start the data collection process in a separate thread."""
        if self.collecting:
            logger.warning("Data collection is already running")
            return
        
        self.collecting = True
        self.collection_thread = threading.Thread(target=self._run_collection_loop, daemon=True)
        self.collection_thread.start()
        logger.info("Data collection started")
    
    def stop_collection(self):
        """Stop the data collection process."""
        if not self.collecting:
            logger.warning("Data collection is not running")
            return
        
        self.collecting = False
        
        # Cancel the collection task if it exists
        if self.collection_task and not self.collection_task.done():
            self.collection_task.cancel()
        
        # Cancel the chart generation task if it exists
        if self.chart_generation_task and not self.chart_generation_task.done():
            self.chart_generation_task.cancel()
        
        # Wait for the thread to finish
        if self.collection_thread and self.collection_thread.is_alive():
            self.collection_thread.join(timeout=5)
        
        logger.info("Data collection stopped")
    
    def _run_collection_loop(self):
        """Run the async collection loop in a separate thread with lower priority."""
        try:
            # Set lower thread priority to avoid interfering with Isaac Sim
            import os
            if hasattr(os, 'nice'):
                try:
                    os.nice(5)  # Lower priority (higher nice value)
                except PermissionError:
                    pass  # Ignore if we can't change priority
            
            # Create a new event loop for this thread
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
            
            # Start both collection and chart generation tasks
            self.collection_task = self.loop.create_task(self._collection_loop())
            if self.chart_generation_enabled:
                self.chart_generation_task = self.loop.create_task(self._chart_generation_loop())
            
            # Run both tasks concurrently
            tasks = [self.collection_task]
            if self.chart_generation_task:
                tasks.append(self.chart_generation_task)
            
            self.loop.run_until_complete(asyncio.gather(*tasks, return_exceptions=True))
            
        except asyncio.CancelledError:
            logger.info("Collection loop cancelled")
        except Exception as e:
            logger.error(f"Error in collection loop: {e}")
        finally:
            if self.loop:
                self.loop.close()
    
    async def _collection_loop(self):
        """Main async collection loop."""
        logger.info(f"Starting collection loop at {self.collection_rate}Hz")
        
        while self.collecting:
            collection_start = time.time()
            
            try:
                # Collect data from all sources
                await self._collect_all_data()
                
                # Update collection statistics
                collection_time = time.time() - collection_start
                self._update_collection_stats(collection_time)
                
                # Adaptive rate adjustment to reduce Isaac Sim impact
                if self.adaptive_collection_enabled:
                    await self._adjust_collection_rate_if_needed(collection_time)
                
                # Wait for next collection cycle
                sleep_time = max(0, self.collection_interval - collection_time)
                if sleep_time > 0:
                    await asyncio.sleep(sleep_time)
                else:
                    # Throttle collection warnings to reduce log spam
                    current_time = time.time()
                    if current_time - self.last_collection_warning >= self.collection_warning_cooldown:
                        logger.warning(f"Collection took {collection_time:.3f}s, longer than interval {self.collection_interval:.3f}s")
                        self.last_collection_warning = current_time
                
            except Exception as e:
                logger.error(f"Error in collection cycle: {e}")
                self.collection_stats['failed_collections'] += 1
                await asyncio.sleep(self.collection_interval)
    
    async def _chart_generation_loop(self):
        """
        Timer-based chart data generation loop.
        Generates and broadcasts chart data at regular intervals.
        """
        logger.info(f"Starting chart generation loop at {self.chart_update_rate}Hz")
        
        while self.collecting and self.chart_generation_enabled:
            generation_start = time.time()
            
            try:
                # Generate chart data for all available chart types
                await self._generate_and_broadcast_charts()
                
                # Update chart generation statistics
                generation_time = time.time() - generation_start
                self._update_chart_generation_stats(generation_time)
                
                # Wait for next chart generation cycle
                sleep_time = max(0, self.chart_update_interval - generation_time)
                if sleep_time > 0:
                    await asyncio.sleep(sleep_time)
                else:
                    logger.warning(f"Chart generation took {generation_time:.3f}s, longer than interval {self.chart_update_interval:.3f}s")
                
            except Exception as e:
                logger.error(f"Error in chart generation cycle: {e}")
                self.chart_generation_stats['failed_generations'] += 1
                await asyncio.sleep(self.chart_update_interval)
    
    async def _collect_all_data(self):
        """Collect data from all registered sources with CPU-friendly scheduling."""
        # Collect robot data
        if self.robot_data_source and self.robot_data_source.is_available():
            await self._collect_robot_data()
            await asyncio.sleep(0)  # Yield control to other tasks
        
        # Collect performance data
        if self.performance_data_source and self.performance_data_source.is_available():
            await self._collect_performance_data()
            await asyncio.sleep(0)  # Yield control to other tasks
        
        # Collect ROS data
        if self.ros_data_source and self.ros_data_source.is_available():
            await self._collect_ros_data()
            await asyncio.sleep(0)  # Yield control to other tasks
        
        # Collect camera data
        if self.camera_data_source and self.camera_data_source.is_available():
            await self._collect_camera_data()
            await asyncio.sleep(0)  # Yield control to other tasks
    
    async def _collect_robot_data(self):
        """Collect robot position and status data."""
        try:
            robot_positions = self.robot_data_source.get_robot_positions()
            
            # Store in history
            history_entry = {
                'timestamp': time.time(),
                'positions': {robot_id: pos.to_dict() for robot_id, pos in robot_positions.items()}
            }
            self.robot_data_history.append(history_entry)
            
            # Update current data cache
            self.current_robot_data = robot_positions
            
            # Broadcast update if web server is available
            if self.web_server:
                positions_dict = {robot_id: pos.to_dict() for robot_id, pos in robot_positions.items()}
                await self.web_server.update_robot_positions(positions_dict)
            
        except Exception as e:
            logger.error(f"Error collecting robot data: {e}")
    
    async def _collect_performance_data(self):
        """Collect system performance metrics."""
        try:
            metrics = PerformanceMetrics(
                fps=self.performance_data_source.get_fps(),
                cpu_usage=self.performance_data_source.get_cpu_usage(),
                memory_usage=self.performance_data_source.get_memory_usage(),
                ros_message_rate=self.performance_data_source.get_ros_message_rate()
            )
            
            # Store in history
            self.performance_history.append({
                'timestamp': metrics.timestamp,
                'metrics': metrics.to_dict()
            })
            
            # Update current data cache
            self.current_performance_data = metrics
            
            # Broadcast update if web server is available
            if self.web_server:
                await self.web_server.update_performance_metrics(metrics.to_dict())
            
        except Exception as e:
            logger.error(f"Error collecting performance data: {e}")
    
    async def _collect_ros_data(self):
        """Collect ROS graph information."""
        try:
            nodes = self.ros_data_source.get_ros_nodes()
            topics = self.ros_data_source.get_ros_topics()
            connections = self.ros_data_source.get_ros_connections()
            
            ros_graph = RosGraphData(
                nodes=nodes,
                topics=topics,
                connections=connections
            )
            
            # Store in history
            self.ros_graph_history.append({
                'timestamp': time.time(),
                'graph': ros_graph.to_dict()
            })
            
            # Update current data cache
            self.current_ros_data = ros_graph
            
            # Broadcast update if web server is available
            if self.web_server:
                await self.web_server.update_ros_graph(ros_graph.to_dict())
            
        except Exception as e:
            logger.error(f"Error collecting ROS data: {e}")
    
    async def _collect_camera_data(self):
        """Collect and stream camera frames."""
        try:
            available_cameras = self.camera_data_source.get_available_cameras()
            
            for camera_id in available_cameras:
                # Only process active cameras
                camera_info = self.camera_data_source.get_camera_info(camera_id)
                if not camera_info or not camera_info.get('is_active', False):
                    continue
                
                # Get encoded frame
                encoded_frame = self.camera_data_source.get_encoded_frame(
                    camera_id=camera_id,
                    format='JPEG',
                    quality=80
                )
                
                if encoded_frame and self.web_server:
                    # Send frame via WebSocket with metadata
                    metadata = {
                        'camera_id': camera_id,
                        'format': 'JPEG',
                        'quality': 80,
                        'width': camera_info.get('resolution', [640, 480])[0],
                        'height': camera_info.get('resolution', [640, 480])[1]
                    }
                    
                    await self.web_server.send_compressed_camera_frame(
                        encoded_frame, camera_id, 'jpeg', 80,
                        metadata['width'], metadata['height']
                    )
            
        except Exception as e:
            logger.error(f"Error collecting camera data: {e}")
    
    def _update_collection_stats(self, collection_time: float):
        """Update collection statistics."""
        self.collection_stats['total_collections'] += 1
        self.collection_stats['last_collection_time'] = collection_time
        
        # Calculate running average
        total = self.collection_stats['total_collections']
        current_avg = self.collection_stats['average_collection_time']
        self.collection_stats['average_collection_time'] = (current_avg * (total - 1) + collection_time) / total
    
    def _update_chart_generation_stats(self, generation_time: float):
        """Update chart generation statistics."""
        self.chart_generation_stats['total_generations'] += 1
        self.chart_generation_stats['last_generation_time'] = generation_time
        self.chart_generation_stats['last_chart_update'] = time.time()
        
        # Calculate running average
        total = self.chart_generation_stats['total_generations']
        current_avg = self.chart_generation_stats['average_generation_time']
        self.chart_generation_stats['average_generation_time'] = (current_avg * (total - 1) + generation_time) / total
    
    async def _generate_and_broadcast_charts(self):
        """
        Generate chart data for all available chart types and broadcast to clients.
        Implements efficient data serialization for chart updates with trajectory optimization.
        """
        if not self.web_server or not self.web_server.websocket_manager.is_connected():
            return
        
        try:
            # Get available chart types
            chart_types = self.get_available_chart_types()
            current_time = time.time()
            
            # Generate chart data for each type with optimizations
            for chart_type in chart_types:
                try:
                    # Special handling for trajectory charts with rolling window
                    if chart_type == "robot_trajectories":
                        # Check if we have recent trajectory updates
                        last_update_check = current_time - 1.0  # Check last 1 second
                        if self.has_trajectory_updates(last_update_check):
                            chart_data = self.generate_chart_data(chart_type, enable_rolling_window=True)
                        else:
                            # Skip if no recent updates to reduce processing
                            continue
                    else:
                        chart_data = self.generate_chart_data(chart_type)
                    
                    # Serialize chart data efficiently
                    serialized_data = self._serialize_chart_data(chart_data)
                    
                    # Broadcast chart data to clients
                    await self.web_server.send_chart_data(chart_type, serialized_data)
                    
                    logger.debug(f"Generated and broadcast chart data for: {chart_type}")
                    
                except Exception as e:
                    logger.error(f"Error generating chart data for {chart_type}: {e}")
                    # Send empty chart on error
                    empty_chart = {
                        "chart_type": chart_type,
                        "series": [],
                        "title": f"Error Loading {chart_type.replace('_', ' ').title()}",
                        "metadata": {"error": True, "error_message": str(e)}
                    }
                    await self.web_server.send_chart_data(chart_type, empty_chart)
            
        except Exception as e:
            logger.error(f"Error in chart generation and broadcast: {e}")
    
    def _serialize_chart_data(self, chart_data: ChartData) -> Dict[str, Any]:
        """
        Efficiently serialize chart data for transmission.
        Optimizes data structure for WebSocket transmission.
        
        Args:
            chart_data: ChartData object to serialize
            
        Returns:
            Dict containing serialized chart data
        """
        try:
            # Convert ChartData to dictionary
            serialized = chart_data.to_dict()
            
            # Optimize series data for transmission
            if 'series' in serialized:
                for series in serialized['series']:
                    # Ensure data is in the most efficient format
                    if 'data' in series and isinstance(series['data'], list):
                        # Convert numpy arrays to lists if needed
                        if hasattr(series['data'], 'tolist'):
                            series['data'] = series['data'].tolist()
                        
                        # Round floating point numbers to reduce payload size
                        if series['data'] and isinstance(series['data'][0], (int, float)):
                            series['data'] = [round(x, 3) if isinstance(x, float) else x for x in series['data']]
                        elif series['data'] and isinstance(series['data'][0], (list, tuple)):
                            # Handle coordinate pairs
                            series['data'] = [
                                [round(coord, 3) if isinstance(coord, float) else coord for coord in point]
                                for point in series['data']
                            ]
            
            # Optimize x-axis data
            if 'x_axis' in serialized and serialized['x_axis']:
                if hasattr(serialized['x_axis'], 'tolist'):
                    serialized['x_axis'] = serialized['x_axis'].tolist()
                
                # Round timestamps to reduce precision
                if isinstance(serialized['x_axis'][0], float):
                    serialized['x_axis'] = [round(x, 2) for x in serialized['x_axis']]
            
            # Add metadata for client optimization
            serialized['metadata'] = {
                'generated_at': time.time(),
                'data_points': sum(len(series.get('data', [])) for series in serialized.get('series', [])),
                'series_count': len(serialized.get('series', []))
            }
            
            return serialized
            
        except Exception as e:
            logger.error(f"Error serializing chart data: {e}")
            return {
                "chart_type": "error",
                "series": [],
                "title": "Serialization Error",
                "metadata": {"error": str(e)}
            }
    
    def get_collection_stats(self) -> Dict[str, Any]:
        """Get collection statistics."""
        return self.collection_stats.copy()
    
    def get_chart_generation_stats(self) -> Dict[str, Any]:
        """Get chart generation statistics."""
        return self.chart_generation_stats.copy()
    
    def enable_chart_generation(self):
        """Enable automatic chart generation."""
        self.chart_generation_enabled = True
        logger.info("Chart generation enabled")
    
    def disable_chart_generation(self):
        """Disable automatic chart generation."""
        self.chart_generation_enabled = False
        logger.info("Chart generation disabled")
    
    def is_chart_generation_enabled(self) -> bool:
        """Check if chart generation is enabled."""
        return self.chart_generation_enabled
    
    def set_chart_update_rate(self, rate: float):
        """
        Set the chart update rate.
        
        Args:
            rate: Chart update frequency in Hz
        """
        if rate <= 0:
            raise ValueError("Chart update rate must be positive")
        
        self.chart_update_rate = rate
        self.chart_update_interval = 1.0 / rate
        logger.info(f"Chart update rate set to {rate}Hz")
    
    def get_robot_data_history(self, limit: Optional[int] = None) -> List[Dict[str, Any]]:
        """Get robot data history."""
        history = list(self.robot_data_history)
        if limit:
            return history[-limit:]
        return history
    
    def get_performance_data_history(self, limit: Optional[int] = None) -> List[Dict[str, Any]]:
        """Get performance data history."""
        history = list(self.performance_history)
        if limit:
            return history[-limit:]
        return history
    
    def get_ros_graph_history(self, limit: Optional[int] = None) -> List[Dict[str, Any]]:
        """Get ROS graph history."""
        history = list(self.ros_graph_history)
        if limit:
            return history[-limit:]
        return history
    
    def get_current_robot_data(self) -> RobotPositions:
        """Get current robot data."""
        return self.current_robot_data
    
    def get_current_performance_data(self) -> Optional[PerformanceMetrics]:
        """Get current performance data."""
        return self.current_performance_data
    
    def get_current_ros_data(self) -> Optional[RosGraphData]:
        """Get current ROS data."""
        return self.current_ros_data
    
    def is_collecting(self) -> bool:
        """Check if data collection is active."""
        return self.collecting
    
    def get_data_source_status(self) -> Dict[str, bool]:
        """Get status of all registered data sources."""
        return {
            'robot_data_source': self.robot_data_source is not None and self.robot_data_source.is_available(),
            'performance_data_source': self.performance_data_source is not None and self.performance_data_source.is_available(),
            'ros_data_source': self.ros_data_source is not None and self.ros_data_source.is_available(),
            'camera_data_source': self.camera_data_source is not None and self.camera_data_source.is_available()
        }
    
    # Chart generation methods
    
    def generate_trajectory_chart_data(self, enable_rolling_window: bool = True) -> ChartData:
        """
        Generate robot trajectory chart data from position history with rolling window support.
        Creates ECharts-compatible data for multi-robot trajectory visualization with real-time updates.
        
        Args:
            enable_rolling_window: Whether to apply rolling window filtering
        
        Returns:
            ChartData: Formatted trajectory chart data
        """
        try:
            robot_history = self.get_robot_data_history()
            return self.chart_generator.generate_trajectory_chart_data(
                robot_history, enable_rolling_window
            )
        except Exception as e:
            logger.error(f"Error generating trajectory chart data: {e}")
            return self.chart_generator._create_empty_trajectory_chart()
    
    def generate_performance_trend_chart_data(self) -> ChartData:
        """
        Generate performance trend chart data from metrics history.
        Creates ECharts-compatible data for performance monitoring.
        
        Returns:
            ChartData: Formatted performance trend chart data
        """
        try:
            performance_history = self.get_performance_data_history()
            return self.chart_generator.generate_performance_trend_chart_data(performance_history)
        except Exception as e:
            logger.error(f"Error generating performance trend chart data: {e}")
            return self.chart_generator._create_empty_performance_chart()
    
    def generate_robot_status_chart_data(self) -> ChartData:
        """
        Generate robot status chart data from position history.
        Creates ECharts-compatible data for robot status monitoring.
        
        Returns:
            ChartData: Formatted robot status chart data
        """
        try:
            robot_history = self.get_robot_data_history()
            return self.chart_generator.generate_robot_status_chart_data(robot_history)
        except Exception as e:
            logger.error(f"Error generating robot status chart data: {e}")
            return self.chart_generator._create_empty_status_chart()
    
    def generate_chart_data(self, chart_type: str, **kwargs) -> ChartData:
        """
        Generate chart data based on chart type with enhanced options.
        
        Args:
            chart_type: Type of chart to generate ('robot_trajectories', 'performance_trends', 'robot_status')
            **kwargs: Additional options for chart generation
            
        Returns:
            ChartData: Generated chart data
        """
        try:
            if chart_type == "robot_trajectories":
                # Use enhanced trajectory generation with rolling window
                enable_rolling_window = kwargs.get('enable_rolling_window', True)
                return self.generate_trajectory_chart_data(enable_rolling_window)
            
            elif chart_type == "performance_trends":
                return self.generate_performance_trend_chart_data()
            
            elif chart_type == "robot_status":
                return self.generate_robot_status_chart_data()
            
            else:
                # Fallback to chart generator
                data_history = {
                    'robot_data': self.get_robot_data_history(),
                    'performance_data': self.get_performance_data_history()
                }
                return self.chart_generator.generate_chart_data(chart_type, data_history)
            
        except Exception as e:
            logger.error(f"Error generating chart data for type {chart_type}: {e}")
            return ChartData(chart_type="line", series=[], title="Error Loading Chart")
    
    def get_available_chart_types(self) -> List[str]:
        """Get list of available chart types."""
        return self.chart_generator.get_available_chart_types()
    
    def set_trajectory_window(self, window_seconds: float):
        """
        Set the rolling window size for trajectory display.
        
        Args:
            window_seconds: Rolling window size in seconds
        """
        self.chart_generator.set_trajectory_window(window_seconds)
    
    def get_trajectory_cache_info(self) -> Dict[str, Any]:
        """Get trajectory cache information."""
        return self.chart_generator.get_trajectory_cache_info()
    
    def clear_trajectory_cache(self):
        """Clear trajectory cache for all robots."""
        self.chart_generator.clear_trajectory_cache()
    
    def has_trajectory_updates(self, since_timestamp: float) -> bool:
        """Check if there have been trajectory updates since timestamp."""
        return self.chart_generator.has_trajectory_updates(since_timestamp)
    
    def get_robot_trajectory_info(self, robot_id: str) -> Optional[Dict[str, Any]]:
        """Get trajectory information for a specific robot."""
        return self.chart_generator.get_robot_trajectory_info(robot_id)
    
    async def _adjust_collection_rate_if_needed(self, collection_time: float):
        """
        Adaptively adjust collection rate based on performance to minimize Isaac Sim impact.
        
        Args:
            collection_time: Time taken for the last collection cycle
        """
        current_time = time.time()
        
        # Only adjust every 5 seconds to avoid frequent changes
        if current_time - self.last_rate_adjustment < 5.0:
            return
        
        self.performance_history_for_adaptation.append(collection_time)
        
        # Need at least 5 samples to make decisions
        if len(self.performance_history_for_adaptation) < 5:
            return
        
        avg_collection_time = sum(self.performance_history_for_adaptation) / len(self.performance_history_for_adaptation)
        target_time = self.collection_interval * 0.5  # Target 50% of interval time
        
        # If collection is taking too long, reduce frequency
        if avg_collection_time > target_time:
            new_rate = max(1.0, self.collection_rate * 0.8)  # Reduce by 20%, minimum 1Hz
            if new_rate != self.collection_rate:
                self.collection_rate = new_rate
                self.collection_interval = 1.0 / new_rate
                logger.info(f"Reduced collection rate to {new_rate:.1f}Hz to improve Isaac Sim performance")
                self.last_rate_adjustment = current_time
        
        # If collection is fast and we're below base rate, increase frequency
        elif avg_collection_time < target_time * 0.5 and self.collection_rate < self.base_collection_rate:
            new_rate = min(self.base_collection_rate, self.collection_rate * 1.2)  # Increase by 20%
            if new_rate != self.collection_rate:
                self.collection_rate = new_rate
                self.collection_interval = 1.0 / new_rate
                logger.info(f"Increased collection rate to {new_rate:.1f}Hz")
                self.last_rate_adjustment = current_time
    
    def set_adaptive_collection(self, enabled: bool):
        """Enable or disable adaptive collection rate."""
        self.adaptive_collection_enabled = enabled
        if not enabled:
            # Reset to base rate
            self.collection_rate = self.base_collection_rate
            self.collection_interval = 1.0 / self.base_collection_rate
        logger.info(f"Adaptive collection rate {'enabled' if enabled else 'disabled'}")
    
    def get_collection_performance_info(self) -> Dict[str, Any]:
        """Get information about collection performance."""
        return {
            'base_rate': self.base_collection_rate,
            'current_rate': self.collection_rate,
            'adaptive_enabled': self.adaptive_collection_enabled,
            'recent_collection_times': list(self.performance_history_for_adaptation),
            'average_collection_time': sum(self.performance_history_for_adaptation) / len(self.performance_history_for_adaptation) if self.performance_history_for_adaptation else 0
        }


# Default implementations for testing and fallback

class DefaultPerformanceDataSource(PerformanceDataSource):
    """Default implementation using system metrics."""
    
    def get_data(self) -> PerformanceMetrics:
        return PerformanceMetrics(
            fps=self.get_fps(),
            cpu_usage=self.get_cpu_usage(),
            memory_usage=self.get_memory_usage(),
            ros_message_rate=self.get_ros_message_rate()
        )
    
    def is_available(self) -> bool:
        return True
    
    def get_fps(self) -> float:
        # Default FPS - would be overridden by Isaac Sim integration
        return 60.0
    
    def get_cpu_usage(self) -> float:
        return psutil.cpu_percent(interval=None)
    
    def get_memory_usage(self) -> float:
        return psutil.virtual_memory().percent
    
    def get_ros_message_rate(self) -> float:
        # Default ROS message rate - would be overridden by ROS integration
        return 30.0


class DefaultRobotDataSource(RobotDataSource):
    """Default implementation for testing."""
    
    def __init__(self):
        self.robot_count = 3
        self.positions = {}
        self._initialize_test_robots()
    
    def _initialize_test_robots(self):
        """Initialize test robot positions."""
        for i in range(self.robot_count):
            robot_id = f"robot_{i}"
            self.positions[robot_id] = RobotPosition(
                x=float(i * 2),
                y=float(i * 1.5),
                z=0.0,
                orientation={'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
                status=RobotStatus.ACTIVE,
                battery_level=100.0 - (i * 10)
            )
    
    def get_data(self) -> RobotPositions:
        return self.get_robot_positions()
    
    def is_available(self) -> bool:
        return True
    
    def get_robot_positions(self) -> RobotPositions:
        # Simulate slight movement for testing
        current_time = time.time()
        for robot_id, pos in self.positions.items():
            pos.x += 0.01 * np.sin(current_time + hash(robot_id) % 10)
            pos.y += 0.01 * np.cos(current_time + hash(robot_id) % 10)
        
        return self.positions.copy()
    
    def get_robot_count(self) -> int:
        return self.robot_count


class DefaultRosDataSource(RosDataSource):
    """Default implementation for testing."""
    
    def get_data(self) -> RosGraphData:
        return RosGraphData(
            nodes=self.get_ros_nodes(),
            topics=self.get_ros_topics(),
            connections=self.get_ros_connections()
        )
    
    def is_available(self) -> bool:
        return True
    
    def get_ros_nodes(self) -> List[RosNode]:
        return [
            RosNode(id="node_1", name="/robot_controller", namespace="/robot1"),
            RosNode(id="node_2", name="/sensor_publisher", namespace="/robot1"),
            RosNode(id="node_3", name="/navigation", namespace="/robot1")
        ]
    
    def get_ros_topics(self) -> List[RosTopic]:
        return [
            RosTopic(name="/cmd_vel", type="geometry_msgs/Twist", publishers=["node_1"], subscribers=["node_3"]),
            RosTopic(name="/sensor_data", type="sensor_msgs/LaserScan", publishers=["node_2"], subscribers=["node_3"])
        ]
    
    def get_ros_connections(self) -> List[RosConnection]:
        return [
            RosConnection(from_node="node_1", to_node="node_3", topic="/cmd_vel"),
            RosConnection(from_node="node_2", to_node="node_3", topic="/sensor_data")
        ]


class DefaultCameraDataSource(CameraDataSource):
    """Default implementation for testing."""
    
    def get_data(self) -> Optional[np.ndarray]:
        return self.get_camera_frame()
    
    def is_available(self) -> bool:
        return True
    
    def get_camera_frame(self, camera_id: str = "default") -> Optional[np.ndarray]:
        # Generate a test frame (would be replaced by Isaac Sim integration)
        return np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    
    def get_available_cameras(self) -> List[str]:
        return ["default", "robot_1_camera", "robot_2_camera"]