"""
Performance metrics data collection implementation for the WebManager.
Handles FPS, CPU, memory, and ROS message rate monitoring with history tracking.
"""

import time
import logging
import threading
import subprocess
from typing import Dict, List, Optional, Any, Callable
from collections import deque
import psutil
import numpy as np

from .models import PerformanceMetrics
from .data_collector import PerformanceDataSource

logger = logging.getLogger(__name__)


class IsaacSimPerformanceDataSource(PerformanceDataSource):
    """
    Performance data source that integrates with Isaac Sim and ROS systems.
    Provides comprehensive system monitoring including FPS, CPU, memory, and ROS metrics.
    """
    
    def __init__(self, isaac_sim_app=None, ros_monitor=None):
        """
        Initialize the performance data source.
        
        Args:
            isaac_sim_app: Reference to Isaac Sim application instance
            ros_monitor: Reference to ROS monitoring system
        """
        self.isaac_sim_app = isaac_sim_app
        self.ros_monitor = ros_monitor
        
        # Performance tracking
        self.fps_history = deque(maxlen=100)
        self.cpu_history = deque(maxlen=100)
        self.memory_history = deque(maxlen=100)
        self.ros_rate_history = deque(maxlen=100)
        
        # Timing for FPS calculation
        self.last_frame_time = time.time()
        self.frame_count = 0
        self.fps_calculation_window = 1.0  # Calculate FPS over 1 second
        
        # ROS message rate tracking
        self.ros_message_counts = {}
        self.ros_rate_calculation_window = 5.0  # Calculate rates over 5 seconds
        self.last_ros_calculation_time = time.time()
        
        # Performance thresholds for warnings
        self.performance_thresholds = {
            'fps_warning': 30.0,
            'fps_critical': 15.0,
            'cpu_warning': 85.0,
            'cpu_critical': 98.0,
            'memory_warning': 85.0,
            'memory_critical': 95.0,
            'ros_rate_warning': 10.0
        }
        
        # System monitoring
        self.process = psutil.Process()
        self.system_start_time = time.time()
        
        # Warning throttling to reduce log spam
        self.last_warning_time = {}
        self.warning_cooldown = 30.0  # 30 seconds between same type warnings
        
        logger.info("IsaacSimPerformanceDataSource initialized")
    
    def _should_log_warning(self, warning_type: str, current_time: float) -> bool:
        """
        Check if a warning should be logged based on cooldown period.
        
        Args:
            warning_type: Type of warning (e.g., 'cpu_critical', 'memory_warning')
            current_time: Current timestamp
            
        Returns:
            True if warning should be logged, False otherwise
        """
        last_time = self.last_warning_time.get(warning_type, 0)
        if current_time - last_time >= self.warning_cooldown:
            self.last_warning_time[warning_type] = current_time
            return True
        return False
    
    def get_data(self) -> PerformanceMetrics:
        """Get current performance metrics."""
        return PerformanceMetrics(
            fps=self.get_fps(),
            cpu_usage=self.get_cpu_usage(),
            memory_usage=self.get_memory_usage(),
            ros_message_rate=self.get_ros_message_rate()
        )
    
    def is_available(self) -> bool:
        """Check if the performance data source is available."""
        return True  # System metrics are always available
    
    def get_fps(self) -> float:
        """
        Get current simulation FPS.
        Integrates with Isaac Sim's frame timing if available.
        """
        try:
            # Try to get FPS from Isaac Sim application
            if self.isaac_sim_app and hasattr(self.isaac_sim_app, 'get_fps'):
                fps = self.isaac_sim_app.get_fps()
                self.fps_history.append(fps)
                return fps
            
            # Try to get from Isaac Sim timeline
            elif self.isaac_sim_app:
                try:
                    import omni.timeline
                    timeline = omni.timeline.get_timeline_interface()
                    if timeline:
                        # Get current time step
                        current_time = timeline.get_current_time()
                        if hasattr(self, '_last_timeline_time'):
                            time_diff = current_time - self._last_timeline_time
                            if time_diff > 0:
                                fps = 1.0 / time_diff
                                self.fps_history.append(fps)
                                self._last_timeline_time = current_time
                                return fps
                        else:
                            self._last_timeline_time = current_time
                except ImportError:
                    pass
            
            # Try to get from simulation context
            if self.isaac_sim_app and hasattr(self.isaac_sim_app, 'get_simulation_context'):
                sim_context = self.isaac_sim_app.get_simulation_context()
                if hasattr(sim_context, 'get_physics_dt'):
                    physics_dt = sim_context.get_physics_dt()
                    fps = 1.0 / physics_dt if physics_dt > 0 else 60.0
                    self.fps_history.append(fps)
                    return fps
            
            # Fallback: calculate FPS based on frame timing
            current_time = time.time()
            self.frame_count += 1
            
            time_elapsed = current_time - self.last_frame_time
            if time_elapsed >= self.fps_calculation_window:
                fps = self.frame_count / time_elapsed
                self.fps_history.append(fps)
                
                # Reset for next calculation
                self.frame_count = 0
                self.last_frame_time = current_time
                
                return fps
            
            # Return last calculated FPS if available
            return self.fps_history[-1] if self.fps_history else 60.0
            
        except Exception as e:
            logger.error(f"Error getting FPS: {e}")
            return 60.0  # Default fallback
    
    def get_cpu_usage(self) -> float:
        """
        Get current CPU usage percentage.
        Provides both system-wide and process-specific CPU usage.
        """
        try:
            # Get system-wide CPU usage
            system_cpu = psutil.cpu_percent(interval=None)
            
            # Get process-specific CPU usage for Isaac Sim
            try:
                process_cpu = self.process.cpu_percent(interval=None)
                
                # Normalize process CPU to percentage (psutil can return > 100% on multi-core)
                # Use system CPU as the primary metric, process CPU as additional info
                cpu_usage = system_cpu
                
                # Log process CPU separately if significantly different
                if abs(process_cpu - system_cpu) > 20:
                    logger.debug(f"Process CPU: {process_cpu:.1f}%, System CPU: {system_cpu:.1f}%")
                    
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                cpu_usage = system_cpu
            
            self.cpu_history.append(cpu_usage)
            
            # Log warning if CPU usage is high (with throttling)
            current_time = time.time()
            if cpu_usage > self.performance_thresholds['cpu_critical']:
                if self._should_log_warning('cpu_critical', current_time):
                    logger.warning(f"Critical CPU usage: {cpu_usage:.1f}%")
            elif cpu_usage > self.performance_thresholds['cpu_warning']:
                if self._should_log_warning('cpu_warning', current_time):
                    logger.info(f"High CPU usage: {cpu_usage:.1f}%")
            
            return cpu_usage
            
        except Exception as e:
            logger.error(f"Error getting CPU usage: {e}")
            return 0.0
    
    def get_memory_usage(self) -> float:
        """
        Get current memory usage percentage.
        Provides both system-wide and process-specific memory usage.
        """
        try:
            # Get system-wide memory usage
            system_memory = psutil.virtual_memory()
            system_memory_percent = system_memory.percent
            
            # Get process-specific memory usage for Isaac Sim
            try:
                process_memory = self.process.memory_info()
                process_memory_percent = (process_memory.rss / system_memory.total) * 100
                
                # Use system memory percentage but log process usage for debugging
                memory_usage = system_memory_percent
                
                logger.debug(f"Memory usage - System: {system_memory_percent:.1f}%, "
                           f"Process: {process_memory_percent:.1f}% "
                           f"({process_memory.rss / (1024**3):.1f} GB)")
                
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                memory_usage = system_memory_percent
            
            self.memory_history.append(memory_usage)
            
            # Log warning if memory usage is high (with throttling)
            current_time = time.time()
            if memory_usage > self.performance_thresholds['memory_critical']:
                if self._should_log_warning('memory_critical', current_time):
                    logger.warning(f"Critical memory usage: {memory_usage:.1f}%")
            elif memory_usage > self.performance_thresholds['memory_warning']:
                if self._should_log_warning('memory_warning', current_time):
                    logger.info(f"High memory usage: {memory_usage:.1f}%")
            
            return memory_usage
            
        except Exception as e:
            logger.error(f"Error getting memory usage: {e}")
            return 0.0    

    def get_ros_message_rate(self) -> float:
        """
        Get current ROS message publishing rate.
        Monitors ROS topic message frequencies across the system.
        """
        try:
            current_time = time.time()
            
            # Use ROS monitor if available (SceneMonitorNode from main.py)
            if self.ros_monitor and hasattr(self.ros_monitor, 'get_message_rates'):
                rates = self.ros_monitor.get_message_rates()
                
                # Calculate average message rate across all topics
                if rates:
                    avg_rate = sum(rates.values()) / len(rates)
                    self.ros_rate_history.append(avg_rate)
                    return avg_rate
            
            # Alternative: try to get statistics from ROS monitor node
            elif self.ros_monitor:
                try:
                    # Try to access ROS node statistics if available
                    if hasattr(self.ros_monitor, 'get_statistics'):
                        stats = self.ros_monitor.get_statistics()
                        if stats and 'message_rate' in stats:
                            rate = stats['message_rate']
                            self.ros_rate_history.append(rate)
                            return rate
                    
                    # Try to get topic count as a proxy for activity
                    elif hasattr(self.ros_monitor, 'get_topic_count'):
                        topic_count = self.ros_monitor.get_topic_count()
                        # Estimate rate based on topic count (rough approximation)
                        estimated_rate = topic_count * 5.0  # Assume 5 Hz per topic
                        self.ros_rate_history.append(estimated_rate)
                        return estimated_rate
                        
                except Exception as e:
                    logger.debug(f"Error accessing ROS monitor statistics: {e}")
            
            # Alternative: calculate from ROS message counts using subprocess
            try:
                # Use ros2 topic list to get active topics and estimate activity
                result = subprocess.run(['ros2', 'topic', 'list'], 
                                      capture_output=True, text=True, timeout=3)
                
                if result.returncode == 0:
                    topic_count = len([line for line in result.stdout.split('\n') if line.strip()])
                    # Rough estimate: assume each topic publishes at ~10 Hz
                    estimated_rate = topic_count * 10.0
                    self.ros_rate_history.append(estimated_rate)
                    return estimated_rate
                    
            except (subprocess.TimeoutExpired, FileNotFoundError):
                pass
            
            # Fallback: return last calculated rate or default
            return self.ros_rate_history[-1] if self.ros_rate_history else 30.0
            
        except Exception as e:
            logger.error(f"Error getting ROS message rate: {e}")
            return 0.0
    
    def get_performance_summary(self) -> Dict[str, Any]:
        """
        Get comprehensive performance summary with historical data.
        
        Returns:
            Dictionary containing current metrics and historical trends
        """
        current_metrics = self.get_data()
        
        # Calculate historical statistics
        def calculate_stats(history_deque):
            if not history_deque:
                return {'min': 0, 'max': 0, 'avg': 0, 'current': 0}
            
            values = list(history_deque)
            return {
                'min': min(values),
                'max': max(values),
                'avg': sum(values) / len(values),
                'current': values[-1]
            }
        
        return {
            'current_metrics': current_metrics.to_dict(),
            'fps_stats': calculate_stats(self.fps_history),
            'cpu_stats': calculate_stats(self.cpu_history),
            'memory_stats': calculate_stats(self.memory_history),
            'ros_rate_stats': calculate_stats(self.ros_rate_history),
            'uptime_seconds': time.time() - self.system_start_time,
            'performance_warnings': self._check_performance_warnings(current_metrics),
            'timestamp': time.time()
        }
    
    def _check_performance_warnings(self, metrics: PerformanceMetrics) -> List[Dict[str, str]]:
        """
        Check for performance warnings based on current metrics.
        
        Args:
            metrics: Current performance metrics
            
        Returns:
            List of warning dictionaries with type and message
        """
        warnings = []
        
        # FPS warnings
        if metrics.fps < self.performance_thresholds['fps_critical']:
            warnings.append({
                'type': 'critical',
                'category': 'fps',
                'message': f"Critical FPS: {metrics.fps:.1f} (< {self.performance_thresholds['fps_critical']})"
            })
        elif metrics.fps < self.performance_thresholds['fps_warning']:
            warnings.append({
                'type': 'warning',
                'category': 'fps',
                'message': f"Low FPS: {metrics.fps:.1f} (< {self.performance_thresholds['fps_warning']})"
            })
        
        # CPU warnings
        if metrics.cpu_usage > self.performance_thresholds['cpu_critical']:
            warnings.append({
                'type': 'critical',
                'category': 'cpu',
                'message': f"Critical CPU usage: {metrics.cpu_usage:.1f}% (> {self.performance_thresholds['cpu_critical']}%)"
            })
        elif metrics.cpu_usage > self.performance_thresholds['cpu_warning']:
            warnings.append({
                'type': 'warning',
                'category': 'cpu',
                'message': f"High CPU usage: {metrics.cpu_usage:.1f}% (> {self.performance_thresholds['cpu_warning']}%)"
            })
        
        # Memory warnings
        if metrics.memory_usage > self.performance_thresholds['memory_critical']:
            warnings.append({
                'type': 'critical',
                'category': 'memory',
                'message': f"Critical memory usage: {metrics.memory_usage:.1f}% (> {self.performance_thresholds['memory_critical']}%)"
            })
        elif metrics.memory_usage > self.performance_thresholds['memory_warning']:
            warnings.append({
                'type': 'warning',
                'category': 'memory',
                'message': f"High memory usage: {metrics.memory_usage:.1f}% (> {self.performance_thresholds['memory_warning']}%)"
            })
        
        # ROS rate warnings
        if metrics.ros_message_rate < self.performance_thresholds['ros_rate_warning']:
            warnings.append({
                'type': 'warning',
                'category': 'ros',
                'message': f"Low ROS message rate: {metrics.ros_message_rate:.1f} Hz (< {self.performance_thresholds['ros_rate_warning']} Hz)"
            })
        
        return warnings
    
    def set_performance_thresholds(self, thresholds: Dict[str, float]):
        """
        Update performance warning thresholds.
        
        Args:
            thresholds: Dictionary of threshold values
        """
        self.performance_thresholds.update(thresholds)
        logger.info(f"Updated performance thresholds: {thresholds}")
    
    def get_system_info(self) -> Dict[str, Any]:
        """
        Get detailed system information for diagnostics.
        
        Returns:
            Dictionary containing system specifications and status
        """
        try:
            # CPU information
            cpu_info = {
                'physical_cores': psutil.cpu_count(logical=False),
                'logical_cores': psutil.cpu_count(logical=True),
                'cpu_freq': psutil.cpu_freq()._asdict() if psutil.cpu_freq() else None,
                'cpu_percent_per_core': psutil.cpu_percent(percpu=True, interval=None)
            }
            
            # Memory information
            memory = psutil.virtual_memory()
            memory_info = {
                'total_gb': memory.total / (1024**3),
                'available_gb': memory.available / (1024**3),
                'used_gb': memory.used / (1024**3),
                'percent': memory.percent
            }
            
            # Disk information
            disk = psutil.disk_usage('/')
            disk_info = {
                'total_gb': disk.total / (1024**3),
                'used_gb': disk.used / (1024**3),
                'free_gb': disk.free / (1024**3),
                'percent': (disk.used / disk.total) * 100
            }
            
            # Process information
            process_info = {
                'pid': self.process.pid,
                'memory_mb': self.process.memory_info().rss / (1024**2),
                'cpu_percent': self.process.cpu_percent(),
                'num_threads': self.process.num_threads(),
                'create_time': self.process.create_time()
            }
            
            return {
                'cpu': cpu_info,
                'memory': memory_info,
                'disk': disk_info,
                'process': process_info,
                'uptime_seconds': time.time() - self.system_start_time,
                'timestamp': time.time()
            }
            
        except Exception as e:
            logger.error(f"Error getting system info: {e}")
            return {}
    
    def reset_history(self):
        """Reset all performance history data."""
        self.fps_history.clear()
        self.cpu_history.clear()
        self.memory_history.clear()
        self.ros_rate_history.clear()
        self.ros_message_counts.clear()
        
        self.last_frame_time = time.time()
        self.frame_count = 0
        self.last_ros_calculation_time = time.time()
        
        logger.info("Performance history reset")
    
    def get_performance_trends(self, window_size: int = 50) -> Dict[str, List[float]]:
        """
        Get performance trend data for charting.
        
        Args:
            window_size: Number of recent data points to include
            
        Returns:
            Dictionary containing trend data for each metric
        """
        def get_recent_values(history_deque, size):
            values = list(history_deque)
            return values[-size:] if len(values) >= size else values
        
        return {
            'fps_trend': get_recent_values(self.fps_history, window_size),
            'cpu_trend': get_recent_values(self.cpu_history, window_size),
            'memory_trend': get_recent_values(self.memory_history, window_size),
            'ros_rate_trend': get_recent_values(self.ros_rate_history, window_size),
            'timestamps': [time.time() - i * 0.1 for i in range(window_size, 0, -1)]
        }


class ROSMessageRateMonitor:
    """
    Helper class for monitoring ROS message rates.
    Can be used as a standalone monitor or integrated with the performance data source.
    """
    
    def __init__(self):
        """Initialize the ROS message rate monitor."""
        self.topic_message_counts = {}
        self.topic_last_update = {}
        self.topic_rates = {}
        self.monitoring_active = False
        
        # Try to import ROS2 libraries
        try:
            import rclpy
            from rclpy.node import Node
            self.rclpy_available = True
        except ImportError:
            self.rclpy_available = False
            logger.warning("ROS2 libraries not available for message rate monitoring")
    
    def start_monitoring(self):
        """Start ROS message rate monitoring."""
        if not self.rclpy_available:
            logger.warning("Cannot start ROS monitoring - ROS2 libraries not available")
            return
        
        self.monitoring_active = True
        # Implementation would depend on specific ROS2 integration
        logger.info("ROS message rate monitoring started")
    
    def stop_monitoring(self):
        """Stop ROS message rate monitoring."""
        self.monitoring_active = False
        logger.info("ROS message rate monitoring stopped")
    
    def get_message_rates(self) -> Dict[str, float]:
        """Get current message rates for all monitored topics."""
        return self.topic_rates.copy()
    
    def get_message_counts(self) -> Dict[str, int]:
        """Get current message counts for all monitored topics."""
        return self.topic_message_counts.copy()