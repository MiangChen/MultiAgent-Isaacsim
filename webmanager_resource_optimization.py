#!/usr/bin/env python3
"""
WebManager Resource Optimization Configuration

This module provides various optimization strategies to reduce WebManager's resource usage
while maintaining essential functionality.
"""

import logging
from typing import Dict, Any, Optional
from dataclasses import dataclass
from enum import Enum

logger = logging.getLogger(__name__)


class OptimizationLevel(Enum):
    """Resource optimization levels."""
    MINIMAL = "minimal"          # Absolute minimum resources
    LOW = "low"                  # Low resource usage
    BALANCED = "balanced"        # Balance between features and resources
    FULL = "full"               # Full features, higher resource usage


@dataclass
class ResourceOptimizationConfig:
    """Configuration for resource optimization."""
    
    # Data collection optimization
    data_collection_rate: float = 2.0          # Reduced from 10Hz to 2Hz
    max_history: int = 200                     # Reduced from 1000 to 200
    adaptive_collection: bool = True           # Enable adaptive rate adjustment
    
    # Chart generation optimization
    chart_update_rate: float = 0.5             # Reduced from 2Hz to 0.5Hz
    chart_generation_enabled: bool = True      # Can disable entirely
    max_trajectory_points: int = 100           # Reduced from 500
    max_performance_points: int = 50           # Reduced from 200
    
    # Camera optimization
    camera_streaming_enabled: bool = False     # Disable by default
    camera_quality: int = 60                   # Reduced from 80
    camera_fps: float = 1.0                    # 1 FPS instead of real-time
    camera_resolution: tuple = (640, 480)      # Reduced resolution
    
    # WebSocket optimization
    websocket_compression: bool = True         # Enable compression
    broadcast_throttling: bool = True          # Throttle broadcasts
    max_connections: int = 5                   # Limit connections
    
    # Memory optimization
    enable_data_cleanup: bool = True           # Periodic cleanup
    cleanup_interval: float = 30.0             # Cleanup every 30s
    memory_limit_mb: int = 100                 # Memory usage limit
    
    # CPU optimization
    thread_priority: int = 5                   # Lower thread priority
    cpu_throttling: bool = True                # Enable CPU throttling
    max_cpu_usage: float = 10.0                # Max 10% CPU usage
    
    # Feature toggles
    disable_ros_monitoring: bool = False       # Keep ROS monitoring
    disable_performance_monitoring: bool = False  # Keep performance monitoring
    disable_robot_tracking: bool = False       # Keep robot tracking


class WebManagerResourceOptimizer:
    """Manages WebManager resource optimization."""
    
    def __init__(self):
        self.current_config: Optional[ResourceOptimizationConfig] = None
        self.optimization_level = OptimizationLevel.BALANCED
        
    def get_optimization_config(self, level: OptimizationLevel) -> ResourceOptimizationConfig:
        """Get optimization configuration for specified level."""
        
        if level == OptimizationLevel.MINIMAL:
            return ResourceOptimizationConfig(
                # Absolute minimum settings
                data_collection_rate=1.0,
                max_history=50,
                chart_update_rate=0.2,
                chart_generation_enabled=False,
                camera_streaming_enabled=False,
                max_connections=2,
                memory_limit_mb=50,
                max_cpu_usage=5.0,
                disable_performance_monitoring=True
            )
            
        elif level == OptimizationLevel.LOW:
            return ResourceOptimizationConfig(
                # Low resource settings
                data_collection_rate=1.5,
                max_history=100,
                chart_update_rate=0.3,
                chart_generation_enabled=True,
                max_trajectory_points=50,
                max_performance_points=25,
                camera_streaming_enabled=False,
                max_connections=3,
                memory_limit_mb=75,
                max_cpu_usage=7.5
            )
            
        elif level == OptimizationLevel.BALANCED:
            return ResourceOptimizationConfig(
                # Balanced settings (default)
                data_collection_rate=2.0,
                max_history=200,
                chart_update_rate=0.5,
                chart_generation_enabled=True,
                max_trajectory_points=100,
                max_performance_points=50,
                camera_streaming_enabled=False,
                camera_quality=60,
                max_connections=5,
                memory_limit_mb=100,
                max_cpu_usage=10.0
            )
            
        else:  # FULL
            return ResourceOptimizationConfig(
                # Full feature settings
                data_collection_rate=5.0,
                max_history=500,
                chart_update_rate=1.0,
                chart_generation_enabled=True,
                max_trajectory_points=300,
                max_performance_points=150,
                camera_streaming_enabled=True,
                camera_quality=80,
                camera_fps=2.0,
                max_connections=10,
                memory_limit_mb=200,
                max_cpu_usage=20.0
            )
    
    def apply_optimization(self, webmanager_system, level: OptimizationLevel):
        """Apply optimization configuration to WebManager system."""
        try:
            config = self.get_optimization_config(level)
            self.current_config = config
            self.optimization_level = level
            
            logger.info(f"Applying {level.value} optimization to WebManager")
            
            # Apply data collection optimization
            if hasattr(webmanager_system, 'data_collector'):
                self._optimize_data_collector(webmanager_system.data_collector, config)
            
            # Apply web server optimization
            if hasattr(webmanager_system, 'web_server'):
                self._optimize_web_server(webmanager_system.web_server, config)
            
            # Apply camera optimization
            if hasattr(webmanager_system, 'camera_data_source'):
                self._optimize_camera_system(webmanager_system.camera_data_source, config)
            
            logger.info(f"WebManager optimization applied successfully")
            
        except Exception as e:
            logger.error(f"Error applying optimization: {e}")
    
    def _optimize_data_collector(self, data_collector, config: ResourceOptimizationConfig):
        """Optimize data collector settings."""
        try:
            # Set collection rate
            data_collector.collection_rate = config.data_collection_rate
            data_collector.collection_interval = 1.0 / config.data_collection_rate
            
            # Set history limits
            data_collector.max_history = config.max_history
            data_collector.robot_data_history = data_collector.robot_data_history.__class__(
                data_collector.robot_data_history, maxlen=config.max_history
            )
            data_collector.performance_history = data_collector.performance_history.__class__(
                data_collector.performance_history, maxlen=config.max_history
            )
            data_collector.ros_graph_history = data_collector.ros_graph_history.__class__(
                data_collector.ros_graph_history, maxlen=config.max_history
            )
            
            # Set chart generation settings
            if config.chart_generation_enabled:
                data_collector.enable_chart_generation()
                data_collector.set_chart_update_rate(config.chart_update_rate)
                
                # Update chart generator settings
                if hasattr(data_collector, 'chart_generator'):
                    data_collector.chart_generator.max_trajectory_points = config.max_trajectory_points
                    data_collector.chart_generator.max_performance_points = config.max_performance_points
            else:
                data_collector.disable_chart_generation()
            
            # Enable adaptive collection
            data_collector.adaptive_collection_enabled = config.adaptive_collection
            
            logger.info(f"Data collector optimized: {config.data_collection_rate}Hz collection, "
                       f"{config.max_history} max history, charts {'enabled' if config.chart_generation_enabled else 'disabled'}")
            
        except Exception as e:
            logger.error(f"Error optimizing data collector: {e}")
    
    def _optimize_web_server(self, web_server, config: ResourceOptimizationConfig):
        """Optimize web server settings."""
        try:
            # Set connection limits
            if hasattr(web_server.websocket_manager, 'max_connections'):
                web_server.websocket_manager.max_connections = config.max_connections
            
            # Enable compression if supported
            if hasattr(web_server.websocket_manager, 'enable_compression'):
                web_server.websocket_manager.enable_compression = config.websocket_compression
            
            # Set broadcast throttling
            if hasattr(web_server.websocket_manager, 'broadcast_throttling'):
                web_server.websocket_manager.broadcast_throttling = config.broadcast_throttling
            
            logger.info(f"Web server optimized: max {config.max_connections} connections, "
                       f"compression {'enabled' if config.websocket_compression else 'disabled'}")
            
        except Exception as e:
            logger.error(f"Error optimizing web server: {e}")
    
    def _optimize_camera_system(self, camera_data_source, config: ResourceOptimizationConfig):
        """Optimize camera system settings."""
        try:
            if not config.camera_streaming_enabled:
                # Disable all cameras
                for camera_id in camera_data_source.get_available_cameras():
                    camera_data_source.set_camera_active(camera_id, False)
                logger.info("Camera streaming disabled for resource optimization")
            else:
                # Set camera quality and resolution
                camera_data_source.default_quality = config.camera_quality
                camera_data_source.max_frame_size = config.camera_resolution
                
                logger.info(f"Camera optimized: quality={config.camera_quality}, "
                           f"resolution={config.camera_resolution}, fps={config.camera_fps}")
            
        except Exception as e:
            logger.error(f"Error optimizing camera system: {e}")
    
    def get_resource_usage_report(self, webmanager_system) -> Dict[str, Any]:
        """Get current resource usage report."""
        try:
            import psutil
            import os
            
            # Get process info
            process = psutil.Process(os.getpid())
            
            # Memory usage
            memory_info = process.memory_info()
            memory_mb = memory_info.rss / 1024 / 1024
            
            # CPU usage
            cpu_percent = process.cpu_percent()
            
            # WebManager specific stats
            webmanager_stats = {}
            
            if hasattr(webmanager_system, 'data_collector'):
                dc = webmanager_system.data_collector
                webmanager_stats.update({
                    'collection_rate': dc.collection_rate,
                    'chart_update_rate': dc.chart_update_rate if hasattr(dc, 'chart_update_rate') else 0,
                    'history_size': len(dc.robot_data_history) + len(dc.performance_history) + len(dc.ros_graph_history),
                    'collection_stats': dc.get_collection_stats(),
                    'chart_generation_enabled': dc.is_chart_generation_enabled()
                })
            
            if hasattr(webmanager_system, 'web_server'):
                ws = webmanager_system.web_server
                webmanager_stats.update({
                    'active_connections': len(ws.websocket_manager.active_connections),
                    'server_running': ws.running
                })
            
            report = {
                'timestamp': __import__('time').time(),
                'optimization_level': self.optimization_level.value if self.optimization_level else 'none',
                'system_resources': {
                    'memory_mb': round(memory_mb, 2),
                    'cpu_percent': round(cpu_percent, 2),
                    'memory_limit_mb': self.current_config.memory_limit_mb if self.current_config else None,
                    'cpu_limit_percent': self.current_config.max_cpu_usage if self.current_config else None
                },
                'webmanager_stats': webmanager_stats,
                'optimization_config': self.current_config.__dict__ if self.current_config else None
            }
            
            return report
            
        except Exception as e:
            logger.error(f"Error generating resource usage report: {e}")
            return {'error': str(e)}
    
    def auto_optimize_based_on_usage(self, webmanager_system) -> bool:
        """Automatically optimize based on current resource usage."""
        try:
            report = self.get_resource_usage_report(webmanager_system)
            
            if 'error' in report:
                return False
            
            memory_mb = report['system_resources']['memory_mb']
            cpu_percent = report['system_resources']['cpu_percent']
            
            # Determine if optimization is needed
            current_level = self.optimization_level
            new_level = current_level
            
            # High resource usage - move to more aggressive optimization
            if memory_mb > 150 or cpu_percent > 15:
                if current_level == OptimizationLevel.FULL:
                    new_level = OptimizationLevel.BALANCED
                elif current_level == OptimizationLevel.BALANCED:
                    new_level = OptimizationLevel.LOW
                elif current_level == OptimizationLevel.LOW:
                    new_level = OptimizationLevel.MINIMAL
            
            # Low resource usage - can enable more features
            elif memory_mb < 50 and cpu_percent < 5:
                if current_level == OptimizationLevel.MINIMAL:
                    new_level = OptimizationLevel.LOW
                elif current_level == OptimizationLevel.LOW:
                    new_level = OptimizationLevel.BALANCED
            
            # Apply new optimization if changed
            if new_level != current_level:
                logger.info(f"Auto-optimizing from {current_level.value} to {new_level.value} "
                           f"(Memory: {memory_mb:.1f}MB, CPU: {cpu_percent:.1f}%)")
                self.apply_optimization(webmanager_system, new_level)
                return True
            
            return False
            
        except Exception as e:
            logger.error(f"Error in auto-optimization: {e}")
            return False


def create_optimized_webmanager_config(optimization_level: OptimizationLevel = OptimizationLevel.BALANCED) -> Dict[str, Any]:
    """Create optimized WebManager configuration dictionary."""
    
    optimizer = WebManagerResourceOptimizer()
    config = optimizer.get_optimization_config(optimization_level)
    
    return {
        'data_collection_rate': config.data_collection_rate,
        'max_history': config.max_history,
        'chart_update_rate': config.chart_update_rate,
        'enable_camera_streaming': config.camera_streaming_enabled,
        'camera_quality': config.camera_quality,
        'enable_compression': config.websocket_compression,
        'optimization_level': optimization_level.value,
        'memory_limit_mb': config.memory_limit_mb,
        'max_cpu_usage': config.max_cpu_usage
    }


def get_resource_friendly_startup_args() -> list:
    """Get command line arguments for resource-friendly WebManager startup."""
    
    return [
        '--data-collection-rate', '2.0',
        '--max-history', '200',
        '--disable-camera-streaming',
        '--camera-quality', '60',
        '--enable-compression',
        '--webmanager-low-impact',
        '--log-level', 'WARNING'  # Reduce log verbosity
    ]


if __name__ == '__main__':
    # Test the optimization configurations
    optimizer = WebManagerResourceOptimizer()
    
    print("WebManager Resource Optimization Configurations")
    print("=" * 50)
    
    for level in OptimizationLevel:
        config = optimizer.get_optimization_config(level)
        print(f"\n{level.value.upper()} Optimization:")
        print(f"  Data collection: {config.data_collection_rate}Hz")
        print(f"  Max history: {config.max_history}")
        print(f"  Chart updates: {config.chart_update_rate}Hz")
        print(f"  Camera streaming: {'Yes' if config.camera_streaming_enabled else 'No'}")
        print(f"  Memory limit: {config.memory_limit_mb}MB")
        print(f"  CPU limit: {config.max_cpu_usage}%")
    
    print(f"\nResource-friendly startup args:")
    print(' '.join(get_resource_friendly_startup_args()))