"""
WebManager package for Isaac Sim web-based monitoring and control.
"""

from .web_server import WebUIServer, WebSocketManager
from .models import (
    RobotPosition, RobotPositions, PerformanceMetrics, 
    RosGraphData, RosNode, RosTopic, RosConnection,
    ChartData, ChartSeries, WebSocketMessage, MessageType
)
from .data_collector import DataCollector
from .robot_data_collector import SwarmManagerRobotDataSource
from .performance_data_collector import IsaacSimPerformanceDataSource, ROSMessageRateMonitor
from .ros_data_collector import ROS2GraphDataSource, ROSGraphMonitor
from .camera_data_collector import IsaacSimCameraDataSource, CameraStreamManager

__version__ = "1.0.0"
__author__ = "WebManager Development Team"

__all__ = [
    # Core components
    'WebUIServer',
    'WebSocketManager', 
    'DataCollector',
    
    # Data sources
    'SwarmManagerRobotDataSource',
    'IsaacSimPerformanceDataSource',
    'ROS2GraphDataSource',
    'IsaacSimCameraDataSource',
    
    # Utilities
    'CameraStreamManager',
    'ROSMessageRateMonitor',
    'ROSGraphMonitor',
    
    # Data models
    'RobotPosition',
    'RobotPositions',
    'PerformanceMetrics',
    'RosGraphData',
    'RosNode',
    'RosTopic', 
    'RosConnection',
    'ChartData',
    'ChartSeries',
    'WebSocketMessage',
    'MessageType'
]