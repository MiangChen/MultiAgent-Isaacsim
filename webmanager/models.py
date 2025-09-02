"""
WebSocket message interfaces and data models for the WebManager system.
"""

from typing import Dict, List, Any, Optional, Union
from dataclasses import dataclass
from enum import Enum
import time


class MessageType(Enum):
    """WebSocket message types."""
    INITIAL_DATA = "initial_data"
    DATA_UPDATE = "data_update"
    CHART_DATA = "chart_data"
    BINARY_DATA = "binary_data"
    REQUEST_CHART_UPDATE = "request_chart_update"
    CAMERA_CONTROL = "camera_control"


class BinaryDataType(Enum):
    """Binary data types for WebSocket transmission."""
    CAMERA_FRAME = "camera_frame"
    SENSOR_DATA = "sensor_data"
    POINT_CLOUD = "point_cloud"
    DEPTH_IMAGE = "depth_image"


class RobotStatus(Enum):
    """Robot status enumeration."""
    ACTIVE = "active"
    INACTIVE = "inactive"
    ERROR = "error"


@dataclass
class RobotPosition:
    """Robot position and status data."""
    x: float
    y: float
    z: Optional[float] = None
    orientation: Optional[Dict[str, float]] = None  # roll, pitch, yaw
    status: RobotStatus = RobotStatus.ACTIVE
    battery_level: Optional[float] = None
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        return {
            'x': self.x,
            'y': self.y,
            'z': self.z,
            'orientation': self.orientation,
            'status': self.status.value,
            'battery_level': self.battery_level
        }


@dataclass
class PerformanceMetrics:
    """System performance metrics."""
    fps: float
    cpu_usage: float
    memory_usage: float
    ros_message_rate: float
    timestamp: float = None
    
    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = time.time()
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        return {
            'fps': self.fps,
            'cpu_usage': self.cpu_usage,
            'memory_usage': self.memory_usage,
            'ros_message_rate': self.ros_message_rate,
            'timestamp': self.timestamp
        }


@dataclass
class RosNode:
    """ROS node information."""
    id: str
    name: str
    namespace: Optional[str] = None
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        return {
            'id': self.id,
            'name': self.name,
            'namespace': self.namespace
        }


@dataclass
class RosTopic:
    """ROS topic information."""
    name: str
    type: str
    publishers: List[str]
    subscribers: List[str]
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        return {
            'name': self.name,
            'type': self.type,
            'publishers': self.publishers,
            'subscribers': self.subscribers
        }


@dataclass
class RosConnection:
    """ROS node connection information."""
    from_node: str
    to_node: str
    topic: str
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        return {
            'from': self.from_node,
            'to': self.to_node,
            'topic': self.topic
        }


@dataclass
class RosGraphData:
    """Complete ROS graph data structure."""
    nodes: List[RosNode]
    topics: List[RosTopic]
    connections: List[RosConnection]
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        return {
            'nodes': [node.to_dict() for node in self.nodes],
            'topics': [topic.to_dict() for topic in self.topics],
            'connections': [conn.to_dict() for conn in self.connections]
        }


@dataclass
class ChartSeries:
    """Chart series data structure."""
    name: str
    data: Union[List[List[float]], List[float]]
    type: str = "line"
    color: Optional[str] = None
    metadata: Optional[Dict[str, Any]] = None
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        result = {
            'name': self.name,
            'data': self.data,
            'type': self.type,
            'color': self.color
        }
        if self.metadata:
            result['metadata'] = self.metadata
        return result


@dataclass
class ChartData:
    """Chart data structure for ECharts."""
    chart_type: str
    series: List[ChartSeries]
    x_axis: Optional[Union[List[float], List[str]]] = None
    title: Optional[str] = None
    x_axis_label: Optional[str] = None
    y_axis_label: Optional[str] = None
    metadata: Optional[Dict[str, Any]] = None
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        result = {
            'chart_type': self.chart_type,
            'series': [series.to_dict() for series in self.series],
            'xAxis': self.x_axis,
            'title': self.title,
            'xAxisLabel': self.x_axis_label,
            'yAxisLabel': self.y_axis_label
        }
        if self.metadata:
            result['metadata'] = self.metadata
        return result


@dataclass
class WebSocketMessage:
    """Base WebSocket message structure."""
    type: MessageType
    data_type: Optional[str] = None
    data: Optional[Any] = None
    timestamp: float = None
    
    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = time.time()
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        return {
            'type': self.type.value,
            'data_type': self.data_type,
            'data': self.data,
            'timestamp': self.timestamp
        }


@dataclass
class BinaryDataMetadata:
    """Metadata for binary data transmission."""
    data_type: BinaryDataType
    size: int
    format: Optional[str] = None
    quality: Optional[int] = None
    width: Optional[int] = None
    height: Optional[int] = None
    camera_id: Optional[str] = None
    timestamp: float = None
    
    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = time.time()
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        return {
            'data_type': self.data_type.value,
            'size': self.size,
            'format': self.format,
            'quality': self.quality,
            'width': self.width,
            'height': self.height,
            'camera_id': self.camera_id,
            'timestamp': self.timestamp
        }


# Type aliases for convenience
RobotPositions = Dict[str, RobotPosition]