"""
ROS graph data collection implementation for the WebManager.
Handles ROS node and topic discovery, connection mapping, and real-time updates.
"""

import time
import logging
import threading
from typing import Dict, List, Optional, Set, Any
from collections import defaultdict
import subprocess
import json

from .models import RosNode, RosTopic, RosConnection, RosGraphData
from .data_collector import RosDataSource

logger = logging.getLogger(__name__)


class ROS2GraphDataSource(RosDataSource):
    """
    ROS graph data source that discovers and monitors ROS2 nodes, topics, and connections.
    Provides real-time ROS graph update detection and broadcasting.
    """
    
    def __init__(self, ros_domain_id: Optional[int] = None):
        """
        Initialize the ROS graph data source.
        
        Args:
            ros_domain_id: ROS domain ID to monitor (None for default)
        """
        self.ros_domain_id = ros_domain_id
        
        # ROS graph data
        self.nodes: Dict[str, RosNode] = {}
        self.topics: Dict[str, RosTopic] = {}
        self.connections: List[RosConnection] = []
        
        # Discovery state
        self.last_discovery_time = 0.0
        self.discovery_interval = 2.0  # Discover every 2 seconds
        self.graph_hash = ""  # For change detection
        
        # ROS2 availability
        self.ros2_available = self._check_ros2_availability()
        
        # Topic type cache
        self.topic_type_cache: Dict[str, str] = {}
        
        # Warning throttling to reduce log spam
        self.last_ros_warning = {}
        self.ros_warning_cooldown = 60.0  # 60 seconds between ROS warnings
        
        logger.info(f"ROS2GraphDataSource initialized (ROS2 available: {self.ros2_available})")
    
    def _should_log_ros_warning(self, warning_type: str) -> bool:
        """
        Check if a ROS warning should be logged based on cooldown period.
        
        Args:
            warning_type: Type of warning (e.g., 'ros2_not_found', 'discovery_timeout')
            
        Returns:
            True if warning should be logged, False otherwise
        """
        current_time = time.time()
        last_time = self.last_ros_warning.get(warning_type, 0)
        if current_time - last_time >= self.ros_warning_cooldown:
            self.last_ros_warning[warning_type] = current_time
            return True
        return False
    
    def _check_ros2_availability(self) -> bool:
        """Check if ROS2 is available on the system."""
        try:
            # Try to import ROS2 Python libraries
            import rclpy
            return True
        except ImportError:
            try:
                # Try to run ros2 command
                result = subprocess.run(['ros2', '--version'], 
                                      capture_output=True, text=True, timeout=5)
                return result.returncode == 0
            except (subprocess.TimeoutExpired, FileNotFoundError):
                return False
    
    def get_data(self) -> RosGraphData:
        """Get current ROS graph data."""
        return RosGraphData(
            nodes=list(self.nodes.values()),
            topics=list(self.topics.values()),
            connections=self.connections.copy()
        )
    
    def is_available(self) -> bool:
        """Check if the ROS data source is available."""
        return self.ros2_available
    
    def get_ros_nodes(self) -> List[RosNode]:
        """
        Get list of active ROS nodes.
        Discovers nodes using ROS2 command line tools or Python API.
        """
        current_time = time.time()
        
        # Only discover if enough time has passed
        if current_time - self.last_discovery_time < self.discovery_interval:
            return list(self.nodes.values())
        
        try:
            self._discover_nodes()
            self.last_discovery_time = current_time
        except Exception as e:
            logger.error(f"Error discovering ROS nodes: {e}")
        
        return list(self.nodes.values())
    
    def get_ros_topics(self) -> List[RosTopic]:
        """
        Get list of active ROS topics.
        Discovers topics and their publishers/subscribers.
        """
        try:
            self._discover_topics()
        except Exception as e:
            logger.error(f"Error discovering ROS topics: {e}")
        
        return list(self.topics.values())
    
    def get_ros_connections(self) -> List[RosConnection]:
        """
        Get ROS node connections.
        Maps relationships between nodes through topics.
        """
        try:
            self._build_connections()
        except Exception as e:
            logger.error(f"Error building ROS connections: {e}")
        
        return self.connections.copy()
    
    def _discover_nodes(self):
        """Discover active ROS nodes."""
        try:
            # Use ros2 command line tool
            result = subprocess.run(['ros2', 'node', 'list'], 
                                  capture_output=True, text=True, timeout=10)
            
            if result.returncode == 0:
                node_names = [name.strip() for name in result.stdout.split('\n') if name.strip()]
                
                # Update nodes dictionary
                current_nodes = {}
                for node_name in node_names:
                    # Extract namespace and node name
                    if '/' in node_name and node_name != '/':
                        parts = node_name.split('/')
                        if len(parts) >= 2:
                            namespace = '/'.join(parts[:-1]) if len(parts) > 2 else '/'
                            name = parts[-1]
                        else:
                            namespace = '/'
                            name = node_name
                    else:
                        namespace = '/'
                        name = node_name.lstrip('/')
                    
                    node_id = f"node_{hash(node_name) % 10000}"
                    current_nodes[node_name] = RosNode(
                        id=node_id,
                        name=node_name,
                        namespace=namespace if namespace != '/' else None
                    )
                
                self.nodes = current_nodes
                logger.debug(f"Discovered {len(self.nodes)} ROS nodes")
            
        except subprocess.TimeoutExpired:
            logger.warning("ROS node discovery timed out")
        except FileNotFoundError:
            if self._should_log_ros_warning('ros2_not_found_nodes'):
                logger.warning("ros2 command not found - using fallback discovery")
            self._discover_nodes_fallback()
        except Exception as e:
            logger.error(f"Error in node discovery: {e}")
    
    def _discover_nodes_fallback(self):
        """Fallback node discovery using Python ROS2 API."""
        try:
            import rclpy
            from rclpy.node import Node
            
            # This is a simplified fallback - in practice, you'd need
            # to integrate with the existing ROS2 context
            logger.info("Using ROS2 Python API for node discovery")
            
        except ImportError:
            logger.warning("ROS2 Python API not available for fallback discovery")
    
    def _discover_topics(self):
        """Discover active ROS topics and their publishers/subscribers."""
        try:
            # Get topic list
            result = subprocess.run(['ros2', 'topic', 'list'], 
                                  capture_output=True, text=True, timeout=10)
            
            if result.returncode == 0:
                topic_names = [name.strip() for name in result.stdout.split('\n') if name.strip()]
                
                current_topics = {}
                for topic_name in topic_names:
                    # Get topic info (publishers and subscribers)
                    topic_info = self._get_topic_info(topic_name)
                    
                    if topic_info:
                        current_topics[topic_name] = RosTopic(
                            name=topic_name,
                            type=topic_info.get('type', 'unknown'),
                            publishers=topic_info.get('publishers', []),
                            subscribers=topic_info.get('subscribers', [])
                        )
                
                self.topics = current_topics
                logger.debug(f"Discovered {len(self.topics)} ROS topics")
            
        except subprocess.TimeoutExpired:
            logger.warning("ROS topic discovery timed out")
        except FileNotFoundError:
            if self._should_log_ros_warning('ros2_not_found_topics'):
                logger.warning("ros2 command not found for topic discovery")
        except Exception as e:
            logger.error(f"Error in topic discovery: {e}")
    
    def _get_topic_info(self, topic_name: str) -> Optional[Dict[str, Any]]:
        """
        Get detailed information about a specific topic.
        
        Args:
            topic_name: Name of the topic to query
            
        Returns:
            Dictionary with topic type, publishers, and subscribers
        """
        try:
            # Get topic type
            topic_type = self._get_topic_type(topic_name)
            
            # Get topic info (publishers and subscribers)
            result = subprocess.run(['ros2', 'topic', 'info', topic_name], 
                                  capture_output=True, text=True, timeout=5)
            
            if result.returncode == 0:
                info_text = result.stdout
                
                # Parse publishers and subscribers from output
                publishers = []
                subscribers = []
                
                lines = info_text.split('\n')
                current_section = None
                
                for line in lines:
                    line = line.strip()
                    if 'Publisher count:' in line:
                        current_section = 'publishers'
                    elif 'Subscription count:' in line:
                        current_section = 'subscribers'
                    elif line.startswith('*') and current_section:
                        # Extract node name from line like "* /node_name (rmw_fastrtps_cpp)"
                        if '(' in line:
                            node_name = line.split('(')[0].strip('* ').strip()
                            if current_section == 'publishers':
                                publishers.append(node_name)
                            elif current_section == 'subscribers':
                                subscribers.append(node_name)
                
                return {
                    'type': topic_type,
                    'publishers': publishers,
                    'subscribers': subscribers
                }
            
        except Exception as e:
            logger.debug(f"Error getting info for topic {topic_name}: {e}")
        
        return None
    
    def _get_topic_type(self, topic_name: str) -> str:
        """
        Get the message type for a topic.
        
        Args:
            topic_name: Name of the topic
            
        Returns:
            Topic message type string
        """
        # Check cache first
        if topic_name in self.topic_type_cache:
            return self.topic_type_cache[topic_name]
        
        try:
            result = subprocess.run(['ros2', 'topic', 'type', topic_name], 
                                  capture_output=True, text=True, timeout=5)
            
            if result.returncode == 0:
                topic_type = result.stdout.strip()
                self.topic_type_cache[topic_name] = topic_type
                return topic_type
            
        except Exception as e:
            logger.debug(f"Error getting type for topic {topic_name}: {e}")
        
        return 'unknown'

    def _build_connections(self):
        """Build connection relationships between nodes through topics."""
        connections = []
        
        try:
            for topic_name, topic in self.topics.items():
                # Create connections from publishers to subscribers through this topic
                for publisher in topic.publishers:
                    for subscriber in topic.subscribers:
                        if publisher != subscriber:  # Avoid self-connections
                            connection = RosConnection(
                                from_node=publisher,
                                to_node=subscriber,
                                topic=topic_name
                            )
                            connections.append(connection)
            
            self.connections = connections
            logger.debug(f"Built {len(self.connections)} ROS connections")
            
        except Exception as e:
            logger.error(f"Error building ROS connections: {e}")
    
    def get_graph_summary(self) -> Dict[str, Any]:
        """
        Get summary statistics of the ROS graph.
        
        Returns:
            Dictionary with graph statistics and health information
        """
        try:
            # Update graph data
            self.get_ros_nodes()
            self.get_ros_topics()
            self.get_ros_connections()
            
            # Calculate statistics
            node_count = len(self.nodes)
            topic_count = len(self.topics)
            connection_count = len(self.connections)
            
            # Analyze topic activity
            active_topics = sum(1 for topic in self.topics.values() 
                              if topic.publishers or topic.subscribers)
            
            # Analyze node connectivity
            connected_nodes = set()
            for connection in self.connections:
                connected_nodes.add(connection.from_node)
                connected_nodes.add(connection.to_node)
            
            isolated_nodes = node_count - len(connected_nodes)
            
            # Topic type distribution
            topic_types = defaultdict(int)
            for topic in self.topics.values():
                topic_types[topic.type] += 1
            
            return {
                'node_count': node_count,
                'topic_count': topic_count,
                'connection_count': connection_count,
                'active_topics': active_topics,
                'isolated_nodes': isolated_nodes,
                'connected_nodes': len(connected_nodes),
                'topic_types': dict(topic_types),
                'discovery_time': self.last_discovery_time,
                'timestamp': time.time()
            }
            
        except Exception as e:
            logger.error(f"Error generating graph summary: {e}")
            return {}
    
    def detect_graph_changes(self) -> bool:
        """
        Detect if the ROS graph has changed since last check.
        
        Returns:
            True if graph has changed, False otherwise
        """
        try:
            # Create a hash of the current graph state
            graph_data = {
                'nodes': sorted([node.name for node in self.nodes.values()]),
                'topics': sorted([f"{topic.name}:{topic.type}" for topic in self.topics.values()]),
                'connections': sorted([f"{conn.from_node}->{conn.to_node}:{conn.topic}" 
                                     for conn in self.connections])
            }
            
            current_hash = str(hash(str(graph_data)))
            
            if current_hash != self.graph_hash:
                self.graph_hash = current_hash
                return True
            
            return False
            
        except Exception as e:
            logger.error(f"Error detecting graph changes: {e}")
            return False
    
    def get_node_details(self, node_name: str) -> Optional[Dict[str, Any]]:
        """
        Get detailed information about a specific node.
        
        Args:
            node_name: Name of the node to query
            
        Returns:
            Dictionary with node details or None if not found
        """
        if node_name not in self.nodes:
            return None
        
        try:
            node = self.nodes[node_name]
            
            # Find topics this node publishes to
            published_topics = []
            subscribed_topics = []
            
            for topic_name, topic in self.topics.items():
                if node_name in topic.publishers:
                    published_topics.append({
                        'name': topic_name,
                        'type': topic.type,
                        'subscriber_count': len(topic.subscribers)
                    })
                
                if node_name in topic.subscribers:
                    subscribed_topics.append({
                        'name': topic_name,
                        'type': topic.type,
                        'publisher_count': len(topic.publishers)
                    })
            
            # Find connections involving this node
            outgoing_connections = [conn for conn in self.connections if conn.from_node == node_name]
            incoming_connections = [conn for conn in self.connections if conn.to_node == node_name]
            
            return {
                'node_info': node.to_dict(),
                'published_topics': published_topics,
                'subscribed_topics': subscribed_topics,
                'outgoing_connections': len(outgoing_connections),
                'incoming_connections': len(incoming_connections),
                'total_connections': len(outgoing_connections) + len(incoming_connections),
                'timestamp': time.time()
            }
            
        except Exception as e:
            logger.error(f"Error getting node details for {node_name}: {e}")
            return None
    
    def get_topic_details(self, topic_name: str) -> Optional[Dict[str, Any]]:
        """
        Get detailed information about a specific topic.
        
        Args:
            topic_name: Name of the topic to query
            
        Returns:
            Dictionary with topic details or None if not found
        """
        if topic_name not in self.topics:
            return None
        
        try:
            topic = self.topics[topic_name]
            
            # Get message rate if available
            message_rate = self._get_topic_message_rate(topic_name)
            
            # Find connections through this topic
            topic_connections = [conn for conn in self.connections if conn.topic == topic_name]
            
            return {
                'topic_info': topic.to_dict(),
                'message_rate': message_rate,
                'connection_count': len(topic_connections),
                'connections': [conn.to_dict() for conn in topic_connections],
                'timestamp': time.time()
            }
            
        except Exception as e:
            logger.error(f"Error getting topic details for {topic_name}: {e}")
            return None
    
    def _get_topic_message_rate(self, topic_name: str) -> Optional[float]:
        """
        Get the message publishing rate for a topic.
        
        Args:
            topic_name: Name of the topic
            
        Returns:
            Message rate in Hz or None if unavailable
        """
        try:
            # Use ros2 topic hz command to get message rate
            result = subprocess.run(['ros2', 'topic', 'hz', topic_name], 
                                  capture_output=True, text=True, timeout=3)
            
            if result.returncode == 0:
                # Parse rate from output (format: "average rate: X.XX")
                for line in result.stdout.split('\n'):
                    if 'average rate:' in line:
                        rate_str = line.split('average rate:')[1].strip()
                        return float(rate_str)
            
        except (subprocess.TimeoutExpired, ValueError, FileNotFoundError):
            pass
        except Exception as e:
            logger.debug(f"Error getting message rate for {topic_name}: {e}")
        
        return None
    
    def export_graph_data(self, format: str = 'json') -> str:
        """
        Export ROS graph data in specified format.
        
        Args:
            format: Export format ('json', 'dot', 'yaml')
            
        Returns:
            Formatted graph data as string
        """
        try:
            graph_data = self.get_data()
            
            if format.lower() == 'json':
                return json.dumps(graph_data.to_dict(), indent=2)
            
            elif format.lower() == 'dot':
                return self._export_to_dot_format(graph_data)
            
            elif format.lower() == 'yaml':
                import yaml
                return yaml.dump(graph_data.to_dict(), default_flow_style=False)
            
            else:
                raise ValueError(f"Unsupported export format: {format}")
            
        except Exception as e:
            logger.error(f"Error exporting graph data: {e}")
            return ""
    
    def _export_to_dot_format(self, graph_data: RosGraphData) -> str:
        """
        Export graph data to DOT format for visualization.
        
        Args:
            graph_data: ROS graph data to export
            
        Returns:
            DOT format string
        """
        dot_lines = ["digraph ROS_Graph {"]
        dot_lines.append("  rankdir=LR;")
        dot_lines.append("  node [shape=box];")
        
        # Add nodes
        for node in graph_data.nodes:
            node_label = node.name.replace('/', '_')
            dot_lines.append(f'  "{node_label}" [label="{node.name}"];')
        
        # Add topic nodes
        for topic in graph_data.topics:
            topic_label = topic.name.replace('/', '_')
            dot_lines.append(f'  "{topic_label}" [label="{topic.name}\\n({topic.type})" shape=ellipse color=blue];')
        
        # Add connections
        for connection in graph_data.connections:
            from_label = connection.from_node.replace('/', '_')
            to_label = connection.to_node.replace('/', '_')
            topic_label = connection.topic.replace('/', '_')
            
            # Node -> Topic -> Node
            dot_lines.append(f'  "{from_label}" -> "{topic_label}";')
            dot_lines.append(f'  "{topic_label}" -> "{to_label}";')
        
        dot_lines.append("}")
        return '\n'.join(dot_lines)
    
    def clear_cache(self):
        """Clear all cached data and force rediscovery."""
        self.nodes.clear()
        self.topics.clear()
        self.connections.clear()
        self.topic_type_cache.clear()
        self.graph_hash = ""
        self.last_discovery_time = 0.0
        
        logger.info("ROS graph cache cleared")
    
    def set_discovery_interval(self, interval: float):
        """
        Set the discovery interval for ROS graph updates.
        
        Args:
            interval: Discovery interval in seconds
        """
        self.discovery_interval = max(0.5, interval)  # Minimum 0.5 seconds
        logger.info(f"ROS discovery interval set to {self.discovery_interval}s")


class ROSGraphMonitor:
    """
    Standalone ROS graph monitor that can run in a separate thread.
    Provides continuous monitoring and change detection.
    """
    
    def __init__(self, data_source: ROS2GraphDataSource, update_callback=None):
        """
        Initialize the ROS graph monitor.
        
        Args:
            data_source: ROS2GraphDataSource instance
            update_callback: Callback function for graph updates
        """
        self.data_source = data_source
        self.update_callback = update_callback
        
        self.monitoring = False
        self.monitor_thread: Optional[threading.Thread] = None
        self.monitor_interval = 1.0  # Check for changes every second
        
    def start_monitoring(self):
        """Start continuous ROS graph monitoring."""
        if self.monitoring:
            logger.warning("ROS graph monitoring is already running")
            return
        
        self.monitoring = True
        self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.monitor_thread.start()
        
        logger.info("ROS graph monitoring started")
    
    def stop_monitoring(self):
        """Stop ROS graph monitoring."""
        self.monitoring = False
        
        if self.monitor_thread and self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=5)
        
        logger.info("ROS graph monitoring stopped")
    
    def _monitor_loop(self):
        """Main monitoring loop."""
        while self.monitoring:
            try:
                # Check for graph changes
                if self.data_source.detect_graph_changes():
                    logger.info("ROS graph changes detected")
                    
                    # Get updated graph data
                    graph_data = self.data_source.get_data()
                    
                    # Call update callback if provided
                    if self.update_callback:
                        self.update_callback(graph_data)
                
                time.sleep(self.monitor_interval)
                
            except Exception as e:
                logger.error(f"Error in ROS graph monitoring loop: {e}")
                time.sleep(self.monitor_interval)
    
    def set_monitor_interval(self, interval: float):
        """Set the monitoring interval."""
        self.monitor_interval = max(0.1, interval)
        logger.info(f"ROS graph monitor interval set to {self.monitor_interval}s")