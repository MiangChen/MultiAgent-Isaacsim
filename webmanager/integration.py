"""
Integration module for setting up the complete WebManager data collection system.
Provides easy setup and configuration for all data collectors and web server.
"""

import logging
from typing import Optional, Dict, Any

from .web_server import WebUIServer
from .data_collector import DataCollector
from .robot_data_collector import SwarmManagerRobotDataSource
from .performance_data_collector import IsaacSimPerformanceDataSource
from .ros_data_collector import ROS2GraphDataSource
from .camera_data_collector import IsaacSimCameraDataSource, CameraStreamManager

logger = logging.getLogger(__name__)


class WebManagerSystem:
    """
    Complete WebManager system integration.
    Sets up and manages all components for web-based Isaac Sim monitoring.
    """
    
    def __init__(self, 
                 host: str = "0.0.0.0", 
                 port: int = 8080,
                 collection_rate: float = 10.0,
                 max_history: int = 1000):
        """
        Initialize the WebManager system.
        
        Args:
            host: Web server host address
            port: Web server port
            collection_rate: Data collection frequency in Hz
            max_history: Maximum historical data points to store
        """
        self.host = host
        self.port = port
        self.collection_rate = collection_rate
        self.max_history = max_history
        
        # Core components
        self.web_server: Optional[WebUIServer] = None
        self.data_collector: Optional[DataCollector] = None
        
        # Data sources
        self.robot_data_source: Optional[SwarmManagerRobotDataSource] = None
        self.performance_data_source: Optional[IsaacSimPerformanceDataSource] = None
        self.ros_data_source: Optional[ROS2GraphDataSource] = None
        self.camera_data_source: Optional[IsaacSimCameraDataSource] = None
        
        # Utilities
        self.camera_stream_manager: Optional[CameraStreamManager] = None
        
        # System state
        self.initialized = False
        self.running = False
        
        logger.info(f"WebManagerSystem created (host={host}, port={port})")
    
    def initialize(self, 
                   swarm_manager=None,
                   isaac_sim_app=None,
                   viewport_manager=None,
                   ros_monitor=None,
                   config=None):
        """
        Initialize all WebManager components.
        
        Args:
            swarm_manager: Reference to robot swarm manager
            isaac_sim_app: Reference to Isaac Sim application
            viewport_manager: Reference to viewport manager
            ros_monitor: Reference to ROS monitoring system
            config: Additional configuration parameters (optional)
        """
        try:
            logger.info("Initializing WebManager system...")
            
            # Initialize web server
            self.web_server = WebUIServer(host=self.host, port=self.port)
            
            # Initialize data collector
            self.data_collector = DataCollector(
                web_server=self.web_server,
                max_history=self.max_history,
                collection_rate=self.collection_rate
            )
            
            # Initialize data sources
            self._initialize_data_sources(
                swarm_manager=swarm_manager,
                isaac_sim_app=isaac_sim_app,
                viewport_manager=viewport_manager,
                ros_monitor=ros_monitor
            )
            
            # Register data sources with collector
            self._register_data_sources()
            
            # Connect web server to data collector
            self.web_server.set_data_collector(self.data_collector)
            
            # Initialize camera stream manager
            if self.camera_data_source:
                self.camera_stream_manager = CameraStreamManager(
                    camera_source=self.camera_data_source,
                    web_server=self.web_server
                )
            
            self.initialized = True
            logger.info("WebManager system initialized successfully")
            
        except Exception as e:
            logger.error(f"Error initializing WebManager system: {e}")
            raise
    
    def _initialize_data_sources(self, **kwargs):
        """Initialize all data sources with provided references."""
        
        # Robot data source
        swarm_manager = kwargs.get('swarm_manager')
        if swarm_manager:
            self.robot_data_source = SwarmManagerRobotDataSource(swarm_manager=swarm_manager)
            logger.info("Robot data source initialized with swarm manager")
        else:
            # Use default data source for testing
            from .data_collector import DefaultRobotDataSource
            self.robot_data_source = DefaultRobotDataSource()
            logger.info("Robot data source initialized with default implementation")
        
        # Performance data source
        isaac_sim_app = kwargs.get('isaac_sim_app')
        ros_monitor = kwargs.get('ros_monitor')
        self.performance_data_source = IsaacSimPerformanceDataSource(
            isaac_sim_app=isaac_sim_app,
            ros_monitor=ros_monitor
        )
        logger.info("Performance data source initialized")
        
        # ROS data source
        self.ros_data_source = ROS2GraphDataSource()
        logger.info("ROS data source initialized")
        
        # Camera data source
        viewport_manager = kwargs.get('viewport_manager')
        self.camera_data_source = IsaacSimCameraDataSource(
            isaac_sim_app=isaac_sim_app,
            viewport_manager=viewport_manager
        )
        logger.info("Camera data source initialized")
    
    def _register_data_sources(self):
        """Register all data sources with the data collector."""
        if self.robot_data_source:
            self.data_collector.register_robot_data_source(self.robot_data_source)
        
        if self.performance_data_source:
            self.data_collector.register_performance_data_source(self.performance_data_source)
        
        if self.ros_data_source:
            self.data_collector.register_ros_data_source(self.ros_data_source)
        
        if self.camera_data_source:
            self.data_collector.register_camera_data_source(self.camera_data_source)
        
        logger.info("All data sources registered with data collector")
    
    def start(self):
        """Start the WebManager system."""
        if not self.initialized:
            raise RuntimeError("WebManager system not initialized. Call initialize() first.")
        
        if self.running:
            logger.warning("WebManager system is already running")
            return
        
        try:
            logger.info("Starting WebManager system...")
            
            # Start web server
            self.web_server.start_server()
            
            # Start data collection
            self.data_collector.start_collection()
            
            self.running = True
            logger.info(f"WebManager system started successfully on http://{self.host}:{self.port}")
            
        except Exception as e:
            logger.error(f"Error starting WebManager system: {e}")
            raise
    
    def stop(self):
        """Stop the WebManager system."""
        if not self.running:
            logger.warning("WebManager system is not running")
            return
        
        try:
            logger.info("Stopping WebManager system...")
            
            # Stop camera streaming
            if self.camera_stream_manager:
                self.camera_stream_manager.stop_all_streams()
            
            # Stop data collection
            if self.data_collector:
                self.data_collector.stop_collection()
            
            # Stop web server
            if self.web_server:
                self.web_server.stop_server()
            
            self.running = False
            logger.info("WebManager system stopped")
            
        except Exception as e:
            logger.error(f"Error stopping WebManager system: {e}")
    
    def get_system_status(self) -> Dict[str, Any]:
        """Get comprehensive system status."""
        status = {
            'initialized': self.initialized,
            'running': self.running,
            'web_server': {
                'host': self.host,
                'port': self.port,
                'active_connections': 0
            },
            'data_collection': {
                'active': False,
                'collection_rate': self.collection_rate,
                'max_history': self.max_history
            },
            'data_sources': {
                'robot_data': False,
                'performance_data': False,
                'ros_data': False,
                'camera_data': False
            }
        }
        
        try:
            # Web server status
            if self.web_server:
                status['web_server']['active_connections'] = len(
                    self.web_server.websocket_manager.active_connections
                )
            
            # Data collection status
            if self.data_collector:
                status['data_collection']['active'] = self.data_collector.is_collecting()
                status['data_collection']['stats'] = self.data_collector.get_collection_stats()
                status['data_sources'] = self.data_collector.get_data_source_status()
            
            # Camera streaming status
            if self.camera_stream_manager:
                status['camera_streaming'] = self.camera_stream_manager.get_stream_status()
            
        except Exception as e:
            logger.error(f"Error getting system status: {e}")
        
        return status
    
    def start_camera_stream(self, camera_id: str = "default", **kwargs):
        """Start streaming from a camera."""
        if self.camera_stream_manager:
            self.camera_stream_manager.start_camera_stream(camera_id, **kwargs)
        else:
            logger.warning("Camera stream manager not available")
    
    def stop_camera_stream(self, camera_id: str):
        """Stop streaming from a camera."""
        if self.camera_stream_manager:
            self.camera_stream_manager.stop_camera_stream(camera_id)
        else:
            logger.warning("Camera stream manager not available")
    
    def add_robot(self, robot_id: str, initial_position: list, **kwargs):
        """Add a robot to tracking."""
        if self.robot_data_source and hasattr(self.robot_data_source, 'add_robot'):
            self.robot_data_source.add_robot(robot_id, initial_position, **kwargs)
        else:
            logger.warning("Robot data source does not support adding robots")
    
    def remove_robot(self, robot_id: str):
        """Remove a robot from tracking."""
        if self.robot_data_source and hasattr(self.robot_data_source, 'remove_robot'):
            self.robot_data_source.remove_robot(robot_id)
        else:
            logger.warning("Robot data source does not support removing robots")


def create_webmanager_system(**kwargs) -> WebManagerSystem:
    """
    Factory function to create and configure a WebManager system.
    
    Args:
        **kwargs: Configuration parameters for WebManagerSystem
        
    Returns:
        Configured WebManagerSystem instance
    """
    return WebManagerSystem(**kwargs)


def setup_webmanager_for_isaac_sim(isaac_sim_app, 
                                   swarm_manager=None,
                                   viewport_manager=None,
                                   **kwargs) -> WebManagerSystem:
    """
    Convenience function to set up WebManager for Isaac Sim integration.
    
    Args:
        isaac_sim_app: Isaac Sim application instance
        swarm_manager: Optional swarm manager instance
        viewport_manager: Optional viewport manager instance
        **kwargs: Additional configuration parameters
        
    Returns:
        Initialized and started WebManagerSystem
    """
    # Create system
    system = create_webmanager_system(**kwargs)
    
    # Initialize with Isaac Sim components
    system.initialize(
        isaac_sim_app=isaac_sim_app,
        swarm_manager=swarm_manager,
        viewport_manager=viewport_manager
    )
    
    # Start system
    system.start()
    
    logger.info("WebManager system set up for Isaac Sim")
    return system