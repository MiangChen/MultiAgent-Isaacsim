"""
Startup configuration and command line argument management for WebManager.
Provides enhanced configuration options and validation.
"""

import argparse
import logging
import os
import sys
from pathlib import Path
from typing import Dict, Any, Optional, List
import yaml
import json


class WebManagerConfig:
    """Configuration class for WebManager startup options."""
    
    def __init__(self):
        self.config_file: Optional[str] = None
        self.web_host: str = "0.0.0.0"
        self.web_port: int = 8080
        self.log_level: str = "INFO"
        self.enable_webmanager: bool = True
        self.enable_ros: bool = True
        self.data_collection_rate: float = 10.0
        self.max_history: int = 1000
        self.disable_camera_streaming: bool = False
        self.camera_quality: int = 80
        self.enable_compression: bool = False
        self.log_file: str = "isaac_sim_webmanager.log"
        self.pid_file: Optional[str] = None
        self.status_file: Optional[str] = None
        self.enable_metrics: bool = True
        self.metrics_port: Optional[int] = None
        self.enable_health_check: bool = True
        self.health_check_interval: float = 30.0
        self.shutdown_timeout: float = 10.0
        self.enable_auto_restart: bool = False
        self.max_restart_attempts: int = 3
        
    def to_dict(self) -> Dict[str, Any]:
        """Convert configuration to dictionary."""
        return {
            'config_file': self.config_file,
            'web_host': self.web_host,
            'web_port': self.web_port,
            'log_level': self.log_level,
            'enable_webmanager': self.enable_webmanager,
            'enable_ros': self.enable_ros,
            'data_collection_rate': self.data_collection_rate,
            'max_history': self.max_history,
            'disable_camera_streaming': self.disable_camera_streaming,
            'camera_quality': self.camera_quality,
            'enable_compression': self.enable_compression,
            'log_file': self.log_file,
            'pid_file': self.pid_file,
            'status_file': self.status_file,
            'enable_metrics': self.enable_metrics,
            'metrics_port': self.metrics_port,
            'enable_health_check': self.enable_health_check,
            'health_check_interval': self.health_check_interval,
            'shutdown_timeout': self.shutdown_timeout,
            'enable_auto_restart': self.enable_auto_restart,
            'max_restart_attempts': self.max_restart_attempts
        }
    
    def from_dict(self, config_dict: Dict[str, Any]) -> None:
        """Load configuration from dictionary."""
        for key, value in config_dict.items():
            if hasattr(self, key):
                setattr(self, key, value)
    
    def from_args(self, args) -> None:
        """Load configuration from parsed arguments object."""
        # Map argument names to config attributes
        arg_mapping = {
            'config': 'config_file',
            'web_host': 'web_host',
            'web_port': 'web_port',
            'log_level': 'log_level',
            'ros': 'enable_ros',
            'data_collection_rate': 'data_collection_rate',
            'max_history': 'max_history',
            'disable_camera_streaming': 'disable_camera_streaming',
            'camera_quality': 'camera_quality',
            'enable_compression': 'enable_compression',
            'log_file': 'log_file',
            'pid_file': 'pid_file',
            'status_file': 'status_file',
            'enable_metrics': 'enable_metrics',
            'metrics_port': 'metrics_port',
            'enable_health_check': 'enable_health_check',
            'health_check_interval': 'health_check_interval',
            'shutdown_timeout': 'shutdown_timeout',
            'enable_auto_restart': 'enable_auto_restart',
            'max_restart_attempts': 'max_restart_attempts'
        }
        
        # Set attributes from args
        for arg_name, config_attr in arg_mapping.items():
            if hasattr(args, arg_name):
                setattr(self, config_attr, getattr(args, arg_name))
        
        # Handle special cases
        if hasattr(args, 'enable_webmanager') and hasattr(args, 'disable_webmanager'):
            self.enable_webmanager = args.enable_webmanager and not args.disable_webmanager
    
    @classmethod
    def from_args_direct(cls, args):
        """Create a WebManagerConfig instance directly from parsed arguments."""
        config = cls()
        config.from_args(args)
        return config
    
    def load_from_file(self, config_file: str) -> None:
        """Load configuration from YAML or JSON file."""
        config_path = Path(config_file)
        
        if not config_path.exists():
            raise FileNotFoundError(f"Configuration file not found: {config_file}")
        
        with open(config_path, 'r') as f:
            if config_path.suffix.lower() in ['.yaml', '.yml']:
                config_data = yaml.safe_load(f)
            elif config_path.suffix.lower() == '.json':
                config_data = json.load(f)
            else:
                raise ValueError(f"Unsupported configuration file format: {config_path.suffix}")
        
        # Extract webmanager section if it exists
        if 'webmanager' in config_data:
            config_data = config_data['webmanager']
        
        self.from_dict(config_data)
        self.config_file = config_file
    
    def save_to_file(self, config_file: str) -> None:
        """Save configuration to YAML file."""
        config_path = Path(config_file)
        config_path.parent.mkdir(parents=True, exist_ok=True)
        
        with open(config_path, 'w') as f:
            yaml.dump({'webmanager': self.to_dict()}, f, default_flow_style=False)
    
    def validate(self) -> List[str]:
        """Validate configuration and return list of errors."""
        errors = []
        
        # Validate port ranges
        if not (1 <= self.web_port <= 65535):
            errors.append(f"Invalid web port: {self.web_port} (must be 1-65535)")
        
        if self.metrics_port and not (1 <= self.metrics_port <= 65535):
            errors.append(f"Invalid metrics port: {self.metrics_port} (must be 1-65535)")
        
        # Validate data collection rate
        if not (0.1 <= self.data_collection_rate <= 100.0):
            errors.append(f"Invalid data collection rate: {self.data_collection_rate} (must be 0.1-100.0 Hz)")
        
        # Validate max history
        if not (10 <= self.max_history <= 100000):
            errors.append(f"Invalid max history: {self.max_history} (must be 10-100000)")
        
        # Validate camera quality
        if not (10 <= self.camera_quality <= 100):
            errors.append(f"Invalid camera quality: {self.camera_quality} (must be 10-100)")
        
        # Validate log level
        valid_log_levels = ['DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL']
        if self.log_level.upper() not in valid_log_levels:
            errors.append(f"Invalid log level: {self.log_level} (must be one of {valid_log_levels})")
        
        # Validate timeouts
        if not (1.0 <= self.shutdown_timeout <= 300.0):
            errors.append(f"Invalid shutdown timeout: {self.shutdown_timeout} (must be 1.0-300.0 seconds)")
        
        if not (1.0 <= self.health_check_interval <= 3600.0):
            errors.append(f"Invalid health check interval: {self.health_check_interval} (must be 1.0-3600.0 seconds)")
        
        # Validate restart attempts
        if not (0 <= self.max_restart_attempts <= 10):
            errors.append(f"Invalid max restart attempts: {self.max_restart_attempts} (must be 0-10)")
        
        return errors


def create_argument_parser() -> argparse.ArgumentParser:
    """Create enhanced argument parser for WebManager startup."""
    
    parser = argparse.ArgumentParser(
        description="Isaac Sim WebManager - Web-based monitoring and control interface",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Start with default settings
  python main.py
  
  # Start on specific host and port
  python main.py --web-host 192.168.1.100 --web-port 8081
  
  # Start without camera streaming
  python main.py --disable-camera-streaming
  
  # Start with debug logging
  python main.py --log-level DEBUG
  
  # Start without WebManager (simulation only)
  python main.py --disable-webmanager
  
  # Load configuration from file
  python main.py --webmanager-config webmanager_config.yaml
  
  # Enable process monitoring
  python main.py --pid-file /tmp/webmanager.pid --status-file /tmp/webmanager.status
        """
    )
    
    # Basic Isaac Sim configuration
    parser.add_argument(
        "--config",
        type=str,
        default="./files/sim_cfg.yaml",
        help="Path to Isaac Sim configuration file (default: ./files/sim_cfg.yaml)"
    )
    
    parser.add_argument(
        "--enable",
        type=str,
        action="append",
        help="Enable a feature. Can be used multiple times."
    )
    
    parser.add_argument(
        "--ros",
        type=bool,
        default=True,
        help="Enable ROS integration (default: True)"
    )
    
    # WebManager configuration
    webmanager_group = parser.add_argument_group('WebManager Configuration')
    
    webmanager_group.add_argument(
        "--webmanager-config",
        type=str,
        help="Path to WebManager configuration file (YAML or JSON)"
    )
    
    webmanager_group.add_argument(
        "--enable-webmanager",
        action="store_true",
        default=True,
        help="Enable WebManager web interface (default: True)"
    )
    
    webmanager_group.add_argument(
        "--disable-webmanager",
        action="store_true",
        help="Disable WebManager web interface"
    )
    
    webmanager_group.add_argument(
        "--web-host",
        type=str,
        default="0.0.0.0",
        help="WebManager server host address (default: 0.0.0.0)"
    )
    
    webmanager_group.add_argument(
        "--web-port",
        type=int,
        default=8080,
        help="WebManager server port (default: 8080)"
    )
    
    # Data collection settings
    data_group = parser.add_argument_group('Data Collection Settings')
    
    data_group.add_argument(
        "--data-collection-rate",
        type=float,
        default=10.0,
        help="Data collection frequency in Hz (default: 10.0)"
    )
    
    data_group.add_argument(
        "--max-history",
        type=int,
        default=1000,
        help="Maximum number of historical data points to store (default: 1000)"
    )
    
    # Camera settings
    camera_group = parser.add_argument_group('Camera Settings')
    
    camera_group.add_argument(
        "--disable-camera-streaming",
        action="store_true",
        help="Disable camera frame streaming to reduce bandwidth"
    )
    
    camera_group.add_argument(
        "--camera-quality",
        type=int,
        default=80,
        choices=range(10, 101),
        metavar="[10-100]",
        help="Camera frame JPEG quality (10-100, default: 80)"
    )
    
    # Logging and monitoring
    logging_group = parser.add_argument_group('Logging and Monitoring')
    
    logging_group.add_argument(
        "--log-level",
        type=str,
        choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
        default="INFO",
        help="Logging level (default: INFO)"
    )
    
    logging_group.add_argument(
        "--log-file",
        type=str,
        default="isaac_sim_webmanager.log",
        help="Log file path (default: isaac_sim_webmanager.log)"
    )
    
    logging_group.add_argument(
        "--pid-file",
        type=str,
        help="Write process ID to file for monitoring"
    )
    
    logging_group.add_argument(
        "--status-file",
        type=str,
        help="Write status information to file for monitoring"
    )
    
    # Performance and optimization
    perf_group = parser.add_argument_group('Performance and Optimization')
    
    perf_group.add_argument(
        "--enable-compression",
        action="store_true",
        help="Enable data compression for WebSocket messages"
    )
    
    perf_group.add_argument(
        "--enable-metrics",
        action="store_true",
        default=True,
        help="Enable performance metrics collection (default: True)"
    )
    
    perf_group.add_argument(
        "--metrics-port",
        type=int,
        help="Port for metrics endpoint (optional)"
    )
    
    # Health and reliability
    health_group = parser.add_argument_group('Health and Reliability')
    
    health_group.add_argument(
        "--enable-health-check",
        action="store_true",
        default=True,
        help="Enable health check monitoring (default: True)"
    )
    
    health_group.add_argument(
        "--health-check-interval",
        type=float,
        default=30.0,
        help="Health check interval in seconds (default: 30.0)"
    )
    
    health_group.add_argument(
        "--shutdown-timeout",
        type=float,
        default=10.0,
        help="Graceful shutdown timeout in seconds (default: 10.0)"
    )
    
    health_group.add_argument(
        "--enable-auto-restart",
        action="store_true",
        help="Enable automatic restart on failure"
    )
    
    health_group.add_argument(
        "--max-restart-attempts",
        type=int,
        default=3,
        help="Maximum automatic restart attempts (default: 3)"
    )
    
    # Utility options
    util_group = parser.add_argument_group('Utility Options')
    
    util_group.add_argument(
        "--check-config",
        action="store_true",
        help="Validate configuration and exit"
    )
    
    util_group.add_argument(
        "--print-config",
        action="store_true",
        help="Print current configuration and exit"
    )
    
    util_group.add_argument(
        "--save-config",
        type=str,
        help="Save current configuration to file and exit"
    )
    
    util_group.add_argument(
        "--dry-run",
        action="store_true",
        help="Validate configuration without starting services"
    )
    
    return parser


def parse_arguments(args: Optional[List[str]] = None) -> WebManagerConfig:
    """Parse command line arguments and return configuration object."""
    
    parser = create_argument_parser()
    parsed_args = parser.parse_args(args)
    
    # Create configuration object
    config = WebManagerConfig()
    
    # Load from configuration file if specified
    if parsed_args.webmanager_config:
        try:
            config.load_from_file(parsed_args.webmanager_config)
        except Exception as e:
            print(f"Error loading configuration file: {e}", file=sys.stderr)
            sys.exit(1)
    
    # Override with command line arguments
    config.config_file = parsed_args.config
    config.web_host = parsed_args.web_host
    config.web_port = parsed_args.web_port
    config.log_level = parsed_args.log_level
    config.enable_webmanager = parsed_args.enable_webmanager and not parsed_args.disable_webmanager
    config.enable_ros = parsed_args.ros
    config.data_collection_rate = parsed_args.data_collection_rate
    config.max_history = parsed_args.max_history
    config.disable_camera_streaming = parsed_args.disable_camera_streaming
    config.camera_quality = parsed_args.camera_quality
    config.enable_compression = parsed_args.enable_compression
    config.log_file = parsed_args.log_file
    config.pid_file = parsed_args.pid_file
    config.status_file = parsed_args.status_file
    config.enable_metrics = parsed_args.enable_metrics
    config.metrics_port = parsed_args.metrics_port
    config.enable_health_check = parsed_args.enable_health_check
    config.health_check_interval = parsed_args.health_check_interval
    config.shutdown_timeout = parsed_args.shutdown_timeout
    config.enable_auto_restart = parsed_args.enable_auto_restart
    config.max_restart_attempts = parsed_args.max_restart_attempts
    
    # Handle utility options
    if parsed_args.check_config:
        errors = config.validate()
        if errors:
            print("Configuration validation errors:", file=sys.stderr)
            for error in errors:
                print(f"  - {error}", file=sys.stderr)
            sys.exit(1)
        else:
            print("Configuration is valid")
            sys.exit(0)
    
    if parsed_args.print_config:
        print("Current configuration:")
        print(yaml.dump(config.to_dict(), default_flow_style=False))
        sys.exit(0)
    
    if parsed_args.save_config:
        try:
            config.save_to_file(parsed_args.save_config)
            print(f"Configuration saved to: {parsed_args.save_config}")
            sys.exit(0)
        except Exception as e:
            print(f"Error saving configuration: {e}", file=sys.stderr)
            sys.exit(1)
    
    if parsed_args.dry_run:
        errors = config.validate()
        if errors:
            print("Configuration validation errors:", file=sys.stderr)
            for error in errors:
                print(f"  - {error}", file=sys.stderr)
            sys.exit(1)
        else:
            print("Configuration is valid - dry run successful")
            sys.exit(0)
    
    # Validate configuration
    errors = config.validate()
    if errors:
        print("Configuration validation errors:", file=sys.stderr)
        for error in errors:
            print(f"  - {error}", file=sys.stderr)
        sys.exit(1)
    
    return config


def setup_enhanced_logging(config: WebManagerConfig) -> logging.Logger:
    """Setup enhanced logging with rotation and structured output."""
    
    # Create logs directory if it doesn't exist
    log_path = Path(config.log_file)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    
    # Configure logging
    log_level = getattr(logging, config.log_level.upper())
    
    # Create formatters
    console_formatter = logging.Formatter(
        "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
    )
    
    file_formatter = logging.Formatter(
        "%(asctime)s - %(name)s - %(levelname)s - %(filename)s:%(lineno)d - %(message)s"
    )
    
    # Create handlers
    console_handler = logging.StreamHandler()
    console_handler.setLevel(log_level)
    console_handler.setFormatter(console_formatter)
    
    file_handler = logging.FileHandler(config.log_file, mode='a')
    file_handler.setLevel(log_level)
    file_handler.setFormatter(file_formatter)
    
    # Configure root logger
    root_logger = logging.getLogger()
    root_logger.setLevel(log_level)
    root_logger.handlers.clear()  # Remove any existing handlers
    root_logger.addHandler(console_handler)
    root_logger.addHandler(file_handler)
    
    # Create main logger
    logger = logging.getLogger("webmanager.startup")
    
    # Log startup information
    logger.info(f"Isaac Sim WebManager starting with log level: {config.log_level}")
    logger.info(f"WebManager enabled: {config.enable_webmanager}")
    if config.enable_webmanager:
        logger.info(f"WebManager will run on {config.web_host}:{config.web_port}")
    logger.info(f"Configuration: {config.to_dict()}")
    
    return logger