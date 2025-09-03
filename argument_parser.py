#!/usr/bin/env python3
"""
Argument Parser for Isaac Sim WebManager

This module contains all command line argument parsing logic for the Isaac Sim WebManager system.
Separated from main.py to improve code organization and maintainability.
"""

import argparse
from typing import Any


def create_argument_parser() -> argparse.ArgumentParser:
    """
    Create and configure the argument parser for Isaac Sim WebManager.
    
    Returns:
        argparse.ArgumentParser: Configured argument parser
    """
    parser = argparse.ArgumentParser(
        description="Isaac Sim WebManager - Multi-robot simulation with web interface",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic usage with WebManager
  python main.py --enable-webmanager
  
  # Custom configuration
  python main.py --config ./custom_sim_cfg.yaml --enable-webmanager --web-port 9090
  
  # Performance optimized for Isaac Sim
  python main.py --enable-webmanager --data-collection-rate 5.0 --webmanager-low-impact
  
  # High performance WebManager (may impact Isaac Sim)
  python main.py --enable-webmanager --data-collection-rate 20.0 --webmanager-high-performance
        """
    )
    
    # Core simulation configuration
    _add_simulation_args(parser)
    
    # WebManager configuration
    _add_webmanager_args(parser)
    
    # Logging and monitoring
    _add_logging_args(parser)
    
    # Process management
    _add_process_management_args(parser)
    
    # Performance optimization
    _add_performance_args(parser)
    
    return parser


def _add_simulation_args(parser: argparse.ArgumentParser) -> None:
    """Add simulation-related arguments."""
    sim_group = parser.add_argument_group('Simulation Configuration')
    
    sim_group.add_argument(
        "--config",
        type=str,
        default="./config/sim_cfg.yaml",
        help="Path to the simulation configuration YAML file (default: ./config/sim_cfg.yaml)"
    )
    
    sim_group.add_argument(
        "--enable",
        type=str,
        action="append",
        help="Enable a feature. Can be used multiple times."
    )
    
    sim_group.add_argument(
        "--ros",
        type=bool,
        default=True,
        help="Enable ROS integration (default: True)"
    )


def _add_webmanager_args(parser: argparse.ArgumentParser) -> None:
    """Add WebManager-related arguments."""
    webmanager_group = parser.add_argument_group('WebManager Configuration')
    
    # Core WebManager settings
    webmanager_group.add_argument(
        "--enable-webmanager",
        action="store_true",
        default=True,
        help="Enable WebManager web interface"
    )
    
    webmanager_group.add_argument(
        "--disable-webmanager",
        action="store_true",
        help="Explicitly disable WebManager web interface"
    )
    
    # Network configuration
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
    webmanager_group.add_argument(
        "--data-collection-rate",
        type=float,
        default=5.0,
        help="Data collection frequency in Hz (default: 5.0)"
    )
    
    webmanager_group.add_argument(
        "--max-history",
        type=int,
        default=1000,
        help="Maximum number of historical data points to store (default: 1000)"
    )
    
    # Camera and streaming settings
    webmanager_group.add_argument(
        "--disable-camera-streaming",
        action="store_true",
        help="Disable camera frame streaming to reduce bandwidth"
    )
    
    webmanager_group.add_argument(
        "--camera-quality",
        type=int,
        default=80,
        choices=range(10, 101),
        metavar="[10-100]",
        help="Camera frame JPEG quality (10-100, default: 80)"
    )
    
    webmanager_group.add_argument(
        "--enable-compression",
        action="store_true",
        help="Enable data compression for WebSocket messages"
    )

#
def _add_logging_args(parser: argparse.ArgumentParser) -> None:
    """Add logging-related arguments."""
    logging_group = parser.add_argument_group('Logging Configuration')

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


def _add_process_management_args(parser: argparse.ArgumentParser) -> None:
    """Add process management and monitoring arguments."""
    process_group = parser.add_argument_group('Process Management')
    
    # Process identification
    process_group.add_argument(
        "--pid-file",
        type=str,
        help="Write process ID to file for monitoring"
    )
    
    process_group.add_argument(
        "--status-file",
        type=str,
        help="Write status information to file for monitoring"
    )
    
    # Metrics and monitoring
    process_group.add_argument(
        "--enable-metrics",
        action="store_true",
        default=True,
        help="Enable performance metrics collection (default: True)"
    )
    
    process_group.add_argument(
        "--metrics-port",
        type=int,
        help="Port for metrics endpoint (optional)"
    )
    
    # Health checking
    process_group.add_argument(
        "--enable-health-check",
        action="store_true",
        default=True,
        help="Enable health check monitoring (default: True)"
    )
    
    process_group.add_argument(
        "--health-check-interval",
        type=float,
        default=30.0,
        help="Health check interval in seconds (default: 30.0)"
    )
    
    # Shutdown and restart behavior
    process_group.add_argument(
        "--shutdown-timeout",
        type=float,
        default=10.0,
        help="Graceful shutdown timeout in seconds (default: 10.0)"
    )
    
    process_group.add_argument(
        "--enable-auto-restart",
        action="store_true",
        help="Enable automatic restart on failure"
    )
    
    process_group.add_argument(
        "--max-restart-attempts",
        type=int,
        default=3,
        help="Maximum automatic restart attempts (default: 3)"
    )


def _add_performance_args(parser: argparse.ArgumentParser) -> None:
    """Add performance optimization arguments."""
    performance_group = parser.add_argument_group('Performance Optimization')
    
    # WebManager performance modes
    performance_group.add_argument(
        "--webmanager-low-impact",
        action="store_true",
        help="Enable low impact mode to minimize Isaac Sim performance impact"
    )
    
    performance_group.add_argument(
        "--webmanager-high-performance",
        action="store_true",
        help="Enable high performance mode for maximum WebManager performance (may impact Isaac Sim)"
    )


def parse_arguments(args=None) -> Any:
    """
    Parse command line arguments and return the parsed arguments.
    
    Args:
        args: Optional list of arguments to parse. If None, uses sys.argv
    
    Returns:
        argparse.Namespace: Parsed command line arguments
    """
    parser = create_argument_parser()
    args = parser.parse_args(args)
    
    # Validate argument combinations
    # _validate_arguments(args)
    
    return args


def _validate_arguments(args: Any) -> None:
    """
    Validate argument combinations and set defaults.
    
    Args:
        args: Parsed arguments namespace
        
    Raises:
        ValueError: If invalid argument combinations are detected
    """
    # Validate WebManager configuration
    if args.disable_webmanager and args.enable_webmanager:
        raise ValueError("Cannot specify both --enable-webmanager and --disable-webmanager")
    
    # Validate performance mode settings
    if args.webmanager_low_impact and args.webmanager_high_performance:
        raise ValueError("Cannot specify both --webmanager-low-impact and --webmanager-high-performance")
    
    # Validate port ranges
    if not (1 <= args.web_port <= 65535):
        raise ValueError(f"Web port must be between 1 and 65535, got {args.web_port}")
    
    if args.metrics_port is not None and not (1 <= args.metrics_port <= 65535):
        raise ValueError(f"Metrics port must be between 1 and 65535, got {args.metrics_port}")
    
    # Validate data collection rate
    if args.data_collection_rate <= 0:
        raise ValueError(f"Data collection rate must be positive, got {args.data_collection_rate}")
    
    # Validate max history
    if args.max_history <= 0:
        raise ValueError(f"Max history must be positive, got {args.max_history}")
    
    # Validate health check interval
    if args.health_check_interval <= 0:
        raise ValueError(f"Health check interval must be positive, got {args.health_check_interval}")
    
    # Validate shutdown timeout
    if args.shutdown_timeout <= 0:
        raise ValueError(f"Shutdown timeout must be positive, got {args.shutdown_timeout}")
    
    # Set performance mode defaults
    if not args.webmanager_high_performance:
        # Default to low impact mode unless high performance is explicitly requested
        args.webmanager_low_impact = True


def get_argument_summary(args: Any) -> str:
    """
    Generate a summary of the parsed arguments for logging.
    
    Args:
        args: Parsed arguments namespace
        
    Returns:
        str: Formatted summary of arguments
    """
    webmanager_enabled = args.enable_webmanager and not args.disable_webmanager
    
    summary_lines = [
        "=== Isaac Sim WebManager Configuration ===",
        f"Simulation config: {args.config}",
        f"ROS enabled: {args.ros}",
        "",
        f"WebManager enabled: {webmanager_enabled}",
    ]
    
    if webmanager_enabled:
        performance_mode = "High Performance" if args.webmanager_high_performance else "Low Impact"
        summary_lines.extend([
            f"WebManager host: {args.web_host}",
            f"WebManager port: {args.web_port}",
            f"Performance mode: {performance_mode}",
            f"Data collection rate: {args.data_collection_rate}Hz",
            f"Max history: {args.max_history}",
            f"Camera streaming: {'Disabled' if args.disable_camera_streaming else 'Enabled'}",
            f"Camera quality: {args.camera_quality}%",
            f"Compression: {'Enabled' if args.enable_compression else 'Disabled'}",
        ])
    
    summary_lines.extend([
        "",
        f"Log level: {args.log_level}",
        f"Log file: {args.log_file}",
        f"Health check: {'Enabled' if args.enable_health_check else 'Disabled'}",
        f"Metrics: {'Enabled' if args.enable_metrics else 'Disabled'}",
    ])
    
    if args.pid_file:
        summary_lines.append(f"PID file: {args.pid_file}")
    if args.status_file:
        summary_lines.append(f"Status file: {args.status_file}")
    
    summary_lines.append("=" * 45)
    
    return "\n".join(summary_lines)


if __name__ == "__main__":
    # Test the argument parser
    args = parse_arguments()
    print(get_argument_summary(args))