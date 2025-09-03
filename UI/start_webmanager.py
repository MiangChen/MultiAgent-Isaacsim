#!/usr/bin/env python3
"""
Enhanced startup script for Isaac Sim WebManager
Provides comprehensive configuration and startup options for the WebManager system.
"""

import argparse
import os
import sys
import subprocess
import logging
import time
import signal
from pathlib import Path
from typing import Optional, List

# Import WebManager configuration modules
try:
    from webmanager.startup_config import WebManagerConfig, parse_arguments, setup_enhanced_logging
    from webmanager.process_manager import ProcessManager, StatusReporter
    WEBMANAGER_MODULES_AVAILABLE = True
except ImportError:
    WEBMANAGER_MODULES_AVAILABLE = False


def setup_basic_logging(log_level="INFO"):
    """Setup basic logging configuration (fallback)"""
    logging.basicConfig(
        level=getattr(logging, log_level.upper()),
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        handlers=[
            logging.StreamHandler(),
            logging.FileHandler("webmanager_startup.log", mode='a')
        ]
    )
    return logging.getLogger(__name__)


def check_dependencies():
    """Check if required dependencies are installed"""
    logger = logging.getLogger(__name__)
    
    required_packages = [
        'fastapi',
        'uvicorn',
        'websockets',
        'numpy',
        'opencv-python',
        'psutil'
    ]
    
    missing_packages = []
    
    for package in required_packages:
        try:
            __import__(package.replace('-', '_'))
        except ImportError:
            missing_packages.append(package)
    
    if missing_packages:
        logger.error(f"Missing required packages: {', '.join(missing_packages)}")
        logger.error("Please install missing packages with: pip install -r requirements.txt")
        return False
    
    logger.info("All required dependencies are installed")
    return True


def check_isaac_sim():
    """Check if Isaac Sim is available"""
    logger = logging.getLogger(__name__)
    
    try:
        # Try to import Isaac Sim modules
        import omni
        logger.info("Isaac Sim modules are available")
        return True
    except ImportError:
        logger.warning("Isaac Sim modules not found - make sure Isaac Sim environment is activated")
        return False


def build_enhanced_command_args(config: 'WebManagerConfig') -> List[str]:
    """Build enhanced command line arguments for main.py from configuration"""
    cmd_args = ["python", "main.py"]
    
    # Add configuration file
    if config.config_file:
        cmd_args.extend(["--config", config.config_file])
    
    # WebManager configuration
    if config.enable_webmanager:
        cmd_args.append("--enable-webmanager")
        cmd_args.extend(["--web-host", config.web_host])
        cmd_args.extend(["--web-port", str(config.web_port)])
    else:
        cmd_args.append("--disable-webmanager")
    
    # Data collection settings
    cmd_args.extend(["--data-collection-rate", str(config.data_collection_rate)])
    cmd_args.extend(["--max-history", str(config.max_history)])
    
    # Logging
    cmd_args.extend(["--log-level", config.log_level])
    cmd_args.extend(["--log-file", config.log_file])
    
    # Camera settings
    if config.disable_camera_streaming:
        cmd_args.append("--disable-camera-streaming")
    
    cmd_args.extend(["--camera-quality", str(config.camera_quality)])
    
    # Other options
    if config.enable_compression:
        cmd_args.append("--enable-compression")
    
    if not config.enable_ros:
        cmd_args.extend(["--ros", "False"])
    
    # Process management options
    if config.pid_file:
        cmd_args.extend(["--pid-file", config.pid_file])
    
    if config.status_file:
        cmd_args.extend(["--status-file", config.status_file])
    
    if config.enable_metrics:
        cmd_args.append("--enable-metrics")
        if config.metrics_port:
            cmd_args.extend(["--metrics-port", str(config.metrics_port)])
    
    if config.enable_health_check:
        cmd_args.append("--enable-health-check")
        cmd_args.extend(["--health-check-interval", str(config.health_check_interval)])
    
    cmd_args.extend(["--shutdown-timeout", str(config.shutdown_timeout)])
    
    if config.enable_auto_restart:
        cmd_args.append("--enable-auto-restart")
        cmd_args.extend(["--max-restart-attempts", str(config.max_restart_attempts)])
    
    return cmd_args


def build_command_args(args):
    """Build command line arguments for main.py (basic version)"""
    cmd_args = ["python", "main.py"]
    
    # Add configuration file
    if args.config:
        cmd_args.extend(["--config", args.config])
    
    # WebManager configuration
    if args.disable_webmanager:
        cmd_args.append("--disable-webmanager")
    else:
        cmd_args.append("--enable-webmanager")
        
        if args.host:
            cmd_args.extend(["--web-host", args.host])
        
        if args.port:
            cmd_args.extend(["--web-port", str(args.port)])
    
    # Data collection settings
    if args.collection_rate:
        cmd_args.extend(["--data-collection-rate", str(args.collection_rate)])
    
    if args.max_history:
        cmd_args.extend(["--max-history", str(args.max_history)])
    
    # Logging
    if args.log_level:
        cmd_args.extend(["--log-level", args.log_level])
    
    # Camera settings
    if args.disable_camera:
        cmd_args.append("--disable-camera-streaming")
    
    if args.camera_quality:
        cmd_args.extend(["--camera-quality", str(args.camera_quality)])
    
    # Other options
    if args.enable_compression:
        cmd_args.append("--enable-compression")
    
    if not args.ros:
        cmd_args.extend(["--ros", "False"])
    
    return cmd_args


def start_with_monitoring(cmd_args: List[str], config: 'WebManagerConfig', logger: logging.Logger) -> int:
    """Start the process with enhanced monitoring and restart capabilities"""
    
    restart_attempts = 0
    max_attempts = config.max_restart_attempts if config.enable_auto_restart else 0
    
    while restart_attempts <= max_attempts:
        try:
            if restart_attempts > 0:
                logger.info(f"Restart attempt {restart_attempts}/{max_attempts}")
                time.sleep(5)  # Wait before restart
            
            # Start the process
            logger.info(f"Starting process: {' '.join(cmd_args)}")
            process = subprocess.Popen(
                cmd_args,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                bufsize=1
            )
            
            # Monitor the process
            return_code = monitor_process(process, config, logger)
            
            if return_code == 0:
                logger.info("Process completed successfully")
                return 0
            elif return_code < 0:
                # Process was terminated by signal
                logger.warning(f"Process terminated by signal {-return_code}")
                if not config.enable_auto_restart:
                    return return_code
            else:
                # Process exited with error
                logger.error(f"Process exited with code {return_code}")
                if not config.enable_auto_restart:
                    return return_code
            
            restart_attempts += 1
            
        except KeyboardInterrupt:
            logger.info("Startup interrupted by user")
            if 'process' in locals():
                logger.info("Terminating child process...")
                process.terminate()
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    logger.warning("Child process did not terminate, killing...")
                    process.kill()
            return 130
        except Exception as e:
            logger.error(f"Error starting process: {e}")
            restart_attempts += 1
            if restart_attempts > max_attempts:
                return 1
    
    logger.error(f"Maximum restart attempts ({max_attempts}) exceeded")
    return 1


def monitor_process(process: subprocess.Popen, config: 'WebManagerConfig', logger: logging.Logger) -> int:
    """Monitor the running process and handle output"""
    
    logger.info(f"Process started with PID: {process.pid}")
    
    # Setup signal handler for graceful shutdown
    def signal_handler(signum, frame):
        logger.info(f"Received signal {signum}, terminating child process...")
        process.terminate()
    
    original_sigint = signal.signal(signal.SIGINT, signal_handler)
    original_sigterm = signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        # Monitor process output
        while True:
            output = process.stdout.readline()
            if output == '' and process.poll() is not None:
                break
            if output:
                # Forward output to our logger
                logger.info(f"[CHILD] {output.strip()}")
        
        # Wait for process to complete
        return_code = process.wait()
        logger.info(f"Process completed with return code: {return_code}")
        return return_code
        
    except Exception as e:
        logger.error(f"Error monitoring process: {e}")
        return 1
    finally:
        # Restore original signal handlers
        signal.signal(signal.SIGINT, original_sigint)
        signal.signal(signal.SIGTERM, original_sigterm)


def main():
    """Enhanced main startup function"""
    
    # Check if enhanced WebManager modules are available
    if not WEBMANAGER_MODULES_AVAILABLE:
        print("Warning: Enhanced WebManager modules not available, using basic startup", file=sys.stderr)
        return main_basic()
    
    # Use enhanced startup with full configuration support
    return main_enhanced()


def main_enhanced():
    """Enhanced startup with full WebManager configuration support"""
    
    try:
        # Parse arguments using enhanced configuration system
        config = parse_arguments()
        
        # Setup enhanced logging
        logger = setup_enhanced_logging(config)
        
        logger.info("Isaac Sim WebManager Enhanced Startup")
        logger.info(f"Configuration loaded: {config.to_dict()}")
        
        # Check dependencies
        if not check_dependencies():
            logger.error("Missing dependencies - cannot start")
            return 1
        
        # Check if main.py exists
        if not Path("../main.py").exists():
            logger.error("main.py not found in current directory")
            logger.error("Please run this script from the Isaac Sim project root directory")
            return 1
        
        # Build command arguments for main.py
        cmd_args = build_enhanced_command_args(config)
        
        # Log startup information
        logger.info("Starting Isaac Sim with enhanced WebManager...")
        if config.enable_webmanager:
            logger.info(f"WebManager will be available at: http://{config.web_host}:{config.web_port}")
        logger.info(f"Command: {' '.join(cmd_args)}")
        
        # Start the process with monitoring
        return start_with_monitoring(cmd_args, config, logger)
        
    except KeyboardInterrupt:
        print("\nStartup interrupted by user")
        return 130  # Standard exit code for Ctrl+C
    except Exception as e:
        print(f"Error in enhanced startup: {e}", file=sys.stderr)
        return 1


def main_basic():
    """Basic startup function (fallback when enhanced modules not available)"""
    
    parser = argparse.ArgumentParser(
        description="Isaac Sim WebManager Startup Script (Basic Mode)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Start with default settings
  python start_webmanager.py
  
  # Start on specific host and port
  python start_webmanager.py --host 192.168.1.100 --port 8081
  
  # Start without camera streaming
  python start_webmanager.py --disable-camera
  
  # Start with debug logging
  python start_webmanager.py --log-level DEBUG
  
  # Start without WebManager (simulation only)
  python start_webmanager.py --disable-webmanager
        """
    )
    
    # Basic configuration
    parser.add_argument(
        "--config",
        type=str,
        default="./files/sim_cfg.yaml",
        help="Path to Isaac Sim configuration file"
    )
    
    # WebManager settings
    parser.add_argument(
        "--disable-webmanager",
        action="store_true",
        help="Disable WebManager web interface"
    )
    
    parser.add_argument(
        "--host",
        type=str,
        default="0.0.0.0",
        help="WebManager server host (default: 0.0.0.0)"
    )
    
    parser.add_argument(
        "--port",
        type=int,
        default=8080,
        help="WebManager server port (default: 8080)"
    )
    
    # Data collection settings
    parser.add_argument(
        "--collection-rate",
        type=float,
        default=10.0,
        help="Data collection rate in Hz (default: 10.0)"
    )
    
    parser.add_argument(
        "--max-history",
        type=int,
        default=1000,
        help="Maximum historical data points (default: 1000)"
    )
    
    # Logging
    parser.add_argument(
        "--log-level",
        choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
        default="INFO",
        help="Logging level (default: INFO)"
    )
    
    # Camera settings
    parser.add_argument(
        "--disable-camera",
        action="store_true",
        help="Disable camera streaming"
    )
    
    parser.add_argument(
        "--camera-quality",
        type=int,
        choices=range(10, 101),
        metavar="[10-100]",
        default=80,
        help="Camera JPEG quality (default: 80)"
    )
    
    # Other options
    parser.add_argument(
        "--enable-compression",
        action="store_true",
        help="Enable WebSocket message compression"
    )
    
    parser.add_argument(
        "--no-ros",
        dest="ros",
        action="store_false",
        help="Disable ROS integration"
    )
    
    parser.add_argument(
        "--check-deps",
        action="store_true",
        help="Check dependencies and exit"
    )
    
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Show command that would be executed without running it"
    )
    
    # Status and monitoring options
    parser.add_argument(
        "--status",
        action="store_true",
        help="Show status of running WebManager instance"
    )
    
    parser.add_argument(
        "--pid-file",
        type=str,
        help="Path to PID file for status checking"
    )
    
    parser.add_argument(
        "--status-file",
        type=str,
        help="Path to status file for monitoring"
    )
    
    args = parser.parse_args()
    
    # Handle status command
    if args.status:
        return handle_status_command(args)
    
    # Setup logging
    logger = setup_basic_logging(args.log_level)
    logger.info("Isaac Sim WebManager Startup Script (Basic Mode)")
    
    # Check dependencies if requested
    if args.check_deps:
        logger.info("Checking dependencies...")
        deps_ok = check_dependencies()
        isaac_ok = check_isaac_sim()
        
        if deps_ok and isaac_ok:
            logger.info("All dependencies are satisfied")
            return 0
        else:
            logger.error("Some dependencies are missing")
            return 1
    
    # Check if main.py exists
    if not Path("../main.py").exists():
        logger.error("main.py not found in current directory")
        logger.error("Please run this script from the Isaac Sim project root directory")
        return 1
    
    # Check dependencies
    if not check_dependencies():
        logger.error("Missing dependencies - use --check-deps for details")
        return 1
    
    # Build command arguments
    cmd_args = build_command_args(args)
    
    # Show command or execute
    if args.dry_run:
        logger.info(f"Would execute: {' '.join(cmd_args)}")
        return 0
    
    # Log startup information
    logger.info(f"Starting Isaac Sim with WebManager...")
    if not args.disable_webmanager:
        logger.info(f"WebManager will be available at: http://{args.host}:{args.port}")
    logger.info(f"Command: {' '.join(cmd_args)}")
    
    try:
        # Execute the main application
        result = subprocess.run(cmd_args, check=False)
        
        if result.returncode == 0:
            logger.info("Isaac Sim WebManager finished successfully")
            return 0
        else:
            logger.error(f"Isaac Sim WebManager exited with code {result.returncode}")
            return result.returncode
            
    except KeyboardInterrupt:
        logger.info("Startup interrupted by user")
        return 130  # Standard exit code for Ctrl+C
    except Exception as e:
        logger.error(f"Error starting Isaac Sim WebManager: {e}")
        return 1


def handle_status_command(args) -> int:
    """Handle status command for checking running WebManager instance"""
    
    if not args.pid_file and not args.status_file:
        print("Error: Either --pid-file or --status-file must be specified for status check", file=sys.stderr)
        return 1
    
    try:
        if WEBMANAGER_MODULES_AVAILABLE:
            # Use enhanced status reporting
            if args.status_file:
                status_data = StatusReporter.read_status_file(args.status_file)
                if status_data:
                    print(StatusReporter.format_status_report(status_data))
                    return 0
                else:
                    print("Status file not found or unreadable")
                    return 1
            
            if args.pid_file:
                pid = StatusReporter.read_pid_file(args.pid_file)
                if pid and StatusReporter.is_process_running(pid):
                    process_info = StatusReporter.get_process_info(pid)
                    if process_info:
                        print(f"WebManager is running (PID: {pid})")
                        print(f"Process: {process_info['name']}")
                        print(f"CPU: {process_info['cpu_percent']:.1f}%")
                        print(f"Memory: {process_info['memory_info']['rss'] / 1024 / 1024:.1f} MB")
                        return 0
                else:
                    print("WebManager is not running")
                    return 1
        else:
            # Basic status check
            if args.pid_file and Path(args.pid_file).exists():
                with open(args.pid_file, 'r') as f:
                    pid = int(f.read().strip())
                try:
                    os.kill(pid, 0)  # Check if process exists
                    print(f"WebManager is running (PID: {pid})")
                    return 0
                except OSError:
                    print("WebManager is not running")
                    return 1
            else:
                print("No status information available")
                return 1
                
    except Exception as e:
        print(f"Error checking status: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    main()