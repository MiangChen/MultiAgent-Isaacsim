#!/usr/bin/env python3
"""
Enhanced startup script for Isaac Sim WebManager with full configuration support.
This script demonstrates the complete startup integration with command line arguments,
graceful shutdown handling, and comprehensive logging and status reporting.
"""

import sys
import os
import subprocess
import logging
from pathlib import Path

# Add current directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

def main():
    """Main startup function with enhanced configuration."""
    
    # Setup basic logging for the startup script
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
    )
    logger = logging.getLogger("startup")
    
    logger.info("Isaac Sim WebManager Enhanced Startup")
    
    # Check if main.py exists
    if not Path("main.py").exists():
        logger.error("main.py not found in current directory")
        logger.error("Please run this script from the Isaac Sim project root directory")
        return 1
    
    # Example command line arguments for different scenarios
    
    # Development mode with debug logging and process monitoring
    dev_args = [
        "python", "main.py",
        "--log-level", "DEBUG",
        "--data-collection-rate", "20.0",
        "--health-check-interval", "10.0",
        "--pid-file", "/tmp/webmanager_dev.pid",
        "--status-file", "/tmp/webmanager_dev.status",
        "--enable-health-check",
        "--shutdown-timeout", "15.0"
    ]
    
    # Production mode with optimized settings
    prod_args = [
        "python", "main.py",
        "--log-level", "WARNING",
        "--log-file", "/var/log/isaac_sim_webmanager.log",
        "--data-collection-rate", "5.0",
        "--camera-quality", "60",
        "--enable-compression",
        "--pid-file", "/var/run/webmanager.pid",
        "--status-file", "/var/run/webmanager.status",
        "--enable-health-check",
        "--health-check-interval", "60.0",
        "--shutdown-timeout", "30.0",
        "--enable-auto-restart",
        "--max-restart-attempts", "5"
    ]
    
    # Minimal mode (simulation only, no web interface)
    minimal_args = [
        "python", "main.py",
        "--disable-webmanager",
        "--log-level", "ERROR",
        "--disable-camera-streaming"
    ]
    
    # Choose configuration based on environment or command line
    if len(sys.argv) > 1:
        mode = sys.argv[1].lower()
    else:
        mode = "dev"  # Default to development mode
    
    if mode == "prod" or mode == "production":
        cmd_args = prod_args
        logger.info("Starting in PRODUCTION mode")
    elif mode == "minimal":
        cmd_args = minimal_args
        logger.info("Starting in MINIMAL mode (no WebManager)")
    else:
        cmd_args = dev_args
        logger.info("Starting in DEVELOPMENT mode")
    
    # Add any additional arguments passed to this script
    if len(sys.argv) > 2:
        cmd_args.extend(sys.argv[2:])
    
    logger.info(f"Command: {' '.join(cmd_args)}")
    
    try:
        # Start the main application
        logger.info("Starting Isaac Sim WebManager...")
        result = subprocess.run(cmd_args, check=False)
        
        if result.returncode == 0:
            logger.info("Isaac Sim WebManager finished successfully")
        else:
            logger.error(f"Isaac Sim WebManager exited with code {result.returncode}")
        
        return result.returncode
        
    except KeyboardInterrupt:
        logger.info("Startup interrupted by user")
        return 130
    except Exception as e:
        logger.error(f"Error starting Isaac Sim WebManager: {e}")
        return 1


def print_usage():
    """Print usage information."""
    print("""
Isaac Sim WebManager Enhanced Startup

Usage:
  python start_webmanager_enhanced.py [mode] [additional_args...]

Modes:
  dev         Development mode with debug logging and frequent updates (default)
  prod        Production mode with optimized settings and monitoring
  minimal     Minimal mode with no web interface (simulation only)

Examples:
  # Start in development mode
  python start_webmanager_enhanced.py dev
  
  # Start in production mode
  python start_webmanager_enhanced.py prod
  
  # Start in minimal mode
  python start_webmanager_enhanced.py minimal
  
  # Start with custom port
  python start_webmanager_enhanced.py dev --web-port 8081
  
  # Start with configuration file
  python start_webmanager_enhanced.py prod --webmanager-config webmanager_config.yaml

Status Monitoring:
  # Check status using PID file
  python webmanager_status.py --pid-file /tmp/webmanager_dev.pid
  
  # Watch status continuously
  python webmanager_status.py --status-file /tmp/webmanager_dev.status --watch
  
  # Get JSON status
  python webmanager_status.py --status-file /tmp/webmanager_dev.status --format json
""")


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] in ["-h", "--help", "help"]:
        print_usage()
        sys.exit(0)
    
    sys.exit(main())