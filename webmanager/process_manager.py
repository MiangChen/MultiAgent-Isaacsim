"""
Process management utilities for WebManager.
Handles PID files, status reporting, graceful shutdown, and health monitoring.
"""

import os
import sys
import time
import json
import signal
import threading
import logging
import psutil
from pathlib import Path
from typing import Dict, Any, Optional, Callable
from dataclasses import dataclass, asdict
from datetime import datetime, timezone


@dataclass
class ProcessStatus:
    """Process status information."""
    pid: int
    start_time: datetime
    status: str  # "starting", "running", "stopping", "stopped", "error"
    webmanager_enabled: bool
    web_host: str
    web_port: int
    ros_enabled: bool
    uptime_seconds: float
    memory_usage_mb: float
    cpu_percent: float
    active_connections: int = 0
    total_requests: int = 0
    error_count: int = 0
    last_error: Optional[str] = None
    last_health_check: Optional[datetime] = None
    health_status: str = "unknown"  # "healthy", "warning", "critical", "unknown"
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        data = asdict(self)
        # Convert datetime objects to ISO format strings
        if self.start_time:
            data['start_time'] = self.start_time.isoformat()
        if self.last_health_check:
            data['last_health_check'] = self.last_health_check.isoformat()
        return data


class ProcessManager:
    """Manages process lifecycle, PID files, and status reporting."""
    
    def __init__(self, config):
        """Initialize process manager with configuration."""
        self.config = config
        self.logger = logging.getLogger(__name__)
        self.pid = os.getpid()
        self.start_time = datetime.now(timezone.utc)
        self.shutdown_requested = False
        self.shutdown_callbacks: list[Callable] = []
        self.status = ProcessStatus(
            pid=self.pid,
            start_time=self.start_time,
            status="starting",
            webmanager_enabled=config.enable_webmanager,
            web_host=config.web_host,
            web_port=config.web_port,
            ros_enabled=config.enable_ros,
            uptime_seconds=0.0,
            memory_usage_mb=0.0,
            cpu_percent=0.0
        )
        
        # Health monitoring
        self.health_check_thread: Optional[threading.Thread] = None
        self.health_check_stop_event = threading.Event()
        
        # Process monitoring
        self.process = psutil.Process(self.pid)
        
        # Setup signal handlers
        self._setup_signal_handlers()
        
        # Write PID file if configured
        if config.pid_file:
            self._write_pid_file()
        
        # Initialize status file
        if config.status_file:
            self._update_status_file()
    
    def _setup_signal_handlers(self):
        """Setup signal handlers for graceful shutdown."""
        def signal_handler(signum, frame):
            signal_names = {
                signal.SIGINT: "SIGINT (Ctrl+C)",
                signal.SIGTERM: "SIGTERM"
            }
            signal_name = signal_names.get(signum, f"Signal {signum}")
            self.logger.info(f"Received {signal_name}, initiating graceful shutdown...")
            self.request_shutdown()
        
        # Register signal handlers
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
        # On Windows, also handle SIGBREAK
        if sys.platform == "win32":
            signal.signal(signal.SIGBREAK, signal_handler)
    
    def _write_pid_file(self):
        """Write PID to file for external monitoring."""
        try:
            pid_path = Path(self.config.pid_file)
            pid_path.parent.mkdir(parents=True, exist_ok=True)
            
            with open(pid_path, 'w') as f:
                f.write(str(self.pid))
            
            self.logger.info(f"PID file written: {self.config.pid_file}")
            
        except Exception as e:
            self.logger.error(f"Failed to write PID file: {e}")
    
    def _remove_pid_file(self):
        """Remove PID file on shutdown."""
        if self.config.pid_file:
            try:
                pid_path = Path(self.config.pid_file)
                if pid_path.exists():
                    pid_path.unlink()
                    self.logger.info(f"PID file removed: {self.config.pid_file}")
            except Exception as e:
                self.logger.error(f"Failed to remove PID file: {e}")
    
    def _update_status_file(self):
        """Update status file with current process information."""
        if not self.config.status_file:
            return
        
        try:
            # Update process metrics
            self.status.uptime_seconds = (datetime.now(timezone.utc) - self.start_time).total_seconds()
            
            try:
                # Get process metrics
                memory_info = self.process.memory_info()
                self.status.memory_usage_mb = memory_info.rss / 1024 / 1024
                self.status.cpu_percent = self.process.cpu_percent()
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                self.logger.warning("Could not get process metrics")
            
            # Write status file
            status_path = Path(self.config.status_file)
            status_path.parent.mkdir(parents=True, exist_ok=True)
            
            with open(status_path, 'w') as f:
                json.dump(self.status.to_dict(), f, indent=2)
            
        except Exception as e:
            self.logger.error(f"Failed to update status file: {e}")
    
    def start_health_monitoring(self):
        """Start health check monitoring thread."""
        if not self.config.enable_health_check:
            return
        
        def health_check_worker():
            """Health check worker thread."""
            self.logger.info(f"Health monitoring started (interval: {self.config.health_check_interval}s)")
            
            while not self.health_check_stop_event.is_set():
                try:
                    self._perform_health_check()
                    self._update_status_file()
                    
                    # Wait for next check or stop event
                    self.health_check_stop_event.wait(timeout=self.config.health_check_interval)
                    
                except Exception as e:
                    self.logger.error(f"Error in health check: {e}")
                    self.status.health_status = "critical"
                    self.status.last_error = str(e)
                    self.status.error_count += 1
        
        self.health_check_thread = threading.Thread(
            target=health_check_worker,
            name="HealthCheckThread",
            daemon=True
        )
        self.health_check_thread.start()
    
    def _perform_health_check(self):
        """Perform health check and update status."""
        try:
            self.status.last_health_check = datetime.now(timezone.utc)
            
            # Check process health
            if not self.process.is_running():
                self.status.health_status = "critical"
                self.status.last_error = "Process not running"
                return
            
            # Check memory usage
            memory_info = self.process.memory_info()
            memory_mb = memory_info.rss / 1024 / 1024
            
            # Check CPU usage
            cpu_percent = self.process.cpu_percent()
            
            # Update metrics
            self.status.memory_usage_mb = memory_mb
            self.status.cpu_percent = cpu_percent
            
            # Determine health status based on metrics
            if memory_mb > 2048:  # More than 2GB
                self.status.health_status = "warning"
                self.status.last_error = f"High memory usage: {memory_mb:.1f}MB"
            elif cpu_percent > 90:  # More than 90% CPU
                self.status.health_status = "warning"
                self.status.last_error = f"High CPU usage: {cpu_percent:.1f}%"
            else:
                self.status.health_status = "healthy"
                self.status.last_error = None
            
        except Exception as e:
            self.status.health_status = "critical"
            self.status.last_error = str(e)
            self.status.error_count += 1
            self.logger.error(f"Health check failed: {e}")
    
    def register_shutdown_callback(self, callback: Callable):
        """Register a callback to be called during shutdown."""
        self.shutdown_callbacks.append(callback)
    
    def request_shutdown(self):
        """Request graceful shutdown."""
        if self.shutdown_requested:
            self.logger.warning("Shutdown already requested")
            return
        
        self.shutdown_requested = True
        self.status.status = "stopping"
        self.logger.info("Graceful shutdown requested")
        
        # Update status file
        self._update_status_file()
        
        # Execute shutdown callbacks
        self._execute_shutdown_callbacks()
    
    def _execute_shutdown_callbacks(self):
        """Execute all registered shutdown callbacks."""
        self.logger.info(f"Executing {len(self.shutdown_callbacks)} shutdown callbacks...")
        
        for i, callback in enumerate(self.shutdown_callbacks):
            try:
                self.logger.debug(f"Executing shutdown callback {i+1}/{len(self.shutdown_callbacks)}")
                callback()
            except Exception as e:
                self.logger.error(f"Error in shutdown callback {i+1}: {e}")
        
        self.logger.info("All shutdown callbacks completed")
    
    def wait_for_shutdown(self, timeout: Optional[float] = None) -> bool:
        """Wait for shutdown to complete."""
        if timeout is None:
            timeout = self.config.shutdown_timeout
        
        start_time = time.time()
        
        while not self.shutdown_requested and (time.time() - start_time) < timeout:
            time.sleep(0.1)
        
        return self.shutdown_requested
    
    def cleanup(self):
        """Cleanup resources and files."""
        self.logger.info("Cleaning up process manager resources...")
        
        # Stop health monitoring
        if self.health_check_thread and self.health_check_thread.is_alive():
            self.health_check_stop_event.set()
            self.health_check_thread.join(timeout=5.0)
            if self.health_check_thread.is_alive():
                self.logger.warning("Health check thread did not stop gracefully")
        
        # Update final status
        self.status.status = "stopped"
        self._update_status_file()
        
        # Remove PID file
        self._remove_pid_file()
        
        self.logger.info("Process manager cleanup completed")
    
    def update_webmanager_stats(self, active_connections: int = 0, total_requests: int = 0):
        """Update WebManager statistics."""
        self.status.active_connections = active_connections
        self.status.total_requests = total_requests
    
    def report_error(self, error_message: str):
        """Report an error for monitoring."""
        self.status.error_count += 1
        self.status.last_error = error_message
        self.logger.error(f"Error reported: {error_message}")
    
    def set_status(self, status: str):
        """Set the current process status."""
        self.status.status = status
        self.logger.info(f"Process status changed to: {status}")
        self._update_status_file()
    
    def get_status(self) -> ProcessStatus:
        """Get current process status."""
        # Update uptime
        self.status.uptime_seconds = (datetime.now(timezone.utc) - self.start_time).total_seconds()
        return self.status
    
    def is_shutdown_requested(self) -> bool:
        """Check if shutdown has been requested."""
        return self.shutdown_requested


class StatusReporter:
    """Utility class for reading and reporting process status."""
    
    @staticmethod
    def read_status_file(status_file: str) -> Optional[Dict[str, Any]]:
        """Read status from file."""
        try:
            with open(status_file, 'r') as f:
                return json.load(f)
        except Exception:
            return None
    
    @staticmethod
    def read_pid_file(pid_file: str) -> Optional[int]:
        """Read PID from file."""
        try:
            with open(pid_file, 'r') as f:
                return int(f.read().strip())
        except Exception:
            return None
    
    @staticmethod
    def is_process_running(pid: int) -> bool:
        """Check if process with given PID is running."""
        try:
            return psutil.pid_exists(pid)
        except Exception:
            return False
    
    @staticmethod
    def get_process_info(pid: int) -> Optional[Dict[str, Any]]:
        """Get detailed process information."""
        try:
            process = psutil.Process(pid)
            return {
                'pid': pid,
                'name': process.name(),
                'status': process.status(),
                'cpu_percent': process.cpu_percent(),
                'memory_info': process.memory_info()._asdict(),
                'create_time': process.create_time(),
                'cmdline': process.cmdline()
            }
        except Exception:
            return None
    
    @staticmethod
    def format_status_report(status_data: Dict[str, Any]) -> str:
        """Format status data into a human-readable report."""
        if not status_data:
            return "No status data available"
        
        lines = [
            "Isaac Sim WebManager Status Report",
            "=" * 40,
            f"PID: {status_data.get('pid', 'Unknown')}",
            f"Status: {status_data.get('status', 'Unknown')}",
            f"Start Time: {status_data.get('start_time', 'Unknown')}",
            f"Uptime: {status_data.get('uptime_seconds', 0):.1f} seconds",
            f"Memory Usage: {status_data.get('memory_usage_mb', 0):.1f} MB",
            f"CPU Usage: {status_data.get('cpu_percent', 0):.1f}%",
            "",
            "WebManager Configuration:",
            f"  Enabled: {status_data.get('webmanager_enabled', False)}",
            f"  Host: {status_data.get('web_host', 'Unknown')}",
            f"  Port: {status_data.get('web_port', 'Unknown')}",
            f"  Active Connections: {status_data.get('active_connections', 0)}",
            f"  Total Requests: {status_data.get('total_requests', 0)}",
            "",
            f"ROS Enabled: {status_data.get('ros_enabled', False)}",
            f"Health Status: {status_data.get('health_status', 'Unknown')}",
            f"Error Count: {status_data.get('error_count', 0)}",
        ]
        
        if status_data.get('last_error'):
            lines.extend([
                f"Last Error: {status_data['last_error']}",
            ])
        
        if status_data.get('last_health_check'):
            lines.extend([
                f"Last Health Check: {status_data['last_health_check']}",
            ])
        
        return "\n".join(lines)


def create_status_command_parser() -> 'argparse.ArgumentParser':
    """Create argument parser for status command."""
    import argparse
    
    parser = argparse.ArgumentParser(
        description="Isaac Sim WebManager Status Utility",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    
    parser.add_argument(
        "--pid-file",
        type=str,
        help="Path to PID file"
    )
    
    parser.add_argument(
        "--status-file",
        type=str,
        help="Path to status file"
    )
    
    parser.add_argument(
        "--format",
        choices=["json", "text"],
        default="text",
        help="Output format (default: text)"
    )
    
    parser.add_argument(
        "--watch",
        action="store_true",
        help="Watch status continuously"
    )
    
    parser.add_argument(
        "--interval",
        type=float,
        default=5.0,
        help="Watch interval in seconds (default: 5.0)"
    )
    
    return parser


def main_status_command():
    """Main function for status command utility."""
    parser = create_status_command_parser()
    args = parser.parse_args()
    
    if not args.pid_file and not args.status_file:
        print("Error: Either --pid-file or --status-file must be specified", file=sys.stderr)
        sys.exit(1)
    
    def print_status():
        """Print current status."""
        if args.status_file:
            status_data = StatusReporter.read_status_file(args.status_file)
            if status_data:
                if args.format == "json":
                    print(json.dumps(status_data, indent=2))
                else:
                    print(StatusReporter.format_status_report(status_data))
            else:
                print("Status file not found or unreadable")
                return False
        
        if args.pid_file:
            pid = StatusReporter.read_pid_file(args.pid_file)
            if pid:
                if StatusReporter.is_process_running(pid):
                    process_info = StatusReporter.get_process_info(pid)
                    if process_info and args.format == "json":
                        print(json.dumps(process_info, indent=2))
                    elif process_info:
                        print(f"Process {pid} is running: {process_info['name']}")
                else:
                    print(f"Process {pid} is not running")
                    return False
            else:
                print("PID file not found or unreadable")
                return False
        
        return True
    
    if args.watch:
        try:
            while True:
                os.system('clear' if os.name == 'posix' else 'cls')
                print(f"WebManager Status (refreshing every {args.interval}s)")
                print("=" * 50)
                if not print_status():
                    print("Process appears to be stopped")
                time.sleep(args.interval)
        except KeyboardInterrupt:
            print("\nStatus monitoring stopped")
    else:
        if not print_status():
            sys.exit(1)


if __name__ == "__main__":
    main_status_command()