#!/usr/bin/env python3
"""
WebManager Status Utility
Provides status checking and monitoring for Isaac Sim WebManager processes.
"""

import sys
import os
import json
import time
import argparse
from pathlib import Path

# Add webmanager to path for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    from webmanager.process_manager import StatusReporter
    PROCESS_MANAGER_AVAILABLE = True
except ImportError:
    PROCESS_MANAGER_AVAILABLE = False


def create_parser():
    """Create command line argument parser."""
    parser = argparse.ArgumentParser(
        description="Isaac Sim WebManager Status Utility",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Check status using PID file
  python webmanager_status.py --pid-file /tmp/webmanager.pid
  
  # Check status using status file
  python webmanager_status.py --status-file /tmp/webmanager.status
  
  # Watch status continuously
  python webmanager_status.py --status-file /tmp/webmanager.status --watch
  
  # Get JSON output
  python webmanager_status.py --status-file /tmp/webmanager.status --format json
        """
    )
    
    parser.add_argument(
        "--pid-file",
        type=str,
        help="Path to PID file for status checking"
    )
    
    parser.add_argument(
        "--status-file",
        type=str,
        help="Path to status file for detailed monitoring"
    )
    
    parser.add_argument(
        "--format",
        choices=["text", "json"],
        default="text",
        help="Output format (default: text)"
    )
    
    parser.add_argument(
        "--watch",
        action="store_true",
        help="Watch status continuously (press Ctrl+C to stop)"
    )
    
    parser.add_argument(
        "--interval",
        type=float,
        default=5.0,
        help="Watch interval in seconds (default: 5.0)"
    )
    
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Suppress output, only return exit codes"
    )
    
    return parser


def check_basic_status(pid_file: str = None, status_file: str = None, format: str = "text", quiet: bool = False) -> int:
    """Basic status check without enhanced process manager."""
    
    if status_file and Path(status_file).exists():
        try:
            with open(status_file, 'r') as f:
                status_data = json.load(f)
            
            if format == "json":
                if not quiet:
                    print(json.dumps(status_data, indent=2))
            else:
                if not quiet:
                    print("Isaac Sim WebManager Status (Basic)")
                    print("=" * 40)
                    print(f"Status: {status_data.get('status', 'Unknown')}")
                    print(f"PID: {status_data.get('pid', 'Unknown')}")
                    print(f"Uptime: {status_data.get('uptime_seconds', 0):.1f} seconds")
                    print(f"WebManager Enabled: {status_data.get('webmanager_enabled', False)}")
                    if status_data.get('webmanager_enabled'):
                        print(f"Web Interface: http://{status_data.get('web_host', 'unknown')}:{status_data.get('web_port', 'unknown')}")
            
            return 0
            
        except Exception as e:
            if not quiet:
                print(f"Error reading status file: {e}")
            return 1
    
    if pid_file and Path(pid_file).exists():
        try:
            with open(pid_file, 'r') as f:
                pid = int(f.read().strip())
            
            # Check if process exists (basic check)
            try:
                os.kill(pid, 0)  # Signal 0 just checks if process exists
                if not quiet:
                    if format == "json":
                        print(json.dumps({"pid": pid, "status": "running"}))
                    else:
                        print(f"WebManager process is running (PID: {pid})")
                return 0
            except OSError:
                if not quiet:
                    if format == "json":
                        print(json.dumps({"pid": pid, "status": "not_running"}))
                    else:
                        print(f"WebManager process is not running (PID: {pid})")
                return 1
                
        except Exception as e:
            if not quiet:
                print(f"Error reading PID file: {e}")
            return 1
    
    if not quiet:
        print("No status information available")
        print("Specify --pid-file or --status-file to check status")
    return 1


def check_enhanced_status(pid_file: str = None, status_file: str = None, format: str = "text", quiet: bool = False) -> int:
    """Enhanced status check using process manager."""
    
    if status_file:
        status_data = StatusReporter.read_status_file(status_file)
        if status_data:
            if format == "json":
                if not quiet:
                    print(json.dumps(status_data, indent=2))
            else:
                if not quiet:
                    print(StatusReporter.format_status_report(status_data))
            return 0
        else:
            if not quiet:
                print("Status file not found or unreadable")
            return 1
    
    if pid_file:
        pid = StatusReporter.read_pid_file(pid_file)
        if pid and StatusReporter.is_process_running(pid):
            process_info = StatusReporter.get_process_info(pid)
            if process_info:
                if format == "json":
                    if not quiet:
                        print(json.dumps(process_info, indent=2))
                else:
                    if not quiet:
                        print(f"WebManager is running (PID: {pid})")
                        print(f"Process: {process_info['name']}")
                        print(f"CPU: {process_info['cpu_percent']:.1f}%")
                        print(f"Memory: {process_info['memory_info']['rss'] / 1024 / 1024:.1f} MB")
                return 0
            else:
                if not quiet:
                    print(f"WebManager process found (PID: {pid}) but could not get details")
                return 0
        else:
            if not quiet:
                print("WebManager is not running")
            return 1
    
    if not quiet:
        print("No status information available")
        print("Specify --pid-file or --status-file to check status")
    return 1


def main():
    """Main function."""
    parser = create_parser()
    args = parser.parse_args()
    
    if not args.pid_file and not args.status_file:
        print("Error: Either --pid-file or --status-file must be specified", file=sys.stderr)
        return 1
    
    # Choose status check function based on availability
    if PROCESS_MANAGER_AVAILABLE:
        status_func = check_enhanced_status
    else:
        status_func = check_basic_status
    
    if args.watch:
        try:
            while True:
                # Clear screen
                if not args.quiet:
                    os.system('clear' if os.name == 'posix' else 'cls')
                    print(f"WebManager Status Monitor (refreshing every {args.interval}s)")
                    print("Press Ctrl+C to stop")
                    print("=" * 60)
                
                exit_code = status_func(
                    pid_file=args.pid_file,
                    status_file=args.status_file,
                    format=args.format,
                    quiet=args.quiet
                )
                
                if not args.quiet and exit_code != 0:
                    print("\nProcess appears to be stopped or unreachable")
                
                time.sleep(args.interval)
                
        except KeyboardInterrupt:
            if not args.quiet:
                print("\nStatus monitoring stopped")
            return 0
    else:
        return status_func(
            pid_file=args.pid_file,
            status_file=args.status_file,
            format=args.format,
            quiet=args.quiet
        )


if __name__ == "__main__":
    sys.exit(main())