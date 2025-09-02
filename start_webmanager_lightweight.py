#!/usr/bin/env python3
"""
Lightweight WebManager Startup Script

Starts WebManager with optimized settings for minimal resource usage.
"""

import sys
import subprocess
import logging
from pathlib import Path

# Add current directory to path for imports
sys.path.insert(0, str(Path(__file__).parent))

from webmanager_resource_optimization import (
    OptimizationLevel, 
    get_resource_friendly_startup_args,
    create_optimized_webmanager_config
)


def main():
    """Start WebManager with lightweight configuration."""
    
    print("🚀 Starting WebManager in Lightweight Mode")
    print("=" * 50)
    
    # Get optimized configuration
    config = create_optimized_webmanager_config(OptimizationLevel.LOW)
    
    print("Lightweight Configuration:")
    print(f"  📊 Data Collection: {config['data_collection_rate']}Hz")
    print(f"  📈 Chart Updates: {config['chart_update_rate']}Hz") 
    print(f"  💾 Max History: {config['max_history']} points")
    print(f"  📷 Camera Streaming: {'Enabled' if config['enable_camera_streaming'] else 'Disabled'}")
    print(f"  🗜️  Compression: {'Enabled' if config['enable_compression'] else 'Disabled'}")
    print(f"  💾 Memory Limit: {config['memory_limit_mb']}MB")
    print(f"  🔧 CPU Limit: {config['max_cpu_usage']}%")
    print()
    
    # Build command with optimized arguments
    cmd = [
        'python3', 'main.py',
        '--enable-webmanager',
        '--data-collection-rate', str(config['data_collection_rate']),
        '--max-history', str(config['max_history']),
        '--camera-quality', str(config['camera_quality']),
        '--webmanager-low-impact',
        '--log-level', 'WARNING'  # Reduce log verbosity
    ]
    
    # Add camera streaming control
    if not config['enable_camera_streaming']:
        cmd.append('--disable-camera-streaming')
    
    # Add compression if enabled
    if config['enable_compression']:
        cmd.append('--enable-compression')
    
    print("Starting WebManager with command:")
    print(f"  {' '.join(cmd)}")
    print()
    print("💡 Tips for further resource reduction:")
    print("  - Close unnecessary browser tabs")
    print("  - Use minimal optimization level if needed")
    print("  - Monitor resource usage with webmanager_resource_monitor.py")
    print()
    print("🌐 WebManager will be available at: http://localhost:8080")
    print("📊 Resource monitoring will be active")
    print()
    
    try:
        # Start the process
        process = subprocess.Popen(cmd)
        
        print("✅ WebManager started successfully!")
        print("   Press Ctrl+C to stop")
        print()
        
        # Wait for process to complete
        process.wait()
        
    except KeyboardInterrupt:
        print("\n🛑 Stopping WebManager...")
        if 'process' in locals():
            process.terminate()
            process.wait()
        print("✅ WebManager stopped")
        
    except subprocess.CalledProcessError as e:
        print(f"❌ Error starting WebManager: {e}")
        sys.exit(1)
        
    except FileNotFoundError:
        print("❌ Error: main.py not found in current directory")
        print("   Make sure you're running this script from the project root")
        sys.exit(1)


def show_optimization_levels():
    """Show available optimization levels."""
    
    print("Available WebManager Optimization Levels:")
    print("=" * 45)
    
    from webmanager_resource_optimization import WebManagerResourceOptimizer
    
    optimizer = WebManagerResourceOptimizer()
    
    for level in OptimizationLevel:
        config = optimizer.get_optimization_config(level)
        
        print(f"\n{level.value.upper()}:")
        print(f"  Data Collection: {config.data_collection_rate}Hz")
        print(f"  Chart Updates: {config.chart_update_rate}Hz")
        print(f"  Max History: {config.max_history}")
        print(f"  Camera: {'On' if config.camera_streaming_enabled else 'Off'}")
        print(f"  Memory Limit: {config.memory_limit_mb}MB")
        print(f"  CPU Limit: {config.max_cpu_usage}%")
        
        # Resource impact estimate
        if level == OptimizationLevel.MINIMAL:
            print("  💚 Impact: Minimal (recommended for low-end systems)")
        elif level == OptimizationLevel.LOW:
            print("  💛 Impact: Low (good balance for most systems)")
        elif level == OptimizationLevel.BALANCED:
            print("  🧡 Impact: Moderate (default configuration)")
        else:
            print("  ❤️  Impact: High (full features, high resources)")


def start_with_monitoring():
    """Start WebManager with resource monitoring."""
    
    print("🔍 Starting WebManager with Resource Monitoring")
    print("=" * 50)
    
    # Start main WebManager in background
    import threading
    import time
    
    def start_webmanager():
        main()
    
    # Start WebManager in separate thread
    webmanager_thread = threading.Thread(target=start_webmanager, daemon=True)
    webmanager_thread.start()
    
    # Give WebManager time to start
    time.sleep(5)
    
    # Start resource monitoring
    try:
        from webmanager_resource_monitor import ResourceMonitor
        
        print("📊 Starting resource monitor...")
        monitor = ResourceMonitor(check_interval=5.0)
        monitor.start_monitoring()
        
        print("✅ Resource monitoring active")
        print("   Monitor will check every 5 seconds")
        print("   Press Ctrl+C to stop both WebManager and monitoring")
        
        # Keep monitoring running
        while webmanager_thread.is_alive():
            time.sleep(1)
            
            # Show periodic resource summary
            if int(time.time()) % 30 == 0:  # Every 30 seconds
                summary = monitor.get_resource_summary()
                if 'current' in summary:
                    current = summary['current']
                    status = summary['status']
                    
                    status_emoji = {
                        'normal': '✅',
                        'warning': '⚠️',
                        'critical': '🚨'
                    }.get(status, '❓')
                    
                    print(f"{status_emoji} Resource Status: "
                          f"Memory: {current['memory_mb']}MB, "
                          f"CPU: {current['cpu_percent']}%, "
                          f"Status: {status}")
        
    except ImportError:
        print("⚠️  Resource monitoring not available (missing dependencies)")
        print("   WebManager will continue without monitoring")
        
        # Just wait for WebManager thread
        webmanager_thread.join()
    
    except KeyboardInterrupt:
        print("\n🛑 Stopping WebManager and monitoring...")


if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description="Lightweight WebManager Startup")
    parser.add_argument('--show-levels', action='store_true', 
                       help='Show available optimization levels')
    parser.add_argument('--with-monitoring', action='store_true',
                       help='Start with resource monitoring')
    
    args = parser.parse_args()
    
    if args.show_levels:
        show_optimization_levels()
    elif args.with_monitoring:
        start_with_monitoring()
    else:
        main()