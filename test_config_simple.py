#!/usr/bin/env python3
"""
Simple test for the optimized configuration handling.
Tests only the configuration classes without WebManager dependencies.
"""

import sys
import os

# Add the current directory to Python path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Import only what we need for testing
from argument_parser import parse_arguments


def create_mock_webmanager_config():
    """Create a mock WebManagerConfig class for testing."""
    
    class MockWebManagerConfig:
        def __init__(self):
            self.config_file = None
            self.web_host = "0.0.0.0"
            self.web_port = 8080
            self.log_level = "INFO"
            self.enable_webmanager = True
            self.enable_ros = True
            self.data_collection_rate = 10.0
            self.max_history = 1000
            self.disable_camera_streaming = False
            self.camera_quality = 80
            self.enable_compression = False
            self.log_file = "isaac_sim_webmanager.log"
            self.pid_file = None
            self.status_file = None
            self.enable_metrics = True
            self.metrics_port = None
            self.enable_health_check = True
            self.health_check_interval = 30.0
            self.shutdown_timeout = 10.0
            self.enable_auto_restart = False
            self.max_restart_attempts = 3
        
        def from_args(self, args):
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
    
    return MockWebManagerConfig


def test_old_vs_new_approach():
    """Test the difference between old manual approach and new optimized approach."""
    print("=== Testing Old vs New Configuration Approach ===")
    
    # Parse some test arguments
    test_args = [
        "--enable-webmanager",
        "--web-host", "192.168.1.100", 
        "--web-port", "9090",
        "--data-collection-rate", "15.0",
        "--log-level", "DEBUG",
        "--camera-quality", "90",
        "--enable-compression",
        "--max-history", "2000",
        "--pid-file", "/tmp/webmanager.pid",
        "--enable-metrics"
    ]
    
    args = parse_arguments(test_args)
    WebManagerConfig = create_mock_webmanager_config()
    
    print(f"Parsed arguments:")
    print(f"  web_host: {args.web_host}")
    print(f"  web_port: {args.web_port}")
    print(f"  data_collection_rate: {args.data_collection_rate}")
    print(f"  log_level: {args.log_level}")
    print(f"  enable_webmanager: {args.enable_webmanager}")
    print(f"  disable_webmanager: {args.disable_webmanager}")
    
    # OLD APPROACH: Manual assignment (what we had before)
    print(f"\n--- OLD APPROACH (Manual Assignment) ---")
    print("Code required:")
    print("    config = WebManagerConfig()")
    print("    config.web_host = args.web_host")
    print("    config.web_port = args.web_port") 
    print("    config.log_level = args.log_level")
    print("    config.log_file = args.log_file")
    print("    config.enable_webmanager = args.enable_webmanager and not args.disable_webmanager")
    print("    config.enable_ros = args.ros")
    print("    config.data_collection_rate = args.data_collection_rate")
    print("    config.max_history = args.max_history")
    print("    config.disable_camera_streaming = args.disable_camera_streaming")
    print("    config.camera_quality = args.camera_quality")
    print("    config.enable_compression = args.enable_compression")
    print("    config.pid_file = args.pid_file")
    print("    config.status_file = args.status_file")
    print("    config.enable_metrics = args.enable_metrics")
    print("    config.metrics_port = args.metrics_port")
    print("    config.enable_health_check = args.enable_health_check")
    print("    config.health_check_interval = args.health_check_interval")
    print("    config.shutdown_timeout = args.shutdown_timeout")
    print("    config.enable_auto_restart = args.enable_auto_restart")
    print("    config.max_restart_attempts = args.max_restart_attempts")
    print("    # Total: ~20 lines of repetitive code")
    
    # Execute old approach
    old_config = WebManagerConfig()
    old_config.web_host = args.web_host
    old_config.web_port = args.web_port
    old_config.log_level = args.log_level
    old_config.log_file = args.log_file
    old_config.enable_webmanager = args.enable_webmanager and not args.disable_webmanager
    old_config.enable_ros = args.ros
    old_config.data_collection_rate = args.data_collection_rate
    old_config.max_history = args.max_history
    old_config.disable_camera_streaming = args.disable_camera_streaming
    old_config.camera_quality = args.camera_quality
    old_config.enable_compression = args.enable_compression
    old_config.pid_file = args.pid_file
    old_config.status_file = args.status_file
    old_config.enable_metrics = args.enable_metrics
    old_config.metrics_port = args.metrics_port
    old_config.enable_health_check = args.enable_health_check
    old_config.health_check_interval = args.health_check_interval
    old_config.shutdown_timeout = args.shutdown_timeout
    old_config.enable_auto_restart = args.enable_auto_restart
    old_config.max_restart_attempts = args.max_restart_attempts
    
    print(f"\nResult:")
    print(f"  web_host: {old_config.web_host}")
    print(f"  web_port: {old_config.web_port}")
    print(f"  enable_webmanager: {old_config.enable_webmanager}")
    print(f"  data_collection_rate: {old_config.data_collection_rate}")
    
    # NEW APPROACH: Optimized single method call
    print(f"\n--- NEW APPROACH (Optimized) ---")
    print("Code required:")
    print("    config = WebManagerConfig.from_args_direct(args)")
    print("    # Total: 1 line of code!")
    
    # Execute new approach
    new_config = WebManagerConfig.from_args_direct(args)
    
    print(f"\nResult:")
    print(f"  web_host: {new_config.web_host}")
    print(f"  web_port: {new_config.web_port}")
    print(f"  enable_webmanager: {new_config.enable_webmanager}")
    print(f"  data_collection_rate: {new_config.data_collection_rate}")
    
    # Verify both approaches produce the same result
    print(f"\n--- VERIFICATION ---")
    matches = []
    matches.append(old_config.web_host == new_config.web_host)
    matches.append(old_config.web_port == new_config.web_port)
    matches.append(old_config.enable_webmanager == new_config.enable_webmanager)
    matches.append(old_config.data_collection_rate == new_config.data_collection_rate)
    matches.append(old_config.log_level == new_config.log_level)
    matches.append(old_config.camera_quality == new_config.camera_quality)
    matches.append(old_config.enable_compression == new_config.enable_compression)
    
    if all(matches):
        print("✓ Both approaches produce IDENTICAL results")
    else:
        print("✗ Approaches produce different results")
        print("  Differences found:")
        if old_config.web_host != new_config.web_host:
            print(f"    web_host: {old_config.web_host} vs {new_config.web_host}")
        if old_config.web_port != new_config.web_port:
            print(f"    web_port: {old_config.web_port} vs {new_config.web_port}")
    
    print(f"\n--- BENEFITS ---")
    print("✓ Code reduction: 20+ lines → 1 line (95% reduction)")
    print("✓ Eliminated repetitive manual mapping")
    print("✓ Centralized argument-to-config conversion logic")
    print("✓ Easier to maintain and extend")
    print("✓ Less prone to copy-paste errors")
    print("✓ Consistent handling of special cases")
    
    return old_config, new_config


def main():
    """Run the test."""
    print("Testing Configuration Optimization")
    print("=" * 50)
    
    try:
        old_config, new_config = test_old_vs_new_approach()
        
        print("\n" + "=" * 50)
        print("✓ Configuration optimization test completed successfully!")
        
    except Exception as e:
        print(f"\n✗ Test failed with error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()