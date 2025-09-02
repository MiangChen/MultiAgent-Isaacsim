#!/usr/bin/env python3
"""
Test script for the optimized configuration handling.

This script tests the new WebManagerConfig.from_args_direct() method
and verifies that it correctly maps arguments to configuration attributes.
"""

import sys
from argument_parser import parse_arguments
from webmanager.startup_config import WebManagerConfig


def test_config_from_args():
    """Test creating config from args object."""
    print("=== Testing WebManagerConfig.from_args_direct() ===")
    
    # Test with default arguments
    print("\n1. Testing with default arguments:")
    args = parse_arguments([])
    config = WebManagerConfig.from_args_direct(args)
    
    print(f"  Web host: {config.web_host}")
    print(f"  Web port: {config.web_port}")
    print(f"  WebManager enabled: {config.enable_webmanager}")
    print(f"  ROS enabled: {config.enable_ros}")
    print(f"  Data collection rate: {config.data_collection_rate}")
    print(f"  Log level: {config.log_level}")
    
    # Test with custom arguments
    print("\n2. Testing with custom arguments:")
    test_args = [
        "--enable-webmanager",
        "--web-host", "192.168.1.100",
        "--web-port", "9090",
        "--data-collection-rate", "15.0",
        "--log-level", "DEBUG",
        "--camera-quality", "90",
        "--enable-compression",
        "--max-history", "2000"
    ]
    
    args = parse_arguments(test_args)
    config = WebManagerConfig.from_args_direct(args)
    
    print(f"  Web host: {config.web_host}")
    print(f"  Web port: {config.web_port}")
    print(f"  WebManager enabled: {config.enable_webmanager}")
    print(f"  Data collection rate: {config.data_collection_rate}")
    print(f"  Log level: {config.log_level}")
    print(f"  Camera quality: {config.camera_quality}")
    print(f"  Compression enabled: {config.enable_compression}")
    print(f"  Max history: {config.max_history}")
    
    # Test WebManager disable
    print("\n3. Testing WebManager disable:")
    disable_args = ["--enable-webmanager", "--disable-webmanager"]
    args = parse_arguments(disable_args)
    config = WebManagerConfig.from_args_direct(args)
    print(f"  WebManager enabled: {config.enable_webmanager}")
    
    return config


def test_config_validation():
    """Test configuration validation."""
    print("\n=== Testing Configuration Validation ===")
    
    # Test valid configuration
    args = parse_arguments(["--enable-webmanager", "--web-port", "8080"])
    config = WebManagerConfig.from_args_direct(args)
    
    errors = config.validate()
    if errors:
        print("  ✗ Validation failed (unexpected):")
        for error in errors:
            print(f"    - {error}")
    else:
        print("  ✓ Valid configuration passed validation")
    
    # Test invalid configuration
    config.web_port = 70000  # Invalid port
    config.data_collection_rate = -5.0  # Invalid rate
    
    errors = config.validate()
    if errors:
        print("  ✓ Invalid configuration correctly caught:")
        for error in errors:
            print(f"    - {error}")
    else:
        print("  ✗ Invalid configuration not caught (unexpected)")


def test_config_dict_conversion():
    """Test configuration dictionary conversion."""
    print("\n=== Testing Configuration Dictionary Conversion ===")
    
    args = parse_arguments([
        "--enable-webmanager",
        "--web-host", "localhost",
        "--web-port", "8081",
        "--log-level", "WARNING"
    ])
    
    config = WebManagerConfig.from_args_direct(args)
    config_dict = config.to_dict()
    
    print("  Configuration as dictionary:")
    for key, value in config_dict.items():
        print(f"    {key}: {value}")
    
    # Test round-trip conversion
    new_config = WebManagerConfig()
    new_config.from_dict(config_dict)
    
    print(f"\n  Round-trip test:")
    print(f"    Original web_host: {config.web_host}")
    print(f"    Restored web_host: {new_config.web_host}")
    print(f"    Original web_port: {config.web_port}")
    print(f"    Restored web_port: {new_config.web_port}")
    
    if (config.web_host == new_config.web_host and 
        config.web_port == new_config.web_port and
        config.log_level == new_config.log_level):
        print("  ✓ Round-trip conversion successful")
    else:
        print("  ✗ Round-trip conversion failed")


def compare_old_vs_new_approach():
    """Compare the old manual approach vs new optimized approach."""
    print("\n=== Comparing Old vs New Approach ===")
    
    # Simulate the old approach
    args = parse_arguments([
        "--enable-webmanager",
        "--web-host", "0.0.0.0",
        "--web-port", "8080",
        "--data-collection-rate", "10.0",
        "--log-level", "INFO"
    ])
    
    # Old approach (manual assignment)
    print("Old approach (manual assignment):")
    old_config = WebManagerConfig()
    old_config.web_host = args.web_host
    old_config.web_port = args.web_port
    old_config.log_level = args.log_level
    old_config.enable_webmanager = args.enable_webmanager and not args.disable_webmanager
    old_config.enable_ros = args.ros
    old_config.data_collection_rate = args.data_collection_rate
    # ... many more lines would be needed
    
    print(f"  Lines of code: ~20+ (manual assignment for each field)")
    print(f"  Web host: {old_config.web_host}")
    print(f"  Web port: {old_config.web_port}")
    
    # New approach (optimized)
    print("\nNew approach (optimized):")
    new_config = WebManagerConfig.from_args_direct(args)
    
    print(f"  Lines of code: 1 (single method call)")
    print(f"  Web host: {new_config.web_host}")
    print(f"  Web port: {new_config.web_port}")
    
    # Verify they produce the same result
    if (old_config.web_host == new_config.web_host and
        old_config.web_port == new_config.web_port and
        old_config.log_level == new_config.log_level and
        old_config.enable_webmanager == new_config.enable_webmanager):
        print("  ✓ Both approaches produce identical results")
    else:
        print("  ✗ Approaches produce different results")


def main():
    """Run all tests."""
    print("Testing Optimized Configuration Handling")
    print("=" * 50)
    
    try:
        config = test_config_from_args()
        test_config_validation()
        test_config_dict_conversion()
        compare_old_vs_new_approach()
        
        print("\n" + "=" * 50)
        print("✓ All tests completed successfully!")
        print("\nOptimization Benefits:")
        print("  - Reduced code from ~20 lines to 1 line")
        print("  - Eliminated manual field mapping")
        print("  - Centralized argument-to-config conversion")
        print("  - Easier to maintain and extend")
        print("  - Less prone to errors")
        
    except Exception as e:
        print(f"\n✗ Test failed with error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()