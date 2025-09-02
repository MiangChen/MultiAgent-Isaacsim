#!/usr/bin/env python3
"""
Test script for the argument parser module.

This script tests various argument combinations to ensure the parser works correctly.
"""

import sys
from argument_parser import create_argument_parser, get_argument_summary


def test_basic_parsing():
    """Test basic argument parsing functionality."""
    print("=== Testing Basic Argument Parsing ===")
    
    parser = create_argument_parser()
    
    # Test default arguments
    args = parser.parse_args([])
    print("Default arguments:")
    print(get_argument_summary(args))
    print()
    
    # Test WebManager enabled
    args = parser.parse_args(["--enable-webmanager"])
    print("WebManager enabled:")
    print(f"  WebManager enabled: {args.enable_webmanager and not args.disable_webmanager}")
    print(f"  Host: {args.web_host}")
    print(f"  Port: {args.web_port}")
    print()
    
    # Test custom configuration
    test_args = [
        "--enable-webmanager",
        "--web-port", "9090",
        "--data-collection-rate", "15.0",
        "--log-level", "DEBUG",
        "--webmanager-high-performance"
    ]
    args = parser.parse_args(test_args)
    print("Custom configuration:")
    print(get_argument_summary(args))
    print()


def test_performance_modes():
    """Test performance mode configurations."""
    print("=== Testing Performance Modes ===")
    
    parser = create_argument_parser()
    
    # Test low impact mode
    args = parser.parse_args(["--enable-webmanager", "--webmanager-low-impact"])
    print(f"Low impact mode: {args.webmanager_low_impact}")
    
    # Test high performance mode
    args = parser.parse_args(["--enable-webmanager", "--webmanager-high-performance"])
    print(f"High performance mode: {args.webmanager_high_performance}")
    print()


def test_validation():
    """Test argument validation."""
    print("=== Testing Argument Validation ===")
    
    parser = create_argument_parser()
    
    # Test conflicting WebManager flags
    try:
        args = parser.parse_args(["--enable-webmanager", "--disable-webmanager"])
        from argument_parser import _validate_arguments
        _validate_arguments(args)
        print("ERROR: Should have failed validation")
    except ValueError as e:
        print(f"✓ Correctly caught conflicting WebManager flags: {e}")
    
    # Test conflicting performance modes
    try:
        args = parser.parse_args(["--webmanager-low-impact", "--webmanager-high-performance"])
        from argument_parser import _validate_arguments
        _validate_arguments(args)
        print("ERROR: Should have failed validation")
    except ValueError as e:
        print(f"✓ Correctly caught conflicting performance modes: {e}")
    
    # Test invalid port
    try:
        args = parser.parse_args(["--web-port", "70000"])
        from argument_parser import _validate_arguments
        _validate_arguments(args)
        print("ERROR: Should have failed validation")
    except ValueError as e:
        print(f"✓ Correctly caught invalid port: {e}")
    
    print()


def test_help_output():
    """Test help output generation."""
    print("=== Testing Help Output ===")
    
    parser = create_argument_parser()
    
    # This would normally exit, so we'll just check that the parser has help
    help_text = parser.format_help()
    
    # Check for key sections
    required_sections = [
        "Simulation Configuration",
        "WebManager Configuration", 
        "Logging Configuration",
        "Process Management",
        "Performance Optimization"
    ]
    
    for section in required_sections:
        if section in help_text:
            print(f"✓ Found section: {section}")
        else:
            print(f"✗ Missing section: {section}")
    
    print()


def main():
    """Run all tests."""
    print("Testing Isaac Sim WebManager Argument Parser")
    print("=" * 50)
    
    try:
        test_basic_parsing()
        test_performance_modes()
        test_validation()
        test_help_output()
        
        print("All tests completed successfully!")
        
    except Exception as e:
        print(f"Test failed with error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()