#!/usr/bin/env python3
"""
Simplified Isaac Sim WebManager Startup Script

This script demonstrates how to use the new argument parser module
to start Isaac Sim with various configurations.
"""

import sys
import subprocess
from argument_parser import create_argument_parser, get_argument_summary


def main():
    """Main startup script with argument parsing."""
    
    # Create parser and parse arguments
    parser = create_argument_parser()
    args = parser.parse_args()
    
    # Display configuration summary
    print("Isaac Sim WebManager Startup")
    print("=" * 40)
    print(get_argument_summary(args))
    print()
    
    # Check if this is just a configuration check
    if len(sys.argv) == 1:
        print("No arguments provided. Use --help to see available options.")
        print("\nQuick start examples:")
        print("  # Enable WebManager with default settings")
        print("  python start_isaac_sim.py --enable-webmanager")
        print()
        print("  # Low impact mode for Isaac Sim performance")
        print("  python start_isaac_sim.py --enable-webmanager --webmanager-low-impact")
        print()
        print("  # High performance WebManager")
        print("  python start_isaac_sim.py --enable-webmanager --webmanager-high-performance --data-collection-rate 20.0")
        return
    
    # Determine if WebManager should be enabled
    webmanager_enabled = args.enable_webmanager and not args.disable_webmanager
    
    if webmanager_enabled:
        print(f"Starting Isaac Sim with WebManager on http://{args.web_host}:{args.web_port}")
        
        # Performance recommendations
        if args.webmanager_high_performance:
            print("⚠️  High performance mode enabled - may impact Isaac Sim performance")
        elif args.webmanager_low_impact:
            print("✓ Low impact mode enabled - optimized for Isaac Sim performance")
        
        if args.data_collection_rate > 15.0:
            print("⚠️  High data collection rate - consider reducing if Isaac Sim performance is affected")
    else:
        print("Starting Isaac Sim without WebManager")
    
    print()
    print("Starting main.py with parsed arguments...")
    
    # Build command to execute main.py with the same arguments
    cmd = ["python3", "main.py"] + sys.argv[1:]
    
    try:
        # Execute main.py with the arguments
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error starting Isaac Sim: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nStartup interrupted by user")
        sys.exit(0)


if __name__ == "__main__":
    main()