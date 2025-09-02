#!/bin/bash

# Isaac Sim WebManager Startup Script
# Provides easy startup with common configurations

set -e  # Exit on any error

# Default values
CONFIG_FILE="./files/sim_cfg.yaml"
WEB_HOST="0.0.0.0"
WEB_PORT="8080"
LOG_LEVEL="INFO"
ENABLE_WEBMANAGER=true
ENABLE_ROS=true

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to show usage
show_usage() {
    cat << EOF
Isaac Sim WebManager Startup Script

Usage: $0 [OPTIONS]

Options:
    -h, --help              Show this help message
    -c, --config FILE       Isaac Sim configuration file (default: ./files/sim_cfg.yaml)
    --host HOST             WebManager host address (default: 0.0.0.0)
    --port PORT             WebManager port (default: 8080)
    --log-level LEVEL       Log level: DEBUG, INFO, WARNING, ERROR, CRITICAL (default: INFO)
    --disable-webmanager    Disable WebManager web interface
    --disable-ros           Disable ROS integration
    --disable-camera        Disable camera streaming
    --camera-quality N      Camera JPEG quality 10-100 (default: 80)
    --collection-rate N     Data collection rate in Hz (default: 10.0)
    --max-history N         Maximum historical data points (default: 1000)
    --enable-compression    Enable WebSocket compression
    --check-deps            Check dependencies and exit
    --dry-run               Show command without executing

Examples:
    # Start with default settings
    $0

    # Start on specific host and port
    $0 --host 192.168.1.100 --port 8081

    # Start without WebManager (simulation only)
    $0 --disable-webmanager

    # Start with debug logging
    $0 --log-level DEBUG

    # Check dependencies
    $0 --check-deps

EOF
}

# Function to check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to check dependencies
check_dependencies() {
    print_info "Checking dependencies..."
    
    # Check Python
    if ! command_exists python3 && ! command_exists python; then
        print_error "Python not found. Please install Python 3.8 or higher."
        return 1
    fi
    
    # Check if main.py exists
    if [ ! -f "main.py" ]; then
        print_error "main.py not found in current directory"
        print_error "Please run this script from the Isaac Sim project root directory"
        return 1
    fi
    
    # Check Python packages
    python3 -c "
import sys
missing = []
packages = ['fastapi', 'uvicorn', 'websockets', 'numpy', 'cv2', 'psutil']
for pkg in packages:
    try:
        __import__(pkg)
    except ImportError:
        missing.append(pkg)

if missing:
    print('Missing packages:', ', '.join(missing))
    print('Install with: pip install -r requirements.txt')
    sys.exit(1)
else:
    print('All required packages are installed')
" 2>/dev/null || {
        print_error "Some Python dependencies are missing"
        print_error "Run: pip install -r requirements.txt"
        return 1
    }
    
    print_success "All dependencies are satisfied"
    return 0
}

# Function to check Isaac Sim environment
check_isaac_sim() {
    print_info "Checking Isaac Sim environment..."
    
    python3 -c "
try:
    import omni
    print('Isaac Sim environment is available')
except ImportError:
    print('Isaac Sim environment not detected')
    print('Make sure to activate Isaac Sim Python environment:')
    print('source ~/.local/share/ov/pkg/isaac_sim-*/python.sh')
    exit(1)
" 2>/dev/null || {
        print_warning "Isaac Sim environment not detected"
        print_warning "Make sure to activate Isaac Sim Python environment:"
        print_warning "source ~/.local/share/ov/pkg/isaac_sim-*/python.sh"
        return 1
    }
    
    return 0
}

# Parse command line arguments
ARGS=()
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_usage
            exit 0
            ;;
        -c|--config)
            CONFIG_FILE="$2"
            shift 2
            ;;
        --host)
            WEB_HOST="$2"
            shift 2
            ;;
        --port)
            WEB_PORT="$2"
            shift 2
            ;;
        --log-level)
            LOG_LEVEL="$2"
            shift 2
            ;;
        --disable-webmanager)
            ENABLE_WEBMANAGER=false
            shift
            ;;
        --disable-ros)
            ENABLE_ROS=false
            shift
            ;;
        --disable-camera)
            ARGS+=(--disable-camera-streaming)
            shift
            ;;
        --camera-quality)
            ARGS+=(--camera-quality "$2")
            shift 2
            ;;
        --collection-rate)
            ARGS+=(--data-collection-rate "$2")
            shift 2
            ;;
        --max-history)
            ARGS+=(--max-history "$2")
            shift 2
            ;;
        --enable-compression)
            ARGS+=(--enable-compression)
            shift
            ;;
        --check-deps)
            check_dependencies
            exit $?
            ;;
        --dry-run)
            DRY_RUN=true
            shift
            ;;
        *)
            print_error "Unknown option: $1"
            show_usage
            exit 1
            ;;
    esac
done

# Build command arguments
CMD_ARGS=(python3 main.py --config "$CONFIG_FILE" --log-level "$LOG_LEVEL")

# WebManager settings
if [ "$ENABLE_WEBMANAGER" = true ]; then
    CMD_ARGS+=(--enable-webmanager --web-host "$WEB_HOST" --web-port "$WEB_PORT")
else
    CMD_ARGS+=(--disable-webmanager)
fi

# ROS settings
if [ "$ENABLE_ROS" = false ]; then
    CMD_ARGS+=(--ros False)
fi

# Add additional arguments
CMD_ARGS+=("${ARGS[@]}")

# Show startup information
print_info "Isaac Sim WebManager Startup"
print_info "Configuration file: $CONFIG_FILE"

if [ "$ENABLE_WEBMANAGER" = true ]; then
    print_info "WebManager will be available at: http://$WEB_HOST:$WEB_PORT"
else
    print_info "WebManager is disabled"
fi

print_info "Log level: $LOG_LEVEL"

# Check dependencies unless it's a dry run
if [ "$DRY_RUN" != true ]; then
    if ! check_dependencies; then
        exit 1
    fi
    
    # Check Isaac Sim (warning only)
    check_isaac_sim || print_warning "Continuing without Isaac Sim environment verification"
fi

# Show command or execute
if [ "$DRY_RUN" = true ]; then
    print_info "Would execute: ${CMD_ARGS[*]}"
    exit 0
fi

print_info "Starting Isaac Sim WebManager..."
print_info "Command: ${CMD_ARGS[*]}"

# Create log directory if it doesn't exist
mkdir -p logs

# Execute the command
exec "${CMD_ARGS[@]}"