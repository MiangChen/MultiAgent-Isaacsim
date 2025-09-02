# Isaac Sim WebManager - Installation Guide

## Overview

The Isaac Sim WebManager provides a web-based monitoring and control interface for Isaac Sim robotic simulations. This guide covers the installation and setup procedures for both the backend Python components and frontend web interface.

## Prerequisites

### System Requirements

- **Operating System**: Linux (Ubuntu 20.04+ recommended), Windows 10/11, or macOS
- **Python**: 3.8 or higher (3.9+ recommended)
- **Isaac Sim**: 2023.1.0 or higher
- **Memory**: 8GB RAM minimum, 16GB+ recommended
- **Network**: Open port 8080 for web interface (configurable)

### Isaac Sim Installation

Ensure Isaac Sim is properly installed and configured on your system. The WebManager integrates with Isaac Sim's Python environment.

## Installation Steps

### 1. Clone or Setup Project

If you haven't already, ensure you have the Isaac Sim project with WebManager components:

```bash
# Navigate to your Isaac Sim project directory
cd /path/to/your/isaac-sim-project
```

### 2. Python Environment Setup

#### Option A: Using Isaac Sim's Python Environment (Recommended)

```bash
# Activate Isaac Sim's Python environment
source ~/.local/share/ov/pkg/isaac_sim-*/python.sh

# Install WebManager dependencies
pip install -r requirements.txt
```

#### Option B: Using Virtual Environment

```bash
# Create virtual environment
python -m venv isaac_webmanager_env

# Activate virtual environment
# On Linux/macOS:
source isaac_webmanager_env/bin/activate
# On Windows:
isaac_webmanager_env\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

### 3. WebManager Specific Dependencies

Install WebManager-specific dependencies:

```bash
# Install WebManager dependencies
pip install -r webmanager/requirements.txt
```

### 4. Optional: ROS2 Integration

If you plan to use ROS2 integration features, install ROS2 dependencies:

```bash
# Uncomment ROS2 dependencies in requirements.txt
# Then install:
pip install rclpy ros2cli ros2topic ros2node

# Source ROS2 environment
source /opt/ros/humble/setup.bash  # Adjust for your ROS2 distribution
```

### 5. Frontend Dependencies (Optional)

The frontend uses CDN-hosted libraries by default. For local development:

```bash
# Install Node.js dependencies (optional)
npm install

# Or serve static files locally
python -m http.server 8000 --directory UI
```

## Configuration

### 1. Basic Configuration

The WebManager can be configured through command-line arguments or environment variables:

```bash
# Basic startup with default settings
python main.py --enable-webmanager

# Custom configuration
python main.py --enable-webmanager --web-port 8080 --web-host 0.0.0.0
```

### 2. Environment Variables

Set environment variables for configuration:

```bash
export WEBMANAGER_HOST=0.0.0.0
export WEBMANAGER_PORT=8080
export WEBMANAGER_LOG_LEVEL=INFO
```

### 3. Configuration Files

Create configuration files if needed:

```yaml
# webmanager_config.yaml
server:
  host: "0.0.0.0"
  port: 8080
  log_level: "INFO"

data_collection:
  rate_hz: 10.0
  max_history: 1000

features:
  enable_ros_integration: true
  enable_camera_streaming: true
  enable_performance_monitoring: true
```

## Verification

### 1. Test Installation

```bash
# Test Python dependencies
python -c "import fastapi, uvicorn, websockets; print('WebManager dependencies OK')"

# Test Isaac Sim integration
python -c "import omni; print('Isaac Sim integration OK')"
```

### 2. Start WebManager

```bash
# Start with WebManager enabled
python main.py --enable-webmanager --web-port 8080

# Check if server is running
curl http://localhost:8080/health
```

### 3. Access Web Interface

1. Open your web browser
2. Navigate to `http://localhost:8080`
3. Verify the dashboard loads and shows connection status

## Troubleshooting

### Common Issues

#### 1. Port Already in Use

```bash
# Check what's using the port
lsof -i :8080

# Use a different port
python main.py --enable-webmanager --web-port 8081
```

#### 2. Isaac Sim Python Environment Issues

```bash
# Ensure Isaac Sim Python environment is activated
which python
# Should point to Isaac Sim's Python installation

# Reinstall dependencies in Isaac Sim environment
pip install --force-reinstall -r requirements.txt
```

#### 3. WebSocket Connection Issues

- Check firewall settings
- Verify the correct host/port configuration
- Ensure no proxy is interfering with WebSocket connections

#### 4. Missing Dependencies

```bash
# Install missing dependencies
pip install --upgrade -r requirements.txt

# Check for conflicting versions
pip check
```

### Performance Issues

#### 1. High CPU Usage

- Reduce data collection rate: `--data-collection-rate 5`
- Disable unused features: `--disable-camera-streaming`

#### 2. Memory Usage

- Reduce history buffer size: `--max-history 500`
- Enable data compression: `--enable-compression`

#### 3. Network Bandwidth

- Reduce camera frame quality: `--camera-quality 60`
- Limit update frequency: `--update-rate 5`

## Development Setup

### 1. Development Dependencies

```bash
# Install development dependencies
pip install pytest pytest-asyncio pytest-mock black flake8

# Install pre-commit hooks (optional)
pip install pre-commit
pre-commit install
```

### 2. Running Tests

```bash
# Run all tests
pytest

# Run specific test categories
pytest tests/test_webmanager.py
pytest tests/test_websocket.py
```

### 3. Code Formatting

```bash
# Format code
black webmanager/
black main.py

# Check code style
flake8 webmanager/
```

## Production Deployment

### 1. Security Considerations

- Change default host from `0.0.0.0` to specific IP
- Use HTTPS in production (configure reverse proxy)
- Implement authentication if needed
- Configure firewall rules

### 2. Performance Optimization

- Use production ASGI server (Gunicorn + Uvicorn)
- Enable compression and caching
- Configure appropriate worker processes
- Monitor resource usage

### 3. Monitoring and Logging

- Configure structured logging
- Set up health checks
- Monitor WebSocket connections
- Track performance metrics

## Support

### Documentation

- Check the main README.md for usage instructions
- Review the design document in `.kiro/specs/webmanager/design.md`
- See requirements document in `.kiro/specs/webmanager/requirements.md`

### Debugging

- Enable debug logging: `--log-level DEBUG`
- Check WebManager logs in the console output
- Use browser developer tools for frontend debugging
- Monitor WebSocket messages in browser Network tab

### Getting Help

1. Check the troubleshooting section above
2. Review Isaac Sim documentation for integration issues
3. Check GitHub issues for known problems
4. Contact the development team for support

## Version Compatibility

| Component | Minimum Version | Recommended Version |
|-----------|----------------|-------------------|
| Python | 3.8 | 3.9+ |
| Isaac Sim | 2023.1.0 | Latest |
| FastAPI | 0.104.0 | Latest |
| Vue.js | 3.0 | 3.3+ |
| Node.js | 16.0 | 18+ (for dev) |

## License

This project is licensed under the MIT License. See LICENSE file for details.