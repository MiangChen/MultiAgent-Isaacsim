# WebManager Setup and Installation Guide

This guide provides detailed instructions for setting up and installing the Isaac Sim WebManager system.

## Prerequisites

### System Requirements

- **Operating System**: Linux (Ubuntu 20.04+ recommended), Windows 10/11, or macOS
- **Python**: 3.8 or higher (3.9+ recommended)
- **Isaac Sim**: 2023.1.0 or higher
- **Memory**: Minimum 8GB RAM (16GB+ recommended)
- **Network**: Available port for web server (default: 8080)

### Isaac Sim Environment

The WebManager requires Isaac Sim to be properly installed and configured. Make sure you can run Isaac Sim before proceeding.

**Linux/macOS:**
```bash
# Activate Isaac Sim Python environment
source ~/.local/share/ov/pkg/isaac_sim-*/python.sh
```

**Windows:**
```cmd
# Activate Isaac Sim Python environment
call "%USERPROFILE%\AppData\Local\ov\pkg\isaac_sim-*\python.bat"
```

## Installation

### 1. Install Python Dependencies

#### Option A: Install All Dependencies (Recommended)
```bash
# Install all project dependencies including WebManager
pip install -r requirements.txt
```

#### Option B: Install WebManager Dependencies Only
```bash
# Install only WebManager-specific dependencies
pip install -r webmanager/requirements.txt
```

#### Option C: Manual Installation
```bash
# Core web framework
pip install "fastapi>=0.104.0,<1.0.0"
pip install "uvicorn[standard]>=0.24.0,<1.0.0"
pip install "websockets>=12.0,<13.0"

# Data processing
pip install "numpy>=1.24.0,<2.0.0"
pip install "opencv-python>=4.8.0,<5.0.0"
pip install "psutil>=5.9.0,<6.0.0"

# Additional utilities
pip install "pydantic>=2.0.0,<3.0.0"
pip install "python-json-logger>=2.0.0,<3.0.0"
```

### 2. Verify Installation

Check if all dependencies are installed correctly:

```bash
# Using the startup script
python start_webmanager.py --check-deps

# Or using the shell script
./start_webmanager.sh --check-deps
```

### 3. Optional: ROS2 Integration

If you plan to use ROS2 integration, install ROS2 dependencies:

```bash
# Uncomment ROS2 lines in requirements.txt, then:
pip install rclpy>=3.3.0
pip install ros2cli>=0.18.0
pip install ros2topic>=0.18.0
pip install ros2node>=0.18.0
```

## Configuration

### 1. Basic Configuration

The WebManager uses the same configuration file as the main Isaac Sim application:

```yaml
# files/sim_cfg.yaml
webmanager:
  enabled: true
  host: "0.0.0.0"
  port: 8080
  data_collection_rate: 10.0
  max_history: 1000
  camera_streaming: true
  camera_quality: 80
```

### 2. Network Configuration

#### Firewall Settings

Make sure the WebManager port is accessible:

**Linux (UFW):**
```bash
sudo ufw allow 8080/tcp
```

**Linux (iptables):**
```bash
sudo iptables -A INPUT -p tcp --dport 8080 -j ACCEPT
```

**Windows:**
```cmd
# Add firewall rule for port 8080
netsh advfirewall firewall add rule name="Isaac Sim WebManager" dir=in action=allow protocol=TCP localport=8080
```

#### Network Access

- **Local access only**: Use `--host 127.0.0.1`
- **Network access**: Use `--host 0.0.0.0` (default)
- **Specific interface**: Use `--host <IP_ADDRESS>`

### 3. Security Considerations

For production deployments:

1. **Use HTTPS**: Configure a reverse proxy (nginx, Apache) with SSL
2. **Authentication**: Consider adding authentication middleware
3. **Firewall**: Restrict access to trusted networks only
4. **Updates**: Keep dependencies updated regularly

## Usage

### 1. Quick Start

```bash
# Start with default settings
python start_webmanager.py

# Or using the shell script
./start_webmanager.sh
```

### 2. Common Startup Options

```bash
# Custom host and port
python start_webmanager.py --host 192.168.1.100 --port 8081

# Disable camera streaming (reduces bandwidth)
python start_webmanager.py --disable-camera

# Debug mode
python start_webmanager.py --log-level DEBUG

# Simulation only (no web interface)
python start_webmanager.py --disable-webmanager
```

### 3. Advanced Configuration

```bash
# High-frequency data collection
python start_webmanager.py --collection-rate 30.0

# Large history buffer
python start_webmanager.py --max-history 5000

# Enable compression for slower networks
python start_webmanager.py --enable-compression

# Custom Isaac Sim configuration
python start_webmanager.py --config ./custom_sim_config.yaml
```

## Accessing the Web Interface

Once started, the WebManager web interface is available at:

- **Local access**: http://localhost:8080
- **Network access**: http://YOUR_IP_ADDRESS:8080

### Browser Requirements

- **Modern browser**: Chrome 90+, Firefox 88+, Safari 14+, Edge 90+
- **JavaScript enabled**: Required for interactive features
- **WebSocket support**: Required for real-time updates

## Troubleshooting

### Common Issues

#### 1. Port Already in Use
```
Error: [Errno 98] Address already in use
```

**Solution:**
```bash
# Find process using the port
sudo netstat -tlnp | grep :8080
# Or use a different port
python start_webmanager.py --port 8081
```

#### 2. Isaac Sim Environment Not Found
```
ImportError: No module named 'omni'
```

**Solution:**
```bash
# Make sure Isaac Sim environment is activated
source ~/.local/share/ov/pkg/isaac_sim-*/python.sh
```

#### 3. Missing Dependencies
```
ImportError: No module named 'fastapi'
```

**Solution:**
```bash
# Install missing dependencies
pip install -r requirements.txt
```

#### 4. WebSocket Connection Failed

**Check:**
- Firewall settings
- Network connectivity
- Browser console for error messages
- Server logs for connection issues

#### 5. Camera Streaming Issues

**Solutions:**
- Reduce camera quality: `--camera-quality 50`
- Disable camera streaming: `--disable-camera`
- Check Isaac Sim viewport configuration

### Debug Mode

Enable debug logging for detailed troubleshooting:

```bash
python start_webmanager.py --log-level DEBUG
```

Check log files:
- `isaac_sim_webmanager.log` - Main application log
- `webmanager_startup.log` - Startup script log

### Performance Optimization

#### For Slower Systems:
```bash
# Reduce data collection frequency
python start_webmanager.py --collection-rate 5.0

# Reduce history buffer
python start_webmanager.py --max-history 500

# Disable camera streaming
python start_webmanager.py --disable-camera
```

#### For Network Optimization:
```bash
# Enable compression
python start_webmanager.py --enable-compression

# Reduce camera quality
python start_webmanager.py --camera-quality 60
```

## Development Setup

### For Development and Testing

```bash
# Install development dependencies
pip install pytest>=7.0.0 pytest-asyncio>=0.21.0 pytest-mock>=3.10.0
pip install black>=23.0.0 flake8>=6.0.0 mypy>=1.0.0

# Run tests (when available)
pytest webmanager/tests/

# Code formatting
black webmanager/
flake8 webmanager/
```

### Frontend Development

The frontend uses CDN-loaded libraries by default. For local development:

```bash
# Install Node.js dependencies (optional)
npm install

# Serve static files locally
npm run serve
```

## Support and Documentation

### Log Files

- **Main log**: `isaac_sim_webmanager.log`
- **Startup log**: `webmanager_startup.log`
- **System logs**: Check system journal/event viewer

### Getting Help

1. Check log files for error messages
2. Verify all dependencies are installed
3. Ensure Isaac Sim environment is properly activated
4. Check network and firewall configuration
5. Review this documentation for common solutions

### Reporting Issues

When reporting issues, please include:

1. Operating system and version
2. Isaac Sim version
3. Python version
4. Complete error messages and stack traces
5. Log file contents (with sensitive information removed)
6. Steps to reproduce the issue

## Version Compatibility

| WebManager Version | Isaac Sim Version | Python Version |
|-------------------|-------------------|----------------|
| 1.0.0             | 2023.1.0+         | 3.8+           |
| 1.1.0             | 2023.1.1+         | 3.9+           |

## License

This WebManager system is part of the Isaac Sim project and follows the same licensing terms.