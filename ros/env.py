# test_ros2_import.py
import sys
import os

print("Python executable:", sys.executable)
print("Python version:", sys.version)
print("Python path:")
for path in sys.path:
    print(f"  {path}")

print("\nEnvironment variables:")
ros_vars = ['ROS_DISTRO', 'AMENT_PREFIX_PATH', 'ROS_VERSION', 'PYTHONPATH']
for var in ros_vars:
    print(f"  {var}: {os.environ.get(var, 'Not set')}")

try:
    import rclpy

    print(f"\nrclpy imported successfully from: {rclpy.__file__}")

    # 测试基本功能
    rclpy.init()
    print("rclpy.init() successful")
    rclpy.shutdown()
    print("rclpy.shutdown() successful")

except ImportError as e:
    print(f"\nFailed to import rclpy: {e}")
except Exception as e:
    print(f"\nError testing rclpy: {e}")
