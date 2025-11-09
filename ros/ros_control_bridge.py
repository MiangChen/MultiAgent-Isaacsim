"""ROS Control Bridge - Application layer bridge from ROS to simulation"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from simulation.control import RobotControl


class RosControlBridge(Node):
    """Bridge ROS cmd_vel to simulation Control objects"""
    
    def __init__(self, robot):
        super().__init__(f'{robot.namespace}_control_bridge')
        self.robot = robot
        
        # Subscribe to /<namespace>/cmd_vel
        self.create_subscription(
            Twist,
            f'/{robot.namespace}/cmd_vel',
            self._cmd_vel_callback,
            10
        )
    
    def _cmd_vel_callback(self, msg: Twist):
        # ROS Twist -> Control object -> robot.apply_control()
        control = RobotControl()
        control.linear_velocity = [msg.linear.x, msg.linear.y, msg.linear.z]
        control.angular_velocity = [msg.angular.x, msg.angular.y, msg.angular.z]
        self.robot.apply_control(control)


class RosControlBridgeManager:
    """Manage ROS bridges for multiple robots"""
    
    def __init__(self):
        self.bridges = {}
        self.executor = None
    
    def add_robots(self, robots):
        for robot in robots:
            if robot.namespace not in self.bridges:
                self.bridges[robot.namespace] = RosControlBridge(robot)
    
    def start(self):
        if not self.bridges:
            return
        
        from rclpy.executors import MultiThreadedExecutor
        import threading
        
        self.executor = MultiThreadedExecutor()
        for bridge in self.bridges.values():
            self.executor.add_node(bridge)
        
        threading.Thread(target=self.executor.spin, daemon=True).start()
    
    def stop(self):
        if self.executor:
            self.executor.shutdown()
        for bridge in self.bridges.values():
            bridge.destroy_node()
        self.bridges.clear()



