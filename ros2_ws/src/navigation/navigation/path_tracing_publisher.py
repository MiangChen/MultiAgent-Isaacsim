import rclpy
from rclpy.node import Node

# Import your custom message type
# NOTE: This import will only work *after* you have built the package once.
# from navigation.msg import PathTracingCmd
from message_interface.msg import *
print("可用的消息类型：", dir())


class PathTracingPublisher(Node):

    def __init__(self):
        super().__init__('path_tracing_publisher')
        # Create a publisher for the PathTracingCmd message type on the /path_tracing topic
        # QoS history depth is set to 10
        self.publisher_ = self.create_publisher(PathTracingCmd, '/path_tracing', 10)
        timer_period = 1/60  # seconds - publish 60hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.current_index = 0
        self.path_length = 10  # Example path length

    def timer_callback(self):
        msg = PathTracingCmd()

        # Populate the message fields
        msg.index = self.current_index
        msg.velocity = 1.0  # Example velocity
        msg.omega = 0.0  # Example angular velocity

        # Determine if the path is complete
        if self.current_index < self.path_length - 1:
            msg.complete = False
        else:
            msg.complete = True  # Simulate completion after reaching the last index

        # Publish the message
        self.publisher_.publish(msg)

        # Log the published message
        self.get_logger().info(
            f'Publishing PathTracingCmd: index={msg.index}, vel={msg.velocity:.2f}, omega={msg.omega:.2f}, complete={msg.complete}')

        # Update the index for the next message (loop for demonstration)
        self.current_index = (self.current_index + 1) % self.path_length
        # If you wanted to stop after completion, you might add:
        # if msg.complete:
        #    self.get_logger().info('Path complete, stopping publisher.')
        #    self.timer.cancel() # Stop the timer


def main(args=None):
    rclpy.init(args=args)
    print("初始化成功")

    path_tracing_publisher = PathTracingPublisher()
    print("path tracing publisher 实例化成功")
    rclpy.spin(path_tracing_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    path_tracing_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

