from log.log_manager import LogManager

from physics_engine.omni_utils import og

import rclpy
from rclpy.node import Node

from rosgraph_msgs.msg import Clock

logger = LogManager().get_logger(__name__)


class NodeTime(Node):
    def __init__(self, world_context):
        super().__init__("isaacsim_simulation_time_publisher")

        self._world = world_context

        self.clock_publisher = self.create_publisher(Clock, "/isaacsim_simulation_clock", 10)

        self._world.add_physics_callback("publish_sim_time", self.publish_time_callback)

        self.get_logger().info("ROS2 Clock Publisher has been started.")

    def publish_time_callback(self, step_size: float):
        sim_time_seconds = self._world.current_time

        if sim_time_seconds is None:
            return


        seconds = int(sim_time_seconds)
        nanoseconds = int((sim_time_seconds - seconds) * 1e9)

        clock_msg = Clock()
        clock_msg.clock.sec = seconds
        clock_msg.clock.nanosec = nanoseconds

        self.clock_publisher.publish(clock_msg)

def create_clock_graph():
    """Sets up the OmniGraph for publishing simulation time."""

    # Setup graph for clock publishing
    clock_topic = "/isaacsim_simulation_clock"
    (clock_graph_handle, _, _, _) = og.Controller.edit(
        {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("PublishClock", "isaacsim.ros1.bridge.ROS1PublishClock"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("PublishClock.inputs:topicName", clock_topic),
            ],
        },
    )

    # Run the clock graph once to generate ROS clock publisher
    og.Controller.evaluate_sync(clock_graph_handle)
