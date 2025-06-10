# 基础消息接口
from rclpy.node import Node
import rclpy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import json


class BaseNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.node_name = node_name
        self.shared_data = {}

        # 创建发布者和订阅者
        self.data_publisher = self.create_publisher(String, f'/{node_name}/data', 10)
        self.query_publisher = self.create_publisher(String, '/query_request', 10)
        self.response_publisher = self.create_publisher(String, '/query_response', 10)

        # 订阅其他节点的数据
        self.create_subscription(String, '/query_request', self.handle_query, 10)
        self.create_subscription(String, '/query_response', self.handle_response, 10)

        self.pending_queries = {}

    def publish_data(self, key, value):
        """发布数据"""
        msg = String()
        data = {
            'node': self.node_name,
            'key': key,
            'value': value,
            'timestamp': self.get_clock().now().to_msg()
        }
        msg.data = json.dumps(data)
        self.data_publisher.publish(msg)

    def query_node_data(self, target_node, key, callback):
        """查询其他节点的数据"""
        query_id = f"{self.node_name}_{target_node}_{key}_{self.get_clock().now().nanoseconds}"

        msg = String()
        query = {
            'query_id': query_id,
            'from_node': self.node_name,
            'target_node': target_node,
            'key': key
        }
        msg.data = json.dumps(query)

        self.pending_queries[query_id] = callback
        self.query_publisher.publish(msg)

    def handle_query(self, msg):
        """处理查询请求"""
        query = json.loads(msg.data)
        if query['target_node'] == self.node_name:
            response_msg = String()
            response = {
                'query_id': query['query_id'],
                'from_node': self.node_name,
                'to_node': query['from_node'],
                'key': query['key'],
                'value': self.shared_data.get(query['key'], None)
            }
            response_msg.data = json.dumps(response)
            self.response_publisher.publish(response_msg)

    def handle_response(self, msg):
        """处理查询响应"""
        response = json.loads(msg.data)
        query_id = response['query_id']
        if query_id in self.pending_queries:
            callback = self.pending_queries.pop(query_id)
            callback(response['value'])


# Swarm 节点
class SwarmNode(BaseNode):
    def __init__(self):
        super().__init__('swarm')
        self.robot_positions = {}
        self.formation_config = {}

        # 定时发布数据
        self.create_timer(1.0, self.update_swarm_data)

    def update_swarm_data(self):
        self.shared_data['robot_count'] = len(self.robot_positions)
        self.shared_data['formation_type'] = 'triangle'
        self.publish_data('robot_count', len(self.robot_positions))
        self.publish_data('formation_type', 'triangle')

    def get_robot_positions(self):
        return self.robot_positions

    def request_map_info(self):
        """请求地图信息"""

        def handle_map_response(value):
            self.get_logger().info(f"Received map size: {value}")

        self.query_node_data('map', 'map_size', handle_map_response)


# Map 节点
class MapNode(BaseNode):
    def __init__(self):
        super().__init__('map')
        self.map_data = None
        self.obstacles = []

        self.create_timer(2.0, self.update_map_data)

    def update_map_data(self):
        self.shared_data['map_size'] = [100, 100]
        self.shared_data['obstacle_count'] = len(self.obstacles)
        self.publish_data('map_size', [100, 100])

    def get_safe_zones(self):
        return [(10, 10), (20, 20), (30, 30)]


# Planning 节点
class PlanningNode(BaseNode):
    def __init__(self):
        super().__init__('planning')
        self.current_path = []
        self.goals = []

        self.create_timer(1.5, self.update_planning_data)

    def update_planning_data(self):
        self.shared_data['path_length'] = len(self.current_path)
        self.shared_data['goals_count'] = len(self.goals)
        self.publish_data('path_length', len(self.current_path))

    def request_swarm_and_map_info(self):
        """同时请求群体和地图信息"""

        def handle_robot_count(value):
            self.get_logger().info(f"Planning for {value} robots")

        def handle_map_size(value):
            self.get_logger().info(f"Planning on map size: {value}")

        self.query_node_data('swarm', 'robot_count', handle_robot_count)
        self.query_node_data('map', 'map_size', handle_map_size)

if __name__ == '__main__':
    rclpy.init()  # ROS2 Python接口初始化
    node_swarm = SwarmNode()
    rclpy.spin(node_swarm)                                 # 循环等待ROS2退出
    node_swarm.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口

