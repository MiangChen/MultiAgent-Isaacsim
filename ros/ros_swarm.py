# 基础消息接口
from rclpy.node import Node
import rclpy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
import json
from pxr import Usd, Tf, UsdGeom
import omni.usd
from geometry_msgs.msg import Transform as RosTransform
from .msg import PrimTransform, SceneModifications, Plan, SkillInfo
from typing import Dict, Any, List


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

class SceneMonitorNode(BaseNode):
    def __init__(self):
        super().__init__('SceneMonitor')
        self.modification_publisher = self.create_publisher(String, 'SceneModification', 10)

        self._stage = omni.usd.get_context().get_stage()
        self._prev_prim_paths = set(self._stage.GetPrimPaths())
        self._prev_transforms = {
            path: self._get_local_xform(path)
            for path in self._prev_prim_paths
        }

        self._key = Tf.Notice.Register(
            Usd.Notice.ObjectsChanged,
            self.on_usd_objects_changed,
            self._stage
        )
    def _get_local_xform(self, path):
        """返回一个 Prim 在本地的变换矩阵（Matrix4d），找不到时返回 None。"""
        prim = self._stage.GetPrimAtPath(path)
        if not prim:
            return None
        xformable = UsdGeom.Xformable(prim)
        ok, mat = xformable.GetLocalTransformation()
        return mat if ok else None

    def _mat_to_ros(self, mat):
        ros_t = RosTransform()
        if mat is None:
            return ros_t
        trans = mat.ExtractTranslation()
        ros_t.translation.x = trans[0]
        ros_t.translation.y = trans[1]
        ros_t.translation.z = trans[2]
        rot_mat = mat.ExtractRotationMatrix()
        quat = Gf.Quatd(rot_mat)
        ros_t.rotation.w = quat.real
        ros_t.rotation.x = quat.imag[0]
        ros_t.rotation.y = quat.imag[1]
        ros_t.rotation.z = quat.imag[2]
        return ros_t

    def _matrices_differ(self, m1, m2, tol=1e-6):
        """简单对比两矩阵是否不同。"""
        if m1 is None or m2 is None:
            return False
        return not m1.AlmostEqual(m2, tol)

    def on_usd_objects_changed(self, notice, sender_stage):
        # 结构性变更 vs 属性变更
        resynced = set(notice.GetResyncedPaths())
        info_changed = set(notice.GetChangedInfoOnlyPaths())

        curr_paths = set(self._stage.GetPrimPaths())
        curr_xforms = {p: self._get_local_xform(p) for p in curr_paths}

        added   = [p for p in resynced if p in curr_paths and p not in self._prev_paths]
        deleted = [p for p in resynced if p not in curr_paths and p in self._prev_paths]
        xform_changed = [
            p for p in info_changed
            if p in curr_xforms and
               p in self._prev_xforms and
               not curr_xforms[p].AlmostEqual(self._prev_xforms[p], 1e-6)
        ]

        # 构建消息
        msg = SceneModifications()
        for p in added:
            item = PrimTransform()
            item.change_type = PrimTransform.ADD
            item.prim_path = p
            item.transform = self._mat_to_ros(curr_xforms.get(p))
            msg.modifications.append(item)
        for p in deleted:
            item = PrimTransform()
            item.change_type = PrimTransform.DELETE
            item.prim_path = p
            # 对于删除项，可以填入上一次快照中的 transform
            item.transform = self._mat_to_ros(self._prev_xforms.get(p))
            msg.modifications.append(item)
        for p in xform_changed:
            item = PrimTransform()
            item.change_type = PrimTransform.TRANSFORM_CHANGED
            item.prim_path = p
            item.transform = self._mat_to_ros(curr_xforms.get(p))
            msg.modifications.append(item)

        # 发布并更新快照
        self.pub.publish(msg)
        self._prev_paths = curr_paths
        self._prev_xforms = curr_xforms

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


"""
if __name__ == '__main__':
    rclpy.init()  # ROS2 Python接口初始化
    node_swarm = SwarmNode()
    node_monitor = SceneMonitorNode()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node_monitor)
    executor.add_node(node_swarm)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # 优雅清理
        node_swarm.destroy_node()
        node_monitor.destroy_node()
        rclpy.shutdown()
"""