import networkx as nx
from unified_planning.plans.partial_order_plan import PartialOrderPlan
from unified_planning.plans.plan import ActionInstance
from typing import Dict, List
import graphviz


class DirectedActionGraph:
    """
    将 PartialOrderPlan 转换为有向无环图（DAG）。

    - 节点: ActionInstance
    - 边: 若 u < v，则添加 u → v
    """

    def __init__(self, po_plan: PartialOrderPlan):
        if not isinstance(po_plan, PartialOrderPlan):
            raise TypeError("po_plan 必须是 PartialOrderPlan 实例")
        self._po_plan = po_plan
        self.graph: nx.DiGraph = self._build_graph()

    def _build_graph(self) -> nx.DiGraph:
        return self._po_plan._graph.copy()

    def adjacency_list(self) -> Dict[str, List[str]]:
        adj: Dict[str, List[str]] = {}
        for node in self.graph.nodes:
            key = str(node)
            adj[key] = sorted(str(succ) for succ in self.graph.successors(node))
        return adj

    def successors(self, action: ActionInstance) -> List[ActionInstance]:
        if action not in self.graph:
            raise ValueError("该 ActionInstance 不在图中")
        return list(self.graph.successors(action))

    def predecessors(self, action: ActionInstance) -> List[ActionInstance]:
        if action not in self.graph:
            raise ValueError("该 ActionInstance 不在图中")
        return list(self.graph.predecessors(action))

    def topological_sort(self) -> List[ActionInstance]:
        """
        返回拓扑排序的 ActionInstance 列表；若图中有环则抛出 networkx.NetworkXUnfeasible。
        """
        return list(nx.topological_sort(self.graph))

    def visualize(self, filename: str = "dag", format: str = "png") -> None:
        """
        可视化有向图，并保存为图片（默认为 .png 格式）。
        - filename: 不带后缀的文件名
        - format: 支持 'png', 'pdf', 'svg' 等
        """
        dot = graphviz.Digraph(format=format)
        for node in self.graph.nodes:
            dot.node(str(node))
        for u, v in self.graph.edges:
            dot.edge(str(u), str(v))
        dot.render(filename, view=False)
        print(f"图已保存为 {filename}.{format}")

    def __str__(self) -> str:
        return str(self.adjacency_list())

    def pretty_print(self, indent: int = 2) -> None:
        pad = " " * indent
        print("DirectedActionGraph:")
        for node, nbrs in self.adjacency_list().items():
            nbrs_str = ", ".join(nbrs) if nbrs else "∅"
            print(f"{pad}{node}  -->  {nbrs_str}")
