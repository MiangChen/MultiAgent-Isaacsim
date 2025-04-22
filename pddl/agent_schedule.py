import json
from typing import Dict, List

from matplotlib import pyplot as plt
from unified_planning.model.multi_agent import MultiAgentProblem
from unified_planning.plans.plan import ActionInstance
from pddl.plan import DirectedActionGraph


class ActionSchedule:
    def __init__(self, dag: DirectedActionGraph, problem: MultiAgentProblem):
        self.dag = dag
        self.problem = problem
        self.plan_steps = self._build_structured_schedule()

    def _extract_agent_name(self, action_instance: ActionInstance) -> str:
        """
        从 ActionInstance 的字符串中提取 agent 名。
        例: 'robot2.navigate-to(depot, place1)' → 'robot2'
        """
        action_str = str(action_instance)
        if '.' in action_str:
            return action_str.split('.', 1)[0]
        return "global"

    def _build_structured_schedule(self) -> Dict[str, Dict[str, Dict[str, Dict[str, str]]]]:
        """
        分层拓扑排序：允许 step 内多个 agent 并行执行。
        """
        graph = self.dag.graph.copy()
        in_degree = {node: graph.in_degree(node) for node in graph.nodes}
        structured = {}
        step = 0

        while in_degree:
            # 当前层级中，入度为 0 的所有节点
            current_level = [n for n in in_degree if in_degree[n] == 0]
            if not current_level:
                raise ValueError("图中存在环，无法进行分层拓扑排序。")

            step_key = f"step_{step}"
            structured[step_key] = {}

            for node in current_level:
                agent = self._extract_agent_name(node)
                action_name = node.action.name
                params = {
                    p.name: str(v) for p, v in zip(node.action.parameters, node.actual_parameters)
                }

                if agent not in structured[step_key]:
                    structured[step_key][agent] = {}
                structured[step_key][agent][action_name] = params

                # 删除该节点，更新其后继节点的入度
                for succ in list(graph.successors(node)):
                    in_degree[succ] -= 1
                del in_degree[node]

            step += 1

        return structured

    def as_dict(self) -> Dict[str, Dict[str, Dict[str, Dict[str, str]]]]:
        """
        返回结构化调度计划字典。
        """
        return self.plan_steps

    def to_json(self, indent: int = 2) -> str:
        """
        返回 JSON 字符串。
        """
        return json.dumps(self.plan_steps, indent=indent)

    def pretty_print(self):
        """
        结构化打印调度内容。
        """
        for step, agents in self.plan_steps.items():
            print(f"{step}:")
            for agent, actions in agents.items():
                print(f"  {agent}:")
                for action_name, params in actions.items():
                    param_str = ", ".join(f"{k}={v}" for k, v in params.items())
                    print(f"    {action_name}({param_str})")

    def visualize_gantt(self, figsize=(12, 6), save_path="plan_gantt.svg", font_size=12):
        import matplotlib.pyplot as plt
        import matplotlib.patches as mpatches

        structured = self.plan_steps
        agent_rows = {}
        for step_idx, (step, agents) in enumerate(structured.items()):
            for agent, actions in agents.items():
                if agent not in agent_rows:
                    agent_rows[agent] = []
                for action_name, param_dict in actions.items():
                    agent_rows[agent].append((step_idx, action_name, param_dict))

        fig, ax = plt.subplots(figsize=figsize)
        yticks, ylabels = [], []
        color_map = {}
        cmap = plt.get_cmap("tab20")
        color_idx = 0

        for i, (agent, actions) in enumerate(agent_rows.items()):
            y = i
            yticks.append(y)
            ylabels.append(agent)
            for (step_idx, action_name, param_dict) in actions:
                color = color_map.setdefault(action_name, cmap(color_idx % 20))
                color_idx += 1

                # 绘制 bar 块
                ax.broken_barh([(step_idx, 1)], (y - 0.4, 0.8),
                               facecolors=color, edgecolor='black')

                # 主动作名
                ax.text(step_idx + 0.5, y, action_name,
                        ha='center', va='center', fontsize=font_size, color='black')

                # 添加作用对象行（参数列表拼接）
                object_str = ", ".join(str(v) for v in param_dict.values())
                ax.text(step_idx + 0.5, y + 0.4, object_str,
                        ha='center', va='bottom', fontsize=font_size - 2, color='dimgray')

        ax.set_yticks(yticks)
        ax.set_yticklabels(ylabels, fontsize=font_size)
        ax.set_xticks(list(range(len(structured))))
        ax.set_xticklabels([f"step {i}" for i in range(len(structured))], fontsize=font_size)
        ax.set_xlabel("Time step", fontsize=font_size + 2)
        ax.set_title("Multi-Agent Action Schedule", fontsize=font_size + 4)
        ax.set_ylim(-1, len(agent_rows) + 1)
        ax.grid(True, axis='x', linestyle='--', alpha=0.5)

        plt.tight_layout()
        plt.savefig(save_path, format="svg", bbox_inches="tight")
        print(f"Gantt chart saved as: {save_path}")
