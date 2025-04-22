# solve_delivery.py

from unified_planning.io import PDDLReader
from unified_planning.shortcuts import OneshotPlanner


def main():
    # 1. 解析 PDDL 文件
    reader = PDDLReader()
    problem = reader.parse_problem("domain.pddl", "problem.pddl")
    print(problem)
    with OneshotPlanner(name='fmap', params={'heuristic': '1'}) as planner:
        result = planner.solve(problem)
        print(result)
if __name__ == "__main__":
    main()