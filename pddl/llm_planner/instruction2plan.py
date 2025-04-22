import datetime
import os
from pathlib import Path

from instruction2goal import instruction2goal
from run_script import run_script
from pddl_planner import MultiAgentPlanner


# 根据时间信息创建文件夹
def get_project_root():
    """Search upwards to find the project root directory."""
    current_path = Path.cwd()
    while True:
        if (
                (current_path / ".git").exists()
                or (current_path / ".project_root").exists()
                or (current_path / ".gitignore").exists()
        ):
            # use metagpt with git clone will land here
            return current_path
        parent_path = current_path.parent
        if parent_path == current_path:
            # use metagpt with pip install will land here
            cwd = Path.cwd()
            return cwd
        current_path = parent_path


def creat_directory():
    project_root = get_project_root()
    current_datetime = datetime.datetime.now()
    formatted_date = current_datetime.strftime("%Y-%m-%d_%H-%M-%S")
    workspace_root = (
            project_root
            / f"pddl/workspace/{formatted_date}"
    )
    os.makedirs(workspace_root)


if __name__ == '__main__':
    goal = instruction2goal(instruction='I want the robots to transport all items back to the depot.')
    multi_agent_planner = MultiAgentPlanner()
    multi_agent_planner.set_goal_and_solve(goal)
    print(multi_agent_planner.plan_steps)
