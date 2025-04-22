from unified_planning.shortcuts import *
from unified_planning.model.multi_agent import *
from unified_planning.io.ma_pddl_writer import MAPDDLWriter
from pddl.agent_schedule import ActionSchedule
from pddl.plan import DirectedActionGraph

from unified_planning.shortcuts import *
from unified_planning.model.multi_agent import *
from unified_planning.io.ma_pddl_writer import MAPDDLWriter
from pddl.agent_schedule import ActionSchedule
from pddl.plan import DirectedActionGraph
import ast


class MultiAgentPlanner:
    def __init__(self):
        self._define_problem()
        self._initialize_problem()
        self.plan_steps = None

    def _get_goal_eval_context(self):
        return {
            "robot_at": self.robot_at,
            "item_at": self.item_at,
            "holding": self.holding,
            "depot": self.depot,
            "items": self.items,
            "places": self.places,
            "Object": Object,
            "BoolType": BoolType,
        }

    def _extract_goals_from_code(self, goal_code: str):
        """
        Parses a string like 'problem.add_goal(...)' or 'problem.add_goals(...)'
        and extracts goal expressions, even from list comprehensions.
        """
        try:
            parsed = ast.parse(goal_code)
            goal_exprs = []
            ctx = self._get_goal_eval_context()

            for stmt in parsed.body:
                if isinstance(stmt, ast.Expr) and isinstance(stmt.value, ast.Call):
                    call = stmt.value
                    func = call.func
                    if (
                            isinstance(func, ast.Attribute)
                            and isinstance(func.value, ast.Name)
                            and func.value.id == "problem"
                            and func.attr in ["add_goal", "add_goals"]
                    ):
                        for arg in call.args:
                            code = compile(ast.Expression(arg), filename="<ast>", mode="eval")
                            result = eval(code, ctx, ctx)
                            # handle both single goal and list of goals
                            if isinstance(result, list):
                                goal_exprs.extend(result)
                            else:
                                goal_exprs.append(result)
            return goal_exprs
        except Exception as e:
            print(f"[Goal Parse Error] {e}")
            return []

    def _define_problem(self):
        # Define types and fluents
        self.Robot = UserType("robot")
        self.Location = UserType("location")
        self.Item = UserType("item")

        self.robot_at = Fluent("robot_at", BoolType(), l=self.Location)
        self.item_at = Fluent("item_at", BoolType(), i=self.Item, l=self.Location)
        self.holding = Fluent("holding", BoolType(), i=self.Item)

        # Define objects
        self.depot = Object("depot", self.Location)
        self.items = [Object(f"item{i}", self.Item) for i in range(1, 3)]
        self.places = [Object(f"place{i}", self.Location) for i in range(1, 3)]

        # Define actions
        self.nav = InstantaneousAction("navigate-to", start=self.Location, goal=self.Location)
        start = self.nav.parameter("start")
        goal = self.nav.parameter("goal")
        self.nav.add_precondition(self.robot_at(start))
        self.nav.add_effect(self.robot_at(start), False)
        self.nav.add_effect(self.robot_at(goal), True)

        self.pu = InstantaneousAction("pick-up", it=self.Item, loc=self.Location)
        it = self.pu.parameter("it")
        loc = self.pu.parameter("loc")
        self.pu.add_precondition(self.robot_at(loc))
        self.pu.add_precondition(self.item_at(it, loc))
        self.pu.add_effect(self.holding(it), True)

        self.pd = InstantaneousAction("put-down", it=self.Item, loc=self.Location)
        it = self.pd.parameter("it")
        loc = self.pd.parameter("loc")
        self.pd.add_precondition(self.robot_at(loc))
        self.pd.add_precondition(self.holding(it))
        self.pd.add_effect(self.holding(it), False)
        self.pd.add_effect(self.item_at(it, loc), True)

    def _initialize_problem(self):
        self.problem = MultiAgentProblem("delivery_task")
        self.problem.ma_environment.add_fluent(self.item_at, default_initial_value=False)

        self.robots = []
        for i in range(1, 4):
            ag = Agent(f"robot{i}", self.problem)
            ag.add_public_fluent(self.robot_at, default_initial_value=False)
            ag.add_private_fluent(self.holding, default_initial_value=False)
            ag.add_actions([self.nav, self.pu, self.pd])
            self.robots.append(ag)
            self.problem.add_agent(ag)

        self.problem.add_objects(self.items)
        self.problem.add_objects(self.places)
        self.problem.add_object(self.depot)

        for it, loc in zip(self.items, self.places):
            self.problem.set_initial_value(self.item_at(it, loc), True)

        for robot in self.robots:
            self.problem.set_initial_value(Dot(robot, self.robot_at(self.depot)), True)

    def set_goal_and_solve(self, goal_code: str):
        """
        Set new goal from a string of Python code and solve the planning problem.
        The goal_code string should evaluate to a list of FluentExpressions.
        """
        self.problem.clear_goals()
        goal_exprs = self._extract_goals_from_code(goal_code)
        if not goal_exprs:
            print("No goals parsed from input.")
            return None

        for goal in goal_exprs:
            self.problem.add_goal(goal)

        w = MAPDDLWriter(self.problem)
        w.write_ma_domain(self.problem.name)
        w.write_ma_problem(self.problem.name)

        with OneshotPlanner(problem_kind=self.problem.kind) as planner:
            result = planner.solve(self.problem)
            po_plan = result.plan
            if po_plan is None:
                print("No plan found.")
                self.plan_steps = None
            else:
                print("Plan found.")
                graph = DirectedActionGraph(po_plan)
                schedule = ActionSchedule(graph, self.problem)
                # schedule.pretty_print()
                # schedule.visualize_gantt()
                self.plan_steps = schedule.plan_steps

        return self.plan_steps


if __name__ == '__main__':
    multi_agent_planner = MultiAgentPlanner()
    multi_agent_planner.set_goal_and_solve(goal_code="""problem.add_goals([item_at(it, depot) for it in items])
""")
    print(multi_agent_planner.plan_steps)
