PROBLEM_TEMPLATE = """
## Role Setting:
You are an assistant for intelligent planning code generation.

## Task: 
You are given a Python script that defines a PDDL-style multi-agent planning domain using the Unified Planning framework.
Based on a user's natural language instruction, your task is to generate the corresponding **goal definition code** using the `add_goal`or `add_goals` method. 

The goal should be expressed as a conjunction of `FluentExpressions`, using `problem.add_goal(...)` or `problem.add_goals(...)`. 
Don't use And(...)
## The following is the base code (with the domain, actions, and initial state already defined):
```python
{code}
...(waiting for goal definition)
```

## Output Format:
Please only return the goal definition as Python code, and **wrap the result inside a markdown code block** as shown below.
```python
xxx
```
## User's Instruction:
{instruction}

Generate the goal specification accordingly.
"""

BASECODE = """from unified_planning.shortcuts import *
from unified_planning.model.multi_agent import *
from unified_planning.io.ma_pddl_writer import MAPDDLWriter
from pddl.agent_schedule import ActionSchedule
from pddl.plan import DirectedActionGraph

# USERTYPEs
Robot = UserType("robot")
Location = UserType("location")
Item = UserType("item")

# FLUENTs
robot_at = Fluent("robot_at", BoolType(), l=Location)
item_at = Fluent("item_at", BoolType(), i=Item, l=Location)
holding = Fluent("holding", BoolType(), i=Item)

# OBJECTs
depot = Object("depot", Location)
items = [Object(f"item{i}", Item) for i in range(1, 3)]
places = [Object(f"place{i}", Location) for i in range(1, 3)]

# ACTIONs
# navigate_to(r,s,d)
nav = InstantaneousAction("navigate-to", start=Location, goal=Location)
start = nav.parameter("start")
goal = nav.parameter("goal")
nav.add_precondition(robot_at(start))
nav.add_effect(robot_at(start), False)
nav.add_effect(robot_at(goal), True)
# pick-up(r, i, l)
pu = InstantaneousAction("pick-up", it=Item, loc=Location)
it = pu.parameter("it")
loc = pu.parameter("loc")
pu.add_precondition(robot_at(loc))
pu.add_precondition(item_at(it, loc))
pu.add_effect(holding(it), True)
# put-down(r, i, l)
pd = InstantaneousAction("put-down", it=Item, loc=Location)
it = pd.parameter("it")
loc = pd.parameter("loc")
pd.add_precondition(robot_at(loc))
pd.add_precondition(holding(it))
pd.add_effect(holding(it), False)
pd.add_effect(item_at(it, loc), True)

# problem
problem = MultiAgentProblem("delivery_task")
problem.ma_environment.add_fluent(item_at, default_initial_value=False)

# agents
robots = []
for i in range(1, 4):
    ag = Agent(f"robot{i}", problem)
    ag.add_public_fluent(robot_at, default_initial_value=False)  # 公有: 位置
    ag.add_private_fluent(holding, default_initial_value=False)  # 私有: 抓取状态
    ag.add_actions([pd, nav, pu])
    robots.append(ag)

for robot in robots:
    problem.add_agent(robot)

# add objects
problem.add_objects(items)
problem.add_objects(places)
problem.add_object(depot)

# set initial values
for it, loc in zip(items, places):
    problem.set_initial_value(item_at(it, loc), True)

for robot in robots:
        problem.set_initial_value(Dot(robot, robot_at(depot)), True)

"""
