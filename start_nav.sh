#!/bin/bash

echo  "Sending goal to h1"
ros2 action send_goal /h1_0/skill_execution plan_msgs/action/SkillExecution '{skill_request: {skill_list: [{skill: "navigation", params: [{key: "goal_pos", value: "[3, 4.5, 1.05]"},{key: "start_pos", value: "[3, 3, 1.05]"}, {key: "goal_quat_wxyz", value: "[0.707, 0.0, 0.0, 0.707]"}]}]}}'

echo "Sending goal to jetbot_0..."
ros2 action send_goal /jetbot_0/skill_execution plan_msgs/action/SkillExecution '{skill_request: {skill_list: [{skill: "navigation", params: [{key: "goal_pos", value: "[15, 5, 0]"}]}]}}' &

echo "Sending goal to cf2x_0..."
#ros2 action send_goal /cf2x_0/skill_execution plan_msgs/action/SkillExecution '{skill_request: {skill_list: [{skill: "navigation", params: [{key: "goal_pos", value: "[6, 6, 3]"}]}]}}' &

echo "Goals sent."
