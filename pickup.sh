ros2 action send_goal /h1_0/skill_execution plan_msgs/action/SkillExecution '{
  skill_request: {
    skill_list: [
      {
        skill: "pickup",
        params: [
          {key: "object_prim_path", value: "/World/Critical_Package"},
          {key: "robot_hand_prim_path", value: "/World/robot/h1/h1_0/pelvis"},
          {key: "local_pos_hand", value: "[0.5, 0.0, 0.0]"},
          {key: "local_pos_object", value: "[0.0, 0.0, 0.0]"},
          {key: "axis", value: "[0.0, 0.0, 1.0]"}
        ]
      }
    ]
  }
}'  && echo "Pickup successful. Waiting 0.5s..." && sleep 0.5 && ros2 action send_goal /h1_0/skill_execution plan_msgs/action/SkillExecution '{
skill_request: {
    skill_list: [
    {
        skill: "navigation",
        params: [
            {key: "start_pos", value: "[4, 4, 1.05]"},
            {key: "goal_pos", value: "[6, 4, 1.05]"},
            {key: "goal_quat_wxyz", value: "[0.707, 0.0, 0.0, 0.707]"}
            ]
            }
        ]
    }
}'
