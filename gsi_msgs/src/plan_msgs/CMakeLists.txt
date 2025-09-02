cmake_minimum_required(VERSION 3.8)
project(plan_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)                 # ← 新增

set(MSG_FILES
  "msg/Parameter.msg"
  "msg/Plan.msg"
  "msg/RobotFeedback.msg"
  "msg/RobotSkill.msg"
  "msg/SkillInfo.msg"
  "msg/TimestepSkills.msg"
  "msg/VelTwistPose.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES geometry_msgs std_msgs           # ← 补齐 std_msgs
)

ament_export_dependencies(rosidl_default_runtime geometry_msgs std_msgs)  # ← 补齐 std_msgs
ament_package()
