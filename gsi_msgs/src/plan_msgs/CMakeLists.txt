cmake_minimum_required(VERSION 3.8)
project(plan_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

set(MSG_FILES
  "msg/SkillFeedback.msg"
  "msg/Parameter.msg"
  "msg/Plan.msg"
  "msg/RobotFeedback.msg"
  "msg/RobotSkill.msg"
  "msg/SkillInfo.msg"
  "msg/TimestepSkills.msg"
  "msg/VelTwistPose.msg"
)

set(ACTION_FILES
  "action/PlanExecution.action"
  "action/SkillExecution.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${ACTION_FILES}
  DEPENDENCIES geometry_msgs std_msgs action_msgs
)

# Export dependencies for downstream packages
ament_export_dependencies(rosidl_default_runtime action_msgs)

# Test dependencies
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
