cmake_minimum_required(VERSION 3.8)
project(explorer_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SymbolicFact.msg"
  "msg/ExplorationTarget.msg"
  "msg/ExplorationTargetArray.msg"
  "msg/DecisionExplanation.msg"
  "msg/MissionStatus.msg"
  "msg/TargetResult.msg"
  "srv/AddSymbolicFact.srv"
  "srv/EvaluateFrontiers.srv"
  "action/ExecuteMission.action"
  DEPENDENCIES
    builtin_interfaces
    std_msgs
    geometry_msgs
    action_msgs
)

ament_package()