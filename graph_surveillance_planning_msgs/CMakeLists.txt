cmake_minimum_required(VERSION 3.8)
project(graph_surveillance_planning_msgs)


##################
## Dependencies ##
##################

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PlotPlan.msg"
  "msg/AgentState.msg"
  "msg/Edge.msg"
  "msg/ExpectedEventRates.msg"
  "msg/Metric.msg"
  "srv/TryFireflyConfig.srv"
  "srv/TryFireflyAlgorithmConfig.srv"
  "srv/GeneratePlan.srv"
  DEPENDENCIES geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

############
## Export ##
############

ament_export_dependencies(
  rosidl_default_runtime
)

ament_package()
