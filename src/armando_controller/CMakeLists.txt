cmake_minimum_required(VERSION 3.5)
project(armando_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdf REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(aruco_msgs REQUIRED)
# ------------------------------------------------------

include_directories(include)

add_executable(armando_controller_node 
  src/armando_controller_node.cpp 
  src/pose_loader.cpp
)
ament_target_dependencies(armando_controller_node 
  rclcpp 
  rclcpp_action 
  std_msgs 
  control_msgs 
  trajectory_msgs 
  ament_index_cpp
)
target_link_libraries(armando_controller_node yaml-cpp)

add_executable(armando_pick_place_node 
  src/armando_pick_and_place.cpp
)
ament_target_dependencies(armando_pick_place_node 
  rclcpp 
  std_msgs 
  control_msgs 
  trajectory_msgs
)

add_executable(armando_ik_node src/armando_ik_node.cpp)

ament_target_dependencies(armando_ik_node 
  rclcpp 
  std_msgs 
  control_msgs 
  trajectory_msgs
  kdl_parser
  tf2_ros
  tf2_kdl
  tf2_geometry_msgs
  aruco_msgs
)
target_link_libraries(armando_ik_node orocos-kdl)

add_executable(armando_ik_kdl_node src/armando_ik_kdl_node.cpp)
ament_target_dependencies(armando_ik_kdl_node
  rclcpp
  rclcpp_action
  sensor_msgs
  geometry_msgs
  std_msgs
  control_msgs
  trajectory_msgs
  kdl_parser
  urdf
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(armando_ik_kdl_node orocos-kdl)

add_executable(armando_aruco_target_node src/armando_aruco_target_node.cpp)
ament_target_dependencies(armando_aruco_target_node
  rclcpp
  geometry_msgs
  tf2_geometry_msgs
  aruco_msgs
)


install(TARGETS 
  armando_controller_node
  armando_pick_place_node
  armando_ik_node
  armando_ik_kdl_node
  armando_aruco_target_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(control_msgs rclcpp std_msgs trajectory_msgs kdl_parser tf2_ros)

ament_package()