cmake_minimum_required(VERSION 3.8)
project(tp5_navigation)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)

add_executable(fsm_node src/fsm_node.cpp)
ament_target_dependencies(fsm_node rclcpp sensor_msgs geometry_msgs visualization_msgs)

add_executable(bt_node src/bt_node.cpp)
ament_target_dependencies(bt_node rclcpp sensor_msgs geometry_msgs visualization_msgs behaviortree_cpp)

install(TARGETS fsm_node bt_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch trees DESTINATION share/${PROJECT_NAME})

ament_package()
