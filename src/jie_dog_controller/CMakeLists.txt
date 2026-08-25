cmake_minimum_required(VERSION 3.8)
project(jie_dog_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(jie_dog_controller_node src/jie_dog_controller_node.cpp)
target_compile_features(jie_dog_controller_node PUBLIC cxx_std_17)
ament_target_dependencies(jie_dog_controller_node PUBLIC rclcpp nav_msgs std_msgs)

install(TARGETS jie_dog_controller_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
