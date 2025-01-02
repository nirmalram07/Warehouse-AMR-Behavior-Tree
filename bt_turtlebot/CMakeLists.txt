cmake_minimum_required(VERSION 3.8)
project(bt_turtlebot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

install(DIRECTORY
launch config
DESTINATION share/${PROJECT_NAME}
)

set(BEHAVIOR_SOURCES
src/behavior_control.cpp 
)

set(TARGET_DEPENDS
  rclcpp
  behaviortree_cpp
  yaml-cpp
  rclcpp_action
  nav2_msgs
  tf2_geometry_msgs
  tf2
)

include_directories(include)
add_executable(ros_control src/ros_control.cpp ${BEHAVIOR_SOURCES})
ament_target_dependencies(ros_control ${TARGET_DEPENDS})
target_link_libraries(ros_control ${YAML_CPP_LIBRARIES})

install(TARGETS
ros_control
DESTINATION lib/${PROJECT_NAME})


ament_package()
