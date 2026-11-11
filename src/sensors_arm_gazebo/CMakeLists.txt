cmake_minimum_required(VERSION 3.5)
project(sensors_arm_gazebo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensors_arm_msgs REQUIRED)

add_library(sensors_arm_kinematic_joints SHARED
  src/kinematic_joint_plugin.cpp
)
ament_target_dependencies(sensors_arm_kinematic_joints
  gazebo_dev
  gazebo_ros
  rclcpp
  sensor_msgs
)
target_include_directories(sensors_arm_kinematic_joints PUBLIC
  ${GAZEBO_INCLUDE_DIRS}
)
target_link_libraries(sensors_arm_kinematic_joints
  ${GAZEBO_LIBRARIES}
)

add_library(sensors_arm_camera_params SHARED
  src/camera_params_plugin.cpp
)
ament_target_dependencies(sensors_arm_camera_params
  gazebo_dev
  gazebo_ros
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  sensors_arm_msgs
)
target_include_directories(sensors_arm_camera_params PUBLIC
  ${GAZEBO_INCLUDE_DIRS}
)
target_link_libraries(sensors_arm_camera_params
  ${GAZEBO_LIBRARIES}
)

# Gazebo plugins live under lib/
install(TARGETS sensors_arm_kinematic_joints sensors_arm_camera_params
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY urdf worlds launch config models media
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
