cmake_minimum_required(VERSION 2.8.3)
project(group3_rwa4)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nist_gear
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  trajectory_msgs
  eigen_conversions
  control_msgs
  geometry_msgs
  moveit_msgs
  rosgraph_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  moveit_core
  moveit_ros_perception
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
)
find_package(Eigen3 REQUIRED)




add_service_files(
  DIRECTORY srv
  FILES GetConveyorBeltPartPickPose.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
catkin_package(
  CATKIN_DEPENDS
  message_runtime
)



add_executable(conveyor_belt_monitor_controller src/conveyor_belt_monitor.cpp src/conveyor_belt_monitor_controller.cpp)
target_link_libraries(conveyor_belt_monitor_controller ${catkin_LIBRARIES})
add_dependencies(conveyor_belt_monitor_controller group3_rwa4_generate_messages_cpp)



add_executable(arm_controller src/arm.cpp src/ariac_agv.cpp src/agility.cpp src/utils.cpp src/gantry.cpp src/arm_controller.cpp)
target_link_libraries(arm_controller ${catkin_LIBRARIES})

add_executable(complete_controller src/arm.cpp src/ariac_agv.cpp src/agility.cpp src/utils.cpp src/gantry.cpp src/complete_controller.cpp)
target_link_libraries(complete_controller ${catkin_LIBRARIES})

add_executable(gantry_controller src/ariac_agv.cpp src/agility.cpp src/utils.cpp src/gantry_controller.cpp src/gantry.cpp)
target_link_libraries(gantry_controller ${catkin_LIBRARIES})

add_executable(blackout_controller src/blackout.cpp src/blackout_controller.cpp)
target_link_libraries(blackout_controller ${catkin_LIBRARIES})

add_executable(obstacle_controller src/obstacle_controller.cpp)
target_link_libraries(obstacle_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES})


install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES
  config/robot_workcell_specs.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
