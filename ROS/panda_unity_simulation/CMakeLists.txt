cmake_minimum_required(VERSION 2.8.3)
project(panda_unity_simulation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED 
geometry_msgs 
actionlib_msgs 
moveit_core 
moveit_ros_planning
moveit_ros_planning_interface 
moveit_ros_move_group
moveit_visual_tools
moveit_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  BoundingBox.msg
  SpatialMappingMesh.msg
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
    FollowObjectMoveit.action 
    PlanPathMoveit.action
    ConfirmPathMoveit.action
    RegisterCollisionObjects.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
  moveit_msgs
)

catkin_package(
CATKIN_DEPENDS geometry_msgs actionlib_msgs
)


include_directories(
${catkin_INCLUDE_DIRS}
)

add_executable(ManipObstaclesManager src/ManipObstaclesManager.cpp)
target_link_libraries(ManipObstaclesManager ${catkin_LIBRARIES})  
add_dependencies(ManipObstaclesManager panda_unity_simulation_generate_messages_cpp)

add_executable(pose_server2 src/pose_server2.cpp)
target_link_libraries(pose_server2 ${catkin_LIBRARIES})
add_dependencies(pose_server2 panda_unity_simulation_generate_messages_cpp)

add_executable(pose_client2 src/pose_client2.cpp)
target_link_libraries(pose_client2 ${catkin_LIBRARIES})
add_dependencies(pose_client2 panda_unity_simulation_generate_messages_cpp)

add_executable(follow_object_interface src/follow_object_interface.cpp)
target_link_libraries(follow_object_interface ${catkin_LIBRARIES})
add_dependencies(follow_object_interface panda_unity_simulation_generate_messages_cpp)