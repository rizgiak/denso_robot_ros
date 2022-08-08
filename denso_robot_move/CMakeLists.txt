cmake_minimum_required(VERSION 3.0.2)
project(denso_robot_move)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    interactive_markers
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    gripper_ntlab_controller
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
    gripper_ntlab_controller
  DEPENDS
    EIGEN3
)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})

add_executable(ik_grip src/ik_grip.cpp)
target_link_libraries(ik_grip ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS ik_grip DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(move_group_in_hand_manipulation_hor_data src/move_group_in_hand_manipulation_hor_data.cpp)
target_link_libraries(move_group_in_hand_manipulation_hor_data ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_group_in_hand_manipulation_hor_data DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(move_group_in_hand_manipulation_data src/move_group_in_hand_manipulation_data.cpp)
target_link_libraries(move_group_in_hand_manipulation_data ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_group_in_hand_manipulation_data DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(move_group_in_hand_manipulation_hor src/move_group_in_hand_manipulation_hor.cpp)
target_link_libraries(move_group_in_hand_manipulation_hor ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_group_in_hand_manipulation_hor DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(move_group_in_hand_manipulation_limit src/move_group_in_hand_manipulation_limit.cpp)
target_link_libraries(move_group_in_hand_manipulation_limit ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_group_in_hand_manipulation_limit DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(move_group_in_hand_manipulation src/move_group_in_hand_manipulation.cpp)
target_link_libraries(move_group_in_hand_manipulation ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_group_in_hand_manipulation DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(move_group_interface_two_points src/move_group_interface_two_points.cpp)
target_link_libraries(move_group_interface_two_points ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_group_interface_two_points DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS
  scripts/GripperNTLab.py
  scripts/move_interface.py
  scripts/cop_test.py
  scripts/cop_test_adjust.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})