cmake_minimum_required(VERSION 3.0.2)
project(mak4_sim_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  mavros_msgs
  tf2_ros
  pcl_ros
  octomap_msgs
)

find_package(yaml-cpp REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)

catkin_package(
  CATKIN_DEPENDS
    roscpp std_msgs geometry_msgs sensor_msgs nav_msgs
    visualization_msgs mavros_msgs tf2_ros pcl_ros octomap_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

# ========== 节点 ==========

# configure_ekf2
add_executable(configure_ekf2_node src/configure_ekf2_node.cpp)
target_link_libraries(configure_ekf2_node ${catkin_LIBRARIES})

# slam_to_mavros
add_executable(slam_to_mavros_node src/slam_to_mavros_node.cpp)
target_link_libraries(slam_to_mavros_node ${catkin_LIBRARIES})

# nan_filter
add_executable(nan_filter_node src/nan_filter_node.cpp)
target_link_libraries(nan_filter_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# keyboard_teleop
add_executable(keyboard_teleop_node src/keyboard_teleop_node.cpp)
target_link_libraries(keyboard_teleop_node ${catkin_LIBRARIES})

# takeoff_hover_test
add_executable(takeoff_hover_test_node src/takeoff_hover_test_node.cpp)
target_link_libraries(takeoff_hover_test_node ${catkin_LIBRARIES})

# patrol_planner (OctoMap-based)
add_executable(patrol_planner_node src/patrol_planner_node.cpp)
target_link_libraries(patrol_planner_node ${catkin_LIBRARIES})

# grid_map_planner (PGM-based)
add_executable(grid_map_planner_node src/grid_map_planner_node.cpp)
target_link_libraries(grid_map_planner_node ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} ${PCL_LIBRARIES})

# save_map
add_executable(save_map_node src/save_map_node.cpp)
target_link_libraries(save_map_node ${catkin_LIBRARIES})

# pcd_to_2d_map (standalone tool, no ROS needed)
add_executable(pcd_to_2d_map src/pcd_to_2d_map_node.cpp)
target_link_libraries(pcd_to_2d_map ${catkin_LIBRARIES})

# load_map
add_executable(load_map_node src/load_map_node.cpp)
target_link_libraries(load_map_node ${catkin_LIBRARIES})

# boustrophedon_planner
add_executable(boustrophedon_planner_node src/boustrophedon_planner_node.cpp)
target_link_libraries(boustrophedon_planner_node ${catkin_LIBRARIES})

# rrt_star_planner
add_executable(rrt_star_planner_node src/rrt_star_planner_node.cpp)
target_link_libraries(rrt_star_planner_node ${catkin_LIBRARIES})

# interactive_planner
add_executable(interactive_planner_node src/interactive_planner_node.cpp)
target_link_libraries(interactive_planner_node ${catkin_LIBRARIES})

# ========== 安装 ==========
install(TARGETS
  configure_ekf2_node slam_to_mavros_node nan_filter_node
  keyboard_teleop_node takeoff_hover_test_node
  patrol_planner_node grid_map_planner_node
  save_map_node pcd_to_2d_map load_map_node
  boustrophedon_planner_node rrt_star_planner_node interactive_planner_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
