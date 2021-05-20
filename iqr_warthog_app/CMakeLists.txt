cmake_minimum_required(VERSION 3.0.2)
# cmake_minimum_required(VERSION 2.8.3)

project(iqr_warthog_app)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  actionlib_msgs  
  tf
  tf2
  tf2_eigen
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

add_action_files(DIRECTORY action FILES WaypointNav.action)
## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES iqr_warthog_app
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
  CATKIN_DEPENDS actionlib_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(path_follow_gerona src/path_follow_gerona.cpp)
add_executable(path_follow_gerona_straight src/path_follow_gerona_straight.cpp)
add_executable(path_follow_pid src/path_follow_pid.cpp)
add_executable(map_localization src/map_localization.cpp)
add_executable(map_position_saver src/map_position_saver.cpp)
add_executable(path_generate src/path_generate.cpp)
add_executable(path_generate_four src/path_generate_four.cpp)
add_executable(path_follow_pid_straight src/path_follow_pid_straight.cpp)
add_executable(cmd_vel_filter src/cmd_vel_filter.cpp)

add_dependencies(path_follow_gerona ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(path_follow_gerona_straight ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(path_follow_pid ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(map_localization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(map_position_saver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(path_generate ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(path_generate_four ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(path_follow_pid_straight ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(cmd_vel_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cmd_vel_filter
  ${catkin_LIBRARIES}
)
target_link_libraries(path_follow_pid_straight
  ${catkin_LIBRARIES}
)
target_link_libraries(path_follow_gerona
  ${catkin_LIBRARIES}
)
target_link_libraries(path_follow_gerona_straight
  ${catkin_LIBRARIES}
)
target_link_libraries(path_follow_pid
  ${catkin_LIBRARIES}
)
target_link_libraries(map_localization
  ${catkin_LIBRARIES}
)
target_link_libraries(map_position_saver
  ${catkin_LIBRARIES}
)
target_link_libraries(path_generate
  ${catkin_LIBRARIES}
)
target_link_libraries(path_generate_four
  ${catkin_LIBRARIES}
)