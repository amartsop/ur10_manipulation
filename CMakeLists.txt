cmake_minimum_required(VERSION 3.0.2)
project(ur10_manipulation)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp rospy std_msgs actionlib moveit_ros_planning_interface
  robotiq_ft_sensor moveit_msgs)

catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs actionlib 
  moveit_ros_planning_interface
  robotiq_ft_sensor moveit_msgs)

# Eigen linking
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

#******************** Build ***********************#
# Source files variable
set(PROJ_SOURCE_FILES
  src/euler_rotations.cpp
  src/robot_control.cpp
  src/path_planning.cpp
  src/path_id.cpp
  src/ft_sensor.cpp
  src/camera_control.cpp)

# Directories variable
set(PROJ_INCL_DIRS include include/ur10_manipulation
  ${catkin_INCLUDE_DIRS})

set(PROJ_LIBS ${catkin_LIBRARIES})

# Include directories to the project
include_directories(${PROJ_INCL_DIRS})

# Add executables to target ur10_manipulation_node
add_executable(${PROJECT_NAME}_node src/ur10_manipulation_node.cpp
  ${PROJ_SOURCE_FILES})

# Link libraries to target ur10_manipulation_node
target_link_libraries(${PROJECT_NAME}_node ${PROJ_LIBS})