# CMAKE SPECS
cmake_minimum_required(VERSION 3.0.2)
project(kinematics_rb1)

# FIND ROS PACKAGES
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  tf
  nav_msgs
  message_generation
  rosgraph_msgs
)

add_message_files(
  FILES
  encoders.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

# USE CATKIN PACKAGES
catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS roscpp message_runtime
)

# ADDING INCLUDES
include_directories(
  include
  include/kinematics_rb1
#  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

# DIFFERENTIAL DRIVE KINEMATICS
add_executable(diff_drive src/kinematics_rb1/diff_drive.cpp)
add_dependencies(diff_drive ${catkin_EXPORTED_TARGETS})
target_link_libraries(diff_drive ${catkin_LIBRARIES})
# DIFFERENTIAL DRIVE KINEMATICS

# ODOMETRY
add_executable(odometry src/kinematics_rb1/odometry.cpp)
add_dependencies(odometry ${catkin_EXPORTED_TARGETS})
target_link_libraries(odometry ${catkin_LIBRARIES})
# ODOMETRY

# LIBRARIES
# VELOCITY CONTROLLER LIBRARY
add_library(velocity_controller 
  STATIC
  src/kinematics_rb1/velocity_controller.cpp
)

# ODOMETRY READER LIBRARY
add_library(odometry_reader
  STATIC
  src/kinematics_rb1/odometry_reader.cpp
)
# END OF LIBRARIES

# KINEMATIC CONTROLLER
add_executable(kinematic_controller src/kinematics_rb1/kinematic_controller.cpp)
add_dependencies(kinematic_controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(kinematic_controller velocity_controller odometry_reader ${catkin_LIBRARIES})
# KINEMATIC CONTROLLER