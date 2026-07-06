cmake_minimum_required(VERSION 3.5)
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
project(aero_hand_open)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_srvs
  trajectory_msgs
)

catkin_package()

# Install python scripts
catkin_install_python(PROGRAMS
  scripts/aero_hand_interpolator_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
