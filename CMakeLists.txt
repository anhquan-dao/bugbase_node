cmake_minimum_required(VERSION 3.0.2)
project(bugbase_node)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_python_setup()

catkin_package(
 LIBRARIES bugbase_driver
 CATKIN_DEPENDS rospy
)

catkin_install_python(
  PROGRAMS 
    nodes/bugbase_node.py 
    nodes/speed_test_node.py 
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)
