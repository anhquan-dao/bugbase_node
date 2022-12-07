cmake_minimum_required(VERSION 3.0.2)
project(bugbase_node)

find_package(catkin REQUIRED COMPONENTS rospy message_generation)
find_package(Boost REQUIRED COMPONENTS python)
find_package(PythonLibs REQUIRED)

catkin_python_setup()

add_message_files(
  FILES
  BugbaseMotor.msg
  BugbaseMotorArray.msg
)

generate_messages()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bugbase_driver esp32_message_header
  CATKIN_DEPENDS rospy message_runtime
)

include_directories(
  esp32/StepperESP/include
  ${CATKIN_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(esp32_message_header SHARED 
  src/esp32_message_header.cpp
)

set_target_properties(esp32_message_header PROPERTIES 
  PREFIX "" 
  LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

target_link_libraries(esp32_message_header
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES} 
  ${Boost_LIBRARIES}
)

install(TARGETS esp32_message_header
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

catkin_install_python(
  PROGRAMS 
    nodes/driver_node.py 
    nodes/speed_test_node.py
    nodes/test_node.py
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  # find_package(rostest REQUIRED)
  # add_rostest(test/rostest.test)
  catkin_add_nosetests(test/test_disconnect.py)
endif()

