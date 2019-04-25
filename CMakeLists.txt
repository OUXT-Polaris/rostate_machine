cmake_minimum_required(VERSION 2.8.3)
project(rostate_machine)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  roslib
)

find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
  FILES
  Event.msg
  State.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES event_client state_machine
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Non ROS Library for State Machine
add_library(state_machine src/state_machine.cpp)

# ROS Wrapper for the State Machine
add_executable(rostate_machine_node src/rostate_machine_node.cpp src/rostate_machine.cpp)
target_link_libraries(rostate_machine_node ${catkin_LIBRARIES} state_machine)
add_dependencies(rostate_machine_node rostate_machine_generate_messages_cpp)

# Event Client for State Machine
add_library(event_client src/event_client.cpp)
target_link_libraries(event_client ${catkin_LIBRARIES} state_machine)
add_dependencies(event_client rostate_machine_generate_messages_cpp)

# Example node for Event Client
add_executable(example_event_client_node
  src/example_event_client_node.cpp 
  src/example_event_client.cpp)
target_link_libraries(example_event_client_node ${catkin_LIBRARIES} event_client)
add_dependencies(example_event_client_node rostate_machine_generate_messages_cpp)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install library
install(TARGETS state_machine rostate_machine_node event_client example_event_client_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test-rostate_machine
  test/test_rostate_machine.test
  test/src/test_rostate_machine.cpp
  ${NODE_STATUS_PUBLISHER_SRC})
  target_link_libraries(test-rostate_machine
  ${catkin_LIBRARIES}
  state_machine)
endif ()
