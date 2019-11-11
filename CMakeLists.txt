cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg tf)

## Generate services in the 'srv' folder
add_service_files(FILES changeOutput.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(
## CATKIN_DEPENDS message_runtime
)

## To run tests, add dependencies for gtest and rostest
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(testTalker test/testTalkerRun.launch test/main.cpp test/testTalker.cpp)
  target_link_libraries(testTalker ${catkin_LIBRARIES})
  add_dependencies(testTalker talker beginner_tutorials_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
endif()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})

