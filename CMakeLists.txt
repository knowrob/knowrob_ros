cmake_minimum_required(VERSION 3.5.0)
set(CMAKE_CXX_FLAGS "-Wswitch -std=c++0x -pthread ${CMAKE_CXX_FLAGS} -Wno-undef")
set(CMAKE_CXX_STANDARD 17)

project(knowrob_ros
        DESCRIPTION "A Knowledge Base System for Cognition-enabled Robots")

IF (NOT WIN32)
    INCLUDE(FindPkgConfig)
ENDIF ()

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
    actionlib
    actionlib_msgs
    knowrob
)

add_message_files(
    DIRECTORY msg
    FILES
    KeyValuePair.msg
    GraphQueryMessage.msg
    GraphAnswerMessage.msg
    EventToken.msg
)
add_action_files(
    DIRECTORY action
    FILES
    AskAll.action
    AskOne.action
    AskIncremental.action
    AskIncrementalNextSolution.action
    Tell.action
)
add_service_files(
    DIRECTORY srv
    FILES
    AskIncrementalFinish.srv
)
generate_messages(DEPENDENCIES
    std_msgs
    actionlib_msgs
)

catkin_package(CATKIN_DEPENDS roscpp roslib knowrob)

include_directories(include ${catkin_INCLUDE_DIRS})

## ROS Interface ###
add_executable(knowrob-ros
    src/ROSInterface.cpp
)
add_dependencies(knowrob-ros ${catkin_EXPORTED_TARGETS})

target_link_libraries(knowrob-ros ${catkin_LIBRARIES})
install(TARGETS knowrob-ros RUNTIME DESTINATION bin)
