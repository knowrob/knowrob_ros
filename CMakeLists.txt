cmake_minimum_required(VERSION 3.5.0)
set(CMAKE_CXX_FLAGS "-Wswitch -std=c++0x -pthread ${CMAKE_CXX_FLAGS} -Wno-undef -lspdlog")
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
find_package(spdlog REQUIRED)
find_package(rostest REQUIRED)

## ==============================
## Python setup
## ==============================
catkin_python_setup()

## =============================
## ROS message generation
## =============================
add_message_files(
    DIRECTORY msg
    FILES
    KeyValuePair.msg
    ModalFrame.msg
    Triple.msg
    TellMessage.msg
    GraphQueryMessage.msg
    GraphAnswerMessage.msg
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

## =============================
## Catkin package
## =============================
catkin_package(CATKIN_DEPENDS roscpp roslib rospy knowrob message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS})

## ROS Interface ###
add_executable(knowrob-ros
    src/ROSInterface.cpp
)
add_dependencies(knowrob-ros 
    ${catkin_EXPORTED_TARGETS}
    knowrob_ros_generate_messages_cpp
)

target_link_libraries(knowrob-ros ${catkin_LIBRARIES} spdlog::spdlog)

## =============================
## Python library installation
## =============================

# Install Python library (e.g., knowrob_ros_lib.py)
# With this:
install(FILES
  src/knowrob_ros_lib.py
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

# Install Python scripts (e.g., test_knowrob_ros_lib.py)
catkin_install_python(PROGRAMS
  scripts/test_knowrob_ros_lib.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## =============================
## Install binaries
## =============================

install(TARGETS knowrob-ros RUNTIME DESTINATION bin)

## =============================
## Install test files
## =============================

if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(
    test/test_knowrob_ros_lib.test
    DEPENDENCIES ${catkin_EXPORTED_TARGETS}
  )
endif()