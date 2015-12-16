cmake_minimum_required(VERSION 2.8.3)
project(uibk_moveit_adapter)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    actionlib
    controller_manager
)

find_package(Eigen 3 REQUIRED)

catkin_package(
    CATKIN_DEPENDS roscpp controller_manager actionlib
    DEPENDS ${Eigen_Libraries}
)

include_directories(
    include
    ${Boost_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

add_executable(hardware_adapter
    src/UibkRobotHW.cpp
    src/UibkControllerManager.cpp)
target_link_libraries(hardware_adapter ${catkin_LIBRARIES} ${Eigen_Libraries})

add_executable(test_gripper_action test/test_gripper_action_client.cpp)
target_link_libraries(test_gripper_action ${catkin_LIBRARIES} ${Eigen_Libraries})
