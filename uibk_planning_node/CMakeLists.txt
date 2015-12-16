cmake_minimum_required(VERSION 2.8.3)
project(uibk_planning_node)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

## find_package(catkin REQUIRED COMPONENTS moveit_core moveit_ros_planning moveit_ros_planning_interface pluginlib actionlib cmake_modules)
find_package(catkin REQUIRED
    roscpp
    actionlib
    moveit_msgs
    moveit_core
    geometry_msgs
    uibk_moveit_adapter
    uibk_robot_moveit_config
    moveit_ros_planning_interface
)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS ${Boost_LIBRARIES} ${Eigen3_Libraries}
)

find_package(Eigen3 REQUIRED)

include_directories(
   include
   ${catkin_INCLUDE_DIRS}
   ${Boost_INCLUDE_DIRS}
   ${EIGEN3_INCLUDE_DIR}
)

AUX_SOURCE_DIRECTORY(src/ UIBKPLANNINGSRC)

add_library(${PROJECT_NAME}
    ${UIBKPLANNINGSRC}
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${Boost_LIBRARIES})

add_executable(move_goal src/move_goal.cpp)
target_link_libraries(move_goal ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${Boost_LIBRARIES})

add_executable(kinematics_helper_tests tests/kinematics_helper_tests.cpp)
add_dependencies(kinematics_helper_tests ${PROJECT_NAME})
target_link_libraries(kinematics_helper_tests ${PROJECT_NAME} ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${Boost_LIBRARIES})

add_executable(test_trajectory_planner tests/test_trajectory_planner.cpp)
add_dependencies(test_trajectory_planner ${PROJECT_NAME})
target_link_libraries(test_trajectory_planner ${PROJECT_NAME} ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${Boost_LIBRARIES})

add_executable(test_ccontrol tests/test_ccontrol.cpp)
add_dependencies(test_ccontrol ${PROJECT_NAME})
target_link_libraries(test_ccontrol ${PROJECT_NAME} ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${Boost_LIBRARIES})
