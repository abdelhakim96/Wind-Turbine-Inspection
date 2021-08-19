cmake_minimum_required(VERSION 2.8.12)
project(offboard_control)
#add_subdirectory(thirdparty/restclient-cpp)
add_definitions(-std=c++11)
# How to print variables:
# message(STATUS "PRINTING PROJECT NAME: ${PROJECT_NAME}")
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        tf
        mavros
        mavros_extras
        mavros_msgs
        )
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES offboard_control
        CATKIN_DEPENDS roscpp rospy std_msgs mavros_msgs geometry_msgs sensor_msgs tf
)

#############
# LIBRARIES #
#############
MESSAGE(STATUS "JSON ${Json_INCLUDE_DIRS}")
include_directories(
        include
        ${Boost_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
        src/offboard_control.cpp
        )
add_dependencies(offboard_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

############
# BINARIES #
############

add_executable(offboard_control_node
        src/offboard_control_node.cpp
        )
add_dependencies(offboard_control_node offboard_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(offboard_control_node ${PROJECT_NAME} ${catkin_LIBRARIES})
