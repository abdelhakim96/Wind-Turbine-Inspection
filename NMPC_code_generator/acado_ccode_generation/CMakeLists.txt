cmake_minimum_required(VERSION 2.8.3)
project(acado_ccode_generation)
set(PROJECT_NAME acado_ccode_generation)
set(ROS_BUILD_TYPE Debug)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
elseif(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 ${CMAKE_CXX_FLAGS_DEBUG} -O1")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++14 or C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs roslib nav_msgs tf ros_acado_bridge
)


find_package(Eigen3 3.0.1 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp tf geometry_msgs
)

#############
# LIBRARIES #
#############

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include

  ${ros_acado_bridge_INCLUDE_DIRS}
  ${ros_acado_bridge_INCLUDE_DIRS}/acado
)

############
# BINARIES #
############
   
option(QUADROTOR "flag to compile quadrotor codes" ON)
if(QUADROTOR)
    # NMPC C code generation
    add_executable(nmpc_pc_learning_ccode 
	   src/quadrotor/nmpc_pc_learning_ccode.cpp
	)
	target_link_libraries(nmpc_pc_learning_ccode
      	   ${catkin_LIBRARIES}
	   acado_toolkit_s
	)

	add_executable(nmpc_pc_quadrotor_ccode 
	   src/quadrotor/nmpc_pc_quadrotor_ccode.cpp
	)
	target_link_libraries(nmpc_pc_quadrotor_ccode
      	   ${catkin_LIBRARIES}
	   acado_toolkit_s
	)
	
endif(QUADROTOR)

