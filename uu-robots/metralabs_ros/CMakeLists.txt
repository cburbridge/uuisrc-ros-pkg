cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#set(CMAKE_INCLUDE_PATH )
#set(CMAKE_INCLUDE_PATH )
set (ML_ROOT_DIR /opt/MetraLabs)
include_directories(${ML_ROOT_DIR}/MetraLabsBase/include ${ML_ROOT_DIR}/MLRobotic/include)
#set(CMAKE_LIBRARY_PATH /opt/MetraLabs/lib)
link_directories (${ML_ROOT_DIR}/lib)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_REENTRANT -DM_LINUX -Wall")

#set (CMAKE_INCLUDE_PATH "$PROJECT_SOURCE_DIR/msg_gen/cpp/include/Foo" ${CMAKE_INCLUDE_PATH})

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rospack(robot_srvs)
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#LINK_DIRECTORIES( ${LINK_DIRECTORIES} )
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

#rosbuild_add_executable(ScitosServer src/ScitosServer.cpp src/PowerCube.cpp src/ScitosBase.cpp src/joint_spline_trajectory_controller.cpp)
rosbuild_add_executable(ScitosServer src/ScitosServer.cpp src/PowerCube.cpp src/ScitosBase.cpp)

#rosbuild_add_executable(tester src/tester.cpp)
#target_link_libraries(example ${PROJECT_NAME})

target_link_libraries(ScitosServer MLRoboticRobot)
target_link_libraries(ScitosServer MLRoboticBase)
target_link_libraries(ScitosServer MetraLabsBase)
target_link_libraries(ScitosServer MLRoboticHardwareSCHUNK)
target_link_libraries(ScitosServer MLRoboticCAN)
target_link_libraries(ScitosServer MLRoboticHardware)
target_link_libraries(ScitosServer MLRoboticSCITOS)
target_link_libraries(ScitosServer MLRoboticSystem)

