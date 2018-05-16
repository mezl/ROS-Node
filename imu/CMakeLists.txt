cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
project(imu)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -O3")

find_package(catkin REQUIRED COMPONENTS roscpp message_generation geometry_msgs std_msgs tf sensor_msgs serial)

catkin_package(
  CATKIN_DEPENDS
  serial
  actionlib
  std_msgs
  diagnostic_msgs
)
# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(imu_mpu6050 src/mpu6050_serial_to_imu_node.cpp)
add_executable(imu_mpu6050xz src/mpu6050_serial_to_imu_node2.cpp)
add_executable(imu_transform src/imu_transform.cpp)
target_link_libraries(imu_mpu6050 ${catkin_LIBRARIES})
target_link_libraries(imu_mpu6050xz ${catkin_LIBRARIES})
target_link_libraries(imu_transform ${catkin_LIBRARIES})
#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
