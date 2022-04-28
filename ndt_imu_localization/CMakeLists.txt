cmake_minimum_required(VERSION 3.0.2)
project(ndt_imu_localization)

add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED
    pcl_ros
    std_msgs
    sensor_msgs
    roscpp
    ndt_omp
    cv_bridge
)
find_package(OpenMP)
find_package(GTSAM REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(PCL REQUIRED QUIET)
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ndt_imu_localization
#  CATKIN_DEPENDS other_catkin_pkg
  DEPENDS GTSAM PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
)

link_directories(
	include
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
)


# Declare a C++ library
add_library(pointcloud_filter
  src/pointcloud_filter.cpp
)



add_executable(pointcloud_filter_node src/pointcloud_filter_node.cpp src/pointcloud_filter.cpp)
target_link_libraries(pointcloud_filter_node
  ${catkin_LIBRARIES}
  pointcloud_filter
)

add_executable(ndt_matching_node src/ndt_matching.cpp)
target_link_libraries(ndt_matching_node
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} 
)

add_executable(imu_preintegration src/imu_preintegration.cpp)
target_link_libraries(imu_preintegration
  ${catkin_LIBRARIES}  ${PCL_LIBRARIES} ${OpenCV_LIBRARIES}  gtsam
)

add_executable(pointcloud_splicing src/pointcloud_splicing.cpp)
target_link_libraries(pointcloud_splicing
  ${catkin_LIBRARIES}  ${PCL_LIBRARIES} 
)

add_executable(publish_pcd_map src/publish_pcd_map.cpp)
target_link_libraries(publish_pcd_map
  ${catkin_LIBRARIES}  ${PCL_LIBRARIES} 
)






