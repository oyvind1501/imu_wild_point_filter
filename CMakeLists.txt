cmake_minimum_required(VERSION 3.5.1)

project(imu_wild_point_filter
        VERSION 0.1
        LANGUAGES CXX)

add_compile_options(-std=c++14)

find_package(Eigen3 REQUIRED)

add_subdirectory(src)
#add_subdirectory(apps/test)

find_package(roscpp QUIET)
if (roscpp_FOUND)
    # TODO: remove unused components
find_package(catkin REQUIRED COMPONENTS
        #geometry_msgs
        roscpp
        sensor_msgs
)

catkin_package(
        # INCLUDE_DIRS include
        # LIBRARIES 
        CATKIN_DEPENDS roscpp sensor_msgs  
        # DEPENDS eigen system_lib
)

include_directories(
	PRIVATE
        apps/ros
	PUBLIC
        ${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
)

add_executable(imu_wild_point_filter apps/ros/ros_node.cpp)

target_link_libraries(imu_wild_point_filter PRIVATE imu_wild_point_filter_lib ${catkin_LIBRARIES})


endif ()
