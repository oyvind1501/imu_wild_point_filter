
find_package(Eigen3 REQUIRED)
add_library(imu_wild_point_filter_lib
        wild_point_filter.cpp)

target_include_directories(imu_wild_point_filter_lib
        PUBLIC
        ../include/imu_wild_point_filter
	${EIGEN3_INCLUDE_DIR}
        PRIVATE
        ../src
        )



