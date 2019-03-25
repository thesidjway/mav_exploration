cmake_minimum_required(VERSION 3.5)
project(mav_exploration)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -ffast-math -march=native -fopenmp")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++11 -ffast-math -march=native -Ofast -O3")
## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        image_transport
        cv_bridge
        tf2
        tf2_ros
        message_filters
        rosbag
        )

find_package( OpenCV REQUIRED)
find_package( yaml-cpp REQUIRED )
find_package( Eigen3 REQUIRED)

## Declare a catkin package
catkin_package(
        CATKIN_DEPENDS roscpp rospy std_msgs image_transport eigen_conversions cv_bridge tf2 tf2_ros message_filters rosbag nbvplanner
        INCLUDE_DIRS include
        LIBRARIES mvtlc
)



include_directories(
        include/
        ${catkin_INCLUDE_DIRS}
        ${roscpp_INCLUDE_DIRS}
        ${rospy_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${yaml-cpp_INCLUDE_DIRS}
        ../../devel/include
)

add_executable(mav_exploration_node src/mav_exploration_node.cc)

target_link_libraries(mav_exploration_node ${OpenCV_LIBRARIES} yaml-cpp ${catkin_LIBRARIES})
