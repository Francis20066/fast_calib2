cmake_minimum_required(VERSION 3.8)
project(fast_calib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(CMAKE_BUILD_TYPE Release)
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
endif()

add_executable(fast_calib_node src/main.cpp)
set_target_properties(fast_calib_node PROPERTIES OUTPUT_NAME fast_calib)
ament_target_dependencies(fast_calib_node rclcpp sensor_msgs pcl_conversions livox_ros_driver2 rosbag2_cpp)
target_link_libraries(fast_calib_node ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(lidar_center_test_node src/lidar_center_test.cpp)
set_target_properties(lidar_center_test_node PROPERTIES OUTPUT_NAME lidar_center_test)
ament_target_dependencies(lidar_center_test_node rclcpp sensor_msgs pcl_conversions livox_ros_driver2 rosbag2_cpp)
target_link_libraries(lidar_center_test_node ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(multi_fast_calib_node src/multi_scene.cpp)
set_target_properties(multi_fast_calib_node PROPERTIES OUTPUT_NAME multi_fast_calib)
ament_target_dependencies(multi_fast_calib_node rclcpp pcl_conversions)
target_link_libraries(multi_fast_calib_node ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS fast_calib_node lidar_center_test_node multi_fast_calib_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config rviz_cfg
  DESTINATION share/${PROJECT_NAME})

ament_package()
