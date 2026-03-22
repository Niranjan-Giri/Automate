cmake_minimum_required(VERSION 3.8)
project(Navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)

add_executable(occupancy_grid src/OccupancyGrid.cpp)
target_include_directories(occupancy_grid PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(occupancy_grid
  rclcpp
  sensor_msgs
  nav_msgs
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions
  pcl_ros
)

add_executable(local_costmap  src/LocalCostmap.cpp)
target_include_directories(local_costmap PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(local_costmap
  rclcpp
  sensor_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  tf2_ros
  tf2_sensor_msgs
)
target_link_libraries(local_costmap ${PCL_LIBRARIES})

INSTALL(TARGETS occupancy_grid
  DESTINATION lib/${PROJECT_NAME}
)
INSTALL(TARGETS local_costmap
  DESTINATION lib/${PROJECT_NAME}
)

INSTALL(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()