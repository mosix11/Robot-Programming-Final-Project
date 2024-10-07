cmake_minimum_required(VERSION 3.8)
project(dmap_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# include_directories(/usr/include/eigen3)
find_package(Eigen3 REQUIRED)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# find_package(pcl_conversions REQUIRED)
# find_package(PCL REQUIRED COMPONENTS common io registration)
# Include directories
include_directories(
  # ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(dmap_localizer src/dmap_localizer.cpp)
# add_executable(robot_relocator src/robot_relocator.cpp)
# target_link_libraries(dmap_localization_node
#   ${PCL_LIBRARIES}
# )

ament_target_dependencies(dmap_localizer
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  # pcl_conversions
  # PCL
  Eigen3
)

# ament_target_dependencies(robot_relocator
#   rclcpp
#   geometry_msgs
#   tf2_ros
# )

install(TARGETS dmap_localizer
  DESTINATION lib/${PROJECT_NAME})

# install(TARGETS
#   robot_relocator
#   DESTINATION lib/${PROJECT_NAME})



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
