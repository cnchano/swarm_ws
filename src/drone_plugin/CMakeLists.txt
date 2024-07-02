cmake_minimum_required(VERSION 3.5)
project(drone_plugin)

set(DEPENDENCIES
    rosidl_default_generators 
    rclcpp
    gazebo_ros 
    tf2_ros
    builtin_interfaces
    std_msgs sensor_msgs geometry_msgs
    drone_common
)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(DEPENDENCY ${DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

# add_library(drone_force_plugin SHARED src/drone_force_plugin.cpp)

# rosidl_target_interfaces(drone_force_plugin 
#   ${PROJECT_NAME} "rosidl_typesupport_cpp"
# )
# target_include_directories(${PROJECT_NAME}
#   PUBLIC
#   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
# specific order: dependents before dependencies
# ament_target_dependencies(drone_force_plugin
#   "rclcpp"
#   "tf2_ros"
#   "rosidl_typesupport_cpp"
#   "drone_common"
#   "gazebo_ros"
# )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# gazebo_ros_drone_force
add_library(gazebo_ros_drone_force SHARED
  src/gazebo_ros_drone_force.cpp
)
target_include_directories(gazebo_ros_drone_force PUBLIC include)
ament_target_dependencies(gazebo_ros_drone_force
  "gazebo_dev"
  "gazebo_ros"
  "geometry_msgs"
  "rclcpp"
#  
  "rosidl_typesupport_cpp"
  "drone_common"
# 
  "tf2"
  "tf2_ros"
)
ament_export_libraries(gazebo_ros_drone_force)

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_package()

install(TARGETS
  gazebo_ros_drone_force
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)