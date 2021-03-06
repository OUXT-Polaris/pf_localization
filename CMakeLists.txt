cmake_minimum_required(VERSION 3.5)
project(pf_localization)

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
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(quaternion_operation REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs_data_buffer REQUIRED)
find_package(data_buffer REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
)

add_library(particle_filter SHARED
  src/particle_filter.cpp
  src/twist_estimator.cpp
)
ament_target_dependencies(particle_filter
  rclcpp
  rclcpp_components
  geometry_msgs
  tf2
  tf2_ros
  tf2_msgs
  tf2_geometry_msgs
  quaternion_operation
  visualization_msgs
  geometry_msgs_data_buffer
  data_buffer)
target_link_libraries(particle_filter ${Boost_LIBRARIES})

add_library(pf_localization_component SHARED
  src/pf_localization_component.cpp
)
ament_target_dependencies(pf_localization_component
  rclcpp
  rclcpp_components
  geometry_msgs
  tf2
  tf2_ros
  tf2_msgs
  tf2_geometry_msgs
  quaternion_operation
  visualization_msgs
  geometry_msgs_data_buffer
  data_buffer)
target_link_libraries(pf_localization_component particle_filter)
rclcpp_components_register_nodes(pf_localization_component
  "pf_localization::PfLocalizationComponent")

add_executable(pf_localization_node
  src/pf_localization_node.cpp
)
target_link_libraries(pf_localization_node pf_localization_component glog)
ament_target_dependencies(pf_localization_node
  rclcpp
  rclcpp_components
  geometry_msgs
  tf2
  tf2_ros
  tf2_msgs
  tf2_geometry_msgs
  quaternion_operation
  visualization_msgs
  geometry_msgs_data_buffer
  data_buffer)

# install executables/libs
#install(TARGETS pf_localization_node
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS particle_filter pf_localization_component
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

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

ament_export_include_directories(include)
ament_package()