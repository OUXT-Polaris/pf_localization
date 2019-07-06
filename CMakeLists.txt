cmake_minimum_required(VERSION 2.8.3)
project(pf_localization)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  quaternion_operation
  roscpp
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs_data_buffer
  visualization_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pf_localization
  CATKIN_DEPENDS geometry_msgs quaternion_operation roscpp geometry_msgs_data_buffer visualization_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_executable(pf_localization_node 
  src/pf_localization_node.cpp 
  src/pf_localization.cpp 
  src/particle_filter.cpp
  src/twist_estimator.cpp)
target_link_libraries(pf_localization_node ${catkin_LIBRARIES} glog)

# install executables/libs
install(TARGETS pf_localization_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install headers
install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS)

# Install Python scripts
set(PYTHON_SCRIPTS scripts/initial_pose_setter.py)
foreach(scripts ${PYTHON_SCRIPTS})
        catkin_install_python(PROGRAMS ${scripts}
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach(scripts)