cmake_minimum_required(VERSION 3.8)
project(auspex_obc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-w)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(MAVSDK REQUIRED)

find_package(auspex_msgs REQUIRED)
find_package(auspex_fci REQUIRED)
find_package(auspex_cam REQUIRED)


################################
# Get dependencies on cam libs #
################################

if(NOT DEFINED ENV{CAM_TYPE})
  message(FATAL_ERROR "CAM_TYPE is not set")
endif()
set(CAM_LIST $ENV{CAM_TYPE})
list(GET CAM_LIST 0 FIRST_CAM)

#################
# Setup targets #
#################

include_directories(include
  ${Boost_INCLUDE_DIRS}
  ${auspex_fci_INCLUDE_DIRS}
)


add_library(auspex_obc_lib
  include/auspex_obc/platform_state_publisher.hpp
  include/auspex_obc/ext_data_listener.hpp
  include/auspex_obc/Vector3D.h
  src/obc_node.cpp
  src/ext_data_listener.cpp
)

set(DEPENDENCIES
  rclcpp
  std_msgs
  tf2_ros
  rclcpp_action
  Eigen3
  auspex_msgs
  sensor_msgs
  geographic_msgs
  MAVSDK
  auspex_fci
  auspex_cam
)

ament_target_dependencies(auspex_obc_lib
  ${DEPENDENCIES}
)


target_link_libraries(auspex_obc_lib
  ${rclcpp_LIBRARIES}
  ${Boost_LIBRARIES}
  Eigen3::Eigen
  nlohmann_json::nlohmann_json
  MAVSDK::mavsdk
)

# auspex_obc
add_executable(auspex_obc src/obc_main.cpp)

if("${FIRST_CAM}" STREQUAL "SIM_UE")
  target_compile_definitions(auspex_obc_lib PRIVATE SWITCH_CAM_HEADER=1)
  target_compile_definitions(auspex_obc PRIVATE SWITCH_CAM_HEADER=1)
elseif("${FIRST_CAM}" STREQUAL "RPI5")
  target_compile_definitions(auspex_obc_lib PRIVATE SWITCH_CAM_HEADER=2)
  target_compile_definitions(auspex_obc PRIVATE SWITCH_CAM_HEADER=2)
elseif("${FIRST_CAM}" STREQUAL "ZT30")
  target_compile_definitions(auspex_obc_lib PRIVATE SWITCH_CAM_HEADER=4)
  target_compile_definitions(auspex_obc PRIVATE SWITCH_CAM_HEADER=4)
elseif("${FIRST_CAM}" STREQUAL "SIM_IS")
  target_compile_definitions(auspex_obc_lib PRIVATE SWITCH_CAM_HEADER=5)
  target_compile_definitions(auspex_obc PRIVATE SWITCH_CAM_HEADER=5)
elseif("${FIRST_CAM}" STREQUAL "NONE")
  target_compile_definitions(auspex_obc_lib PRIVATE SWITCH_CAM_HEADER=0)
  target_compile_definitions(auspex_obc PRIVATE SWITCH_CAM_HEADER=0)
endif()


target_link_libraries(auspex_obc
  auspex_obc_lib
)

install(TARGETS
  auspex_obc
  auspex_obc_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
# Install launch files.
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

############
# Testing ##
############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
