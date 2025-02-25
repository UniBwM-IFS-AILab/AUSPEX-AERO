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
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(auspex_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(auspex_fci REQUIRED)
find_package(msg_context_cpp REQUIRED)

#Variable to include cpp source file if sim is true
set(USE_PX4_SIMULATION FALSE)

if (DEFINED ENV{OBC_MODE} AND "$ENV{OBC_MODE}" STREQUAL "PX4_SIMULATION")
  set(USE_PX4_SIMULATION TRUE) 

  set(AIRSIM_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../../AirSim)
  add_subdirectory("${AIRSIM_ROOT}/cmake/rpclib_wrapper" rpclib_wrapper)
  add_subdirectory("${AIRSIM_ROOT}/cmake/AirLib" AirLib)
  add_subdirectory("${AIRSIM_ROOT}/cmake/MavLinkCom" MavLinkCom)

  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra -Wnoexcept -Wstrict-null-sentinel")
  set(CXX_EXP_LIB "-nostdinc++ -I/usr/include/c++/8 -I/usr/include/x86_64-linux-gnu/c++/8 -nodefaultlibs -l/usr/lib/x86_64-linux-gnu/libc++.so -l/usr/lib/x86_64-linux-gnu/libc++abi.so -lm -lc -lgcc_s -lgcc -lstdc++fs -fmax-errors=10")
  set(RPC_LIB_INCLUDES " ${AIRSIM_ROOT}/external/rpclib/rpclib-2.3.0/include")
  set(RPC_LIB rpc)
elseif (DEFINED ENV{OBC_MODE} AND "$ENV{OBC_MODE}" STREQUAL "PX4")

  include_directories(/usr/local/include/libcamera)
  # Specify the library paths
  set(LIBCAMERA_LIBRARIES
      /usr/local/lib/aarch64-linux-gnu/libcamera.so
      /usr/local/lib/aarch64-linux-gnu/libcamera-base.so
      /usr/local/lib/aarch64-linux-gnu/libcamera_app.so
  )

  include_directories(/usr/local/include/rpicam-apps)
  # Specify the library paths
  set(RPICAM_LIBRARIES
    /usr/local/lib/aarch64-linux-gnu/rpicam_app.so
  )
endif()

#################
# Setup targets #
#################

include_directories(include
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${auspex_fci_INCLUDE_DIRS}
  ${msg_context_cpp_INCLUDE_DIRS}
)


if (DEFINED ENV{OBC_MODE} AND "$ENV{OBC_MODE}" STREQUAL "PX4_SIMULATION")
  include_directories(include
    ${RPC_LIB_INCLUDES}
    ${AIRSIM_ROOT}/AirLib/deps/eigen3
    ${AIRSIM_ROOT}/AirLib/include
    ${AIRSIM_ROOT}/MavLinkCom/include
    ${AIRSIM_ROOT}/MavLinkCom/common_utils
  )

endif()


add_library(auspex_obc_lib
								include/auspex_obc/drone_state_publisher.hpp
                include/auspex_obc/Vector3D.h
                include/auspex_obc/obc_node.h
                
                $<$<NOT:$<BOOL:${USE_PX4_SIMULATION}>>:src/cam_rpi.cpp>
                $<$<NOT:$<BOOL:${USE_PX4_SIMULATION}>>:src/cam_rpi5.cpp>
                $<$<BOOL:${USE_PX4_SIMULATION}>:src/cam_sim.cpp>
                )

# auspex_obc
add_executable(auspex_obc src/obc_main.cpp src/obc_node.cpp)

if(DEFINED ENV{OBC_MODE} AND "$ENV{OBC_MODE}" STREQUAL "PX4_SIMULATION")
  target_compile_definitions(auspex_obc PRIVATE INCLUDE_SIM_HEADER)
endif()

set(DEPENDENCIES
    rclcpp
    tf2_ros
    px4_msgs
    rclcpp_action
    Eigen3
    auspex_msgs
    OpenCV
    cv_bridge
    sensor_msgs
    auspex_fci
    msg_context_cpp
)

ament_target_dependencies(auspex_obc_lib ${DEPENDENCIES})

target_link_libraries(auspex_obc
  ${rclcpp_LIBRARIES}
  ${Boost_LIBRARIES} 
  auspex_obc_lib 
  Eigen3::Eigen 
  ${OpenCV_LIBS} 
  $<$<BOOL:${USE_PX4_SIMULATION}>:AirLib>
  $<$<NOT:$<BOOL:${USE_PX4_SIMULATION}>>:${LIBCAMERA_LIBRARIES}>
  $<$<NOT:$<BOOL:${USE_PX4_SIMULATION}>>:${RPICAM_LIBRARIES}>
  nlohmann_json::nlohmann_json
)


install(TARGETS auspex_obc DESTINATION lib/${PROJECT_NAME} )

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
