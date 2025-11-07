cmake_minimum_required(VERSION 3.8)
project(auspex_cam)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui xphoto)
find_package(image_transport REQUIRED)
find_package(auspex_fci REQUIRED)
find_package(auspex_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(MAVSDK REQUIRED)

# GStreamer for RTSP streaming
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED 
  gstreamer-1.0 
  gstreamer-rtsp-server-1.0 
  gstreamer-app-1.0
)

set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)

if(NOT DEFINED ENV{CAM_TYPE})
  message(FATAL_ERROR "CAM_TYPE is not set")
endif()
set(CAM_LIST $ENV{CAM_TYPE})
list(GET CAM_LIST 0 FIRST_CAM)

if(FIRST_CAM STREQUAL "SIM_IS")

  find_package(PkgConfig REQUIRED)
  pkg_check_modules(ZMQ libzmq)

elseif(FIRST_CAM STREQUAL "SIM_UE")

  set(AIRSIM_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../../../AirSim CACHE PATH "")
  add_subdirectory("${AIRSIM_ROOT}/cmake/rpclib_wrapper" rpclib_wrapper)
  add_subdirectory("${AIRSIM_ROOT}/cmake/MavLinkCom"     MavLinkCom)
  add_subdirectory("${AIRSIM_ROOT}/cmake/AirLib"         AirLib)

elseif(FIRST_CAM STREQUAL "RPI5")

  include_directories(/usr/local/include/libcamera)
  set(LIBCAMERA_INCLUDE_DIRS /usr/local/include/libcamera)
  set(LIBCAMERA_LIBRARIES /usr/local/lib/aarch64-linux-gnu/libcamera-base.so)
  set(LIBCAMERA_BASE_LIBRARIES /usr/local/lib/aarch64-linux-gnu/libcamera.so)
  
endif()

# sources
set(SRC
  src/camera_controller_base.cpp
  src/stream_handler/stream_handler.cpp
  src/stream_handler/rtsp_server_manager.cpp
  $<$<STREQUAL:${FIRST_CAM},SIM_UE>:src/camera_controller_simUE.cpp>
  $<$<STREQUAL:${FIRST_CAM},SIM_UE>:src/airsim_connection_manager/airsim_connection_manager.cpp>
  $<$<STREQUAL:${FIRST_CAM},SIM_IS>:src/camera_controller_simIS.cpp>
  $<$<STREQUAL:${FIRST_CAM},RPI5>:src/camera_controller_rpi5.cpp>
  $<$<STREQUAL:${FIRST_CAM},ZT30>:src/camera_controller_siyi_zt30.cpp>
  $<$<STREQUAL:${FIRST_CAM},ZT30>:src/siyi_sdk/siyi_sdk.cpp>
)

add_library(auspex_cam SHARED ${SRC})

target_include_directories(auspex_cam
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
    ${cv_bridge_INCLUDE_DIRS}
    ${auspex_fci_INCLUDE_DIRS}
    ${auspex_msgs_INCLUDE_DIRS}
    ${geographic_msgs_INCLUDE_DIRS}
    ${GST_INCLUDE_DIRS}

    $<$<STREQUAL:${FIRST_CAM},SIM_UE>:${AIRSIM_ROOT}/AirLib/include>
    $<$<STREQUAL:${FIRST_CAM},SIM_UE>:${AIRSIM_ROOT}/AirLib/deps/eigen3>
    $<$<STREQUAL:${FIRST_CAM},SIM_UE>:${AIRSIM_ROOT}/MavLinkCom/include>
    $<$<STREQUAL:${FIRST_CAM},SIM_UE>:${AIRSIM_ROOT}/MavLinkCom/common_utils>
    $<$<STREQUAL:${FIRST_CAM},RPI5>:${LIBCAMERA_INCLUDE_DIRS}>
    $<$<STREQUAL:${FIRST_CAM},SIM_IS>:${ZMQ_INCLUDE_DIRS}>
)

# ————————————————————————————————
# link everything
# ————————————————————————————————
 target_link_libraries(auspex_cam
   PUBLIC
    rclcpp::rclcpp
    cv_bridge::cv_bridge
    image_transport::image_transport
    Eigen3::Eigen
    ${OpenCV_LIBS}
    MAVSDK::mavsdk
    ${GST_LIBRARIES}
   PRIVATE
     $<$<STREQUAL:${FIRST_CAM},SIM_UE>:AirLib>
     $<$<STREQUAL:${FIRST_CAM},SIM_UE>:MavLinkCom>
     $<$<STREQUAL:${FIRST_CAM},RPI5>:${LIBCAMERA_LIBRARIES}>
     $<$<STREQUAL:${FIRST_CAM},RPI5>:${LIBCAMERA_BASE_LIBRARIES}>
     $<$<STREQUAL:${FIRST_CAM},SIM_IS>:${ZMQ_LIBRARIES}>
 )

# ————————————————————————————————
# install headers & library
# ————————————————————————————————
install(
  DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)
install(
  DIRECTORY src/
  DESTINATION include/auspex_cam
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)
install(
  TARGETS auspex_cam
  EXPORT  export_auspex_cam
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/auspex_cam
)

ament_export_targets(export_auspex_cam HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  auspex_fci
  auspex_msgs
  geographic_msgs
  cv_bridge
  OpenCV
  image_transport
)
ament_package()