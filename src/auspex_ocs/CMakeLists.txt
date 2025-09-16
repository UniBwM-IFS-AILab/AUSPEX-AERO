cmake_minimum_required(VERSION 3.8)
project(auspex_ocs)

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
find_package(msg_context_cpp REQUIRED)

if(DEFINED ENV{MSG_CONTEXT} AND "$ENV{MSG_CONTEXT}" STREQUAL "MENTHON")
  set(MSG_CONTEXT_MENTHON TRUE)
  find_package(hat_menthon REQUIRED)
else()
  set(MSG_CONTEXT_MENTHON FALSE)
endif()

if(hat_menthon_FOUND AND MSG_CONTEXT_MENTHON)
  add_compile_definitions(HAVE_HAT_MENTHON)
endif()

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
  src/cam_publisher_base.cpp
  $<$<STREQUAL:${FIRST_CAM},SIM_UE>:src/cam_publisher_simUE.cpp>
  $<$<STREQUAL:${FIRST_CAM},SIM_UE>:src/airsim_connection_manager.cpp>
  $<$<STREQUAL:${FIRST_CAM},SIM_IS>:src/cam_publisher_simIS.cpp>
  $<$<STREQUAL:${FIRST_CAM},RPI5>:src/cam_publisher_rpi5.cpp>
  $<$<STREQUAL:${FIRST_CAM},RPI>:src/cam_publisher_rpi.cpp>
  $<$<STREQUAL:${FIRST_CAM},ZT30>:src/cam_publisher_mk_smart.cpp>
  $<$<STREQUAL:${FIRST_CAM},NONE>:src/cam_publisher_empty.cpp>
)

add_library(auspex_ocs SHARED ${SRC})

target_include_directories(auspex_ocs
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
    ${cv_bridge_INCLUDE_DIRS}
    ${msg_context_cpp_INCLUDE_DIRS}
    ${auspex_fci_INCLUDE_DIRS}
    ${geographic_msgs_INCLUDE_DIRS}

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
 target_link_libraries(auspex_ocs
   PUBLIC
    rclcpp::rclcpp
    msg_context_cpp::msg_context_cpp
    cv_bridge::cv_bridge
    image_transport::image_transport
    Eigen3::Eigen
    ${OpenCV_LIBS}
    MAVSDK::mavsdk
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
  DESTINATION include/auspex_ocs
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)
install(
  TARGETS auspex_ocs
  EXPORT  export_auspex_ocs
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/auspex_ocs
)

ament_export_targets(export_auspex_ocs HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  msg_context_cpp
  auspex_fci
  auspex_msgs
  geographic_msgs
  cv_bridge
  OpenCV
  image_transport
)
ament_package()