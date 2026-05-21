cmake_minimum_required(VERSION 3.8)
if(POLICY CMP0079)
  cmake_policy(SET CMP0079 NEW)
endif()
project(gici_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-fpermissive>)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -Wall -O3 -Wno-unused-but-set-variable")

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(OpenCV 4 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosidl_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  msg/GlonassEphemeris.msg
  msg/GnssAntennaPosition.msg
  msg/GnssEphemerides.msg
  msg/GnssEphemeris.msg
  msg/GnssIonosphereParameter.msg
  msg/GnssObservation.msg
  msg/GnssObservations.msg
  msg/GnssSsrCodeBias.msg
  msg/GnssSsrCodeBiases.msg
  msg/GnssSsrEphemerides.msg
  msg/GnssSsrEphemeris.msg
  msg/GnssSsrPhaseBias.msg
  msg/GnssSsrPhaseBiases.msg
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_subdirectory(../../.. gici.out)
if(TARGET rtklib)
  target_link_libraries(rtklib m)
endif()

add_definitions(-DENAGLO -DENACMP -DENAGAL -DENAQZS -DENAIRN -DNFREQ=3 -DNEXOBS=3 -DDLL)

aux_source_directory(src/ros_interface DIR_ROS_TOOLS)

add_executable(gici_ros_main
  src/gici_ros_main.cpp
  ${DIR_ROS_TOOLS}
)
target_include_directories(gici_ros_main PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
ament_target_dependencies(gici_ros_main
  rclcpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  cv_bridge
)
target_link_libraries(gici_ros_main
  gici
  ${OpenCV_LIBS}
  ${cpp_typesupport_target}
)

install(TARGETS gici_ros_main
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS gici rtklib fast vikit_common svo
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch option rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime std_msgs)
ament_package()
