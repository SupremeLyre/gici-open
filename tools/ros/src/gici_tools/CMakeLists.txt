cmake_minimum_required(VERSION 3.8)
if(POLICY CMP0079)
  cmake_policy(SET CMP0079 NEW)
endif()
project(gici_tools)

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
find_package(rosbag2_cpp REQUIRED)
find_package(gici_ros REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_subdirectory(../../../.. gici.out)

set(GICI_THIRD_PARTY_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/gici.out/third_party/lib)
foreach(target fast vikit_common svo rtklib)
  if(TARGET ${target})
    set_target_properties(${target} PROPERTIES
      ARCHIVE_OUTPUT_DIRECTORY ${GICI_THIRD_PARTY_OUTPUT_DIR}
      LIBRARY_OUTPUT_DIRECTORY ${GICI_THIRD_PARTY_OUTPUT_DIR}
      RUNTIME_OUTPUT_DIRECTORY ${GICI_THIRD_PARTY_OUTPUT_DIR}
    )
  endif()
endforeach()

if(TARGET rtklib)
  target_link_libraries(rtklib m)
endif()

# Add definitions here to ensure correct memory allocation in RTKLIB
add_definitions(-DENAGLO -DENACMP -DENAGAL -DENAQZS -DENAIRN -DNFREQ=3 -DNEXOBS=3 -DDLL)

# Sources
aux_source_directory(src/ros_utility DIR_UTILITIES)

set(GICI_TOOLS_ROS_DEPS
  rclcpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  cv_bridge
  rosbag2_cpp
  gici_ros
)

function(add_gici_tool target)
  add_executable(${target} ${ARGN})
  ament_target_dependencies(${target} ${GICI_TOOLS_ROS_DEPS})
  target_link_libraries(${target}
    gici
    ${OpenCV_LIBS}
  )
endfunction()

# Executables
add_gici_tool(publish_ie_poses
              src/publish_ie_poses.cpp ${DIR_UTILITIES})

add_gici_tool(publish_tum_poses
              src/publish_tum_poses.cpp ${DIR_UTILITIES})

add_gici_tool(publish_nmea_poses
              src/publish_nmea_poses.cpp ${DIR_UTILITIES})

add_gici_tool(imupack_to_rosbag
              src/imupack_to_rosbag.cpp)
              
add_gici_tool(imagepack_to_rosbag
              src/imagepack_to_rosbag.cpp)

add_gici_tool(nmea_to_rosbag_navsatfix
              src/nmea_to_rosbag_navsatfix.cpp ${DIR_UTILITIES})

add_gici_tool(gici_files_to_rosbag
              src/gici_files_to_rosbag.cpp ${DIR_UTILITIES})

install(TARGETS
  publish_ie_poses
  publish_tum_poses
  publish_nmea_poses
  imupack_to_rosbag
  imagepack_to_rosbag
  nmea_to_rosbag_navsatfix
  gici_files_to_rosbag
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
install(DIRECTORY option
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
