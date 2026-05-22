cmake_minimum_required(VERSION 3.8)
project(gici_ros)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

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
  DEPENDENCIES
  std_msgs
)

ament_export_dependencies(rosidl_default_runtime std_msgs)
ament_package()
