cmake_minimum_required(VERSION 3.5)
project(ublox_read_2)

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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CfgValGetType.msg"
  "msg/Ephemeris.msg"
  "msg/GlonassEphemeris.msg"
  "msg/Observation.msg"
  "msg/ObsVec.msg"
  "msg/PositionVelocityTime.msg"
  "msg/PosVelEcef.msg"
  "msg/RelPos.msg"
  "msg/RelPosFlags.msg"
  "msg/RTCMInput.msg"
  "msg/Satellite.msg"
  "msg/SatelliteStatus.msg"
  "msg/SurveyStatus.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost)
add_subdirectory(lib/async_comm)
set(async_comm_LIBRARIES async_comm)
set(async_comm_INCLUDE_DIRS lib/async_comm/include;${Boost_INCLUDE_DIRS})


message("async_comm_INCLUDE_DIRS: ${async_comm_INCLUDE_DIRS}\n")

add_library(UBLOX
    src/ublox.cpp
	    src/parsers/ubx_maps.cpp
    src/parsers/ubx.cpp
    src/parsers/rtcm.cpp
    src/parsers/nav.cpp
    src/utctime.cpp
)

message("UBLOX Boost include dirs: ${Boost_INCLUDE_DIRS}")
target_link_libraries(UBLOX ${async_comm_LIBRARIES} ${gnss_utils_LIBRARIES})
target_include_directories(UBLOX PUBLIC include ${async_comm_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

ament_package()