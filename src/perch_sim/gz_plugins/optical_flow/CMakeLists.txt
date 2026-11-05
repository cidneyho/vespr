# The sim's optical flow sensor (README.md). Not a colcon package: run_sim.sh builds it the first
# time the sim starts, against PX4's build (its PX4-OpticalFlow feature tracker) and the Gazebo
# libraries that come with ROS, the ones PX4's own plugins use.
#   cmake -S . -B <build> -DPX4_DIR=$HOME/PX4-Autopilot && cmake --build <build>
cmake_minimum_required(VERSION 3.16)
project(PerchOpticalFlowSystem CXX)
set(CMAKE_CXX_STANDARD 17)

set(PX4_DIR "$ENV{HOME}/PX4-Autopilot" CACHE PATH "PX4-Autopilot, built for px4_sitl")
set(PX4_OPTICAL_FLOW "${PX4_DIR}/build/px4_sitl_default/OpticalFlow/install")
if(NOT EXISTS "${PX4_OPTICAL_FLOW}/lib/libOpticalFlow.so")
  message(FATAL_ERROR "No ${PX4_OPTICAL_FLOW}/lib/libOpticalFlow.so: build PX4 first (make px4_sitl)")
endif()

# The Gazebo that runs is ROS's (/opt/ros/<distro>/opt/gz_*_vendor); prefer it over any in /usr
file(GLOB GZ_VENDOR_DIRS /opt/ros/*/opt/gz_*_vendor)
list(PREPEND CMAKE_PREFIX_PATH ${GZ_VENDOR_DIRS})

find_package(gz-sim8 REQUIRED)
find_package(gz-sensors8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-transport13 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc video)

add_library(${PROJECT_NAME} SHARED
  OpticalFlowSystem.cpp
  OpticalFlowSensor.cpp
  RotationFreeFlow.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${PX4_OPTICAL_FLOW}/include
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} PRIVATE
  gz-sim8::gz-sim8
  gz-sensors8::gz-sensors8
  gz-plugin2::register
  gz-transport13::gz-transport13
  ${OpenCV_LIBS}
  ${PX4_OPTICAL_FLOW}/lib/libOpticalFlow.so
)
set_target_properties(${PROJECT_NAME} PROPERTIES BUILD_RPATH "${PX4_OPTICAL_FLOW}/lib")
