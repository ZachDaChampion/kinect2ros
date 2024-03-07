cmake_minimum_required(VERSION 3.8)
project(kinect2ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Add path to freenect2 so CMake can find it
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} $ENV{HOME}/freenect2/lib/cmake/freenect2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs  REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS highgui imgcodecs imgproc videoio)
find_package(freenect2 REQUIRED)

generate_parameter_library(
  kinect2ros_params
  src/parameters.yaml
)

add_executable(kinect2ros src/kinect2ros.cpp src/main.cpp)

ament_target_dependencies(kinect2ros
  rclcpp
  std_msgs
  sensor_msgs
  image_transport
  tf2
  tf2_ros
  tf2_geometry_msgs
  camera_info_manager
)

target_include_directories(kinect2ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${freenect2_INCLUDE_DIR}
)

target_compile_features(kinect2ros PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_link_libraries(kinect2ros
  ${OpenCV_LIBS}
  ${freenect2_LIBRARIES}
  kinect2ros_params
)

install(TARGETS kinect2ros DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY camera_calibration DESTINATION share/${PROJECT_NAME})


ament_package()
