cmake_minimum_required(VERSION 3.8)
project(kinect2ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Add path to freenect2 so CMake can find it
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} $ENV{HOME}/freenect2)

find_package(ament_cmake REQUIRED)
find_package( OpenCV REQUIRED )
find_package(freenect2 REQUIRED)

add_executable(kinect2ros src/kinect2ros.cpp)

target_include_directories(kinect2ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${freenect2_INCLUDE_DIR}
)

target_compile_features(kinect2ros PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_link_libraries(kinect2ros ${OpenCV_LIBS} ${freenect2_LIBRARIES})

install(TARGETS kinect2ros
  DESTINATION lib/${PROJECT_NAME})


ament_package()
