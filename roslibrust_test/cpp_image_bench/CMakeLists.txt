cmake_minimum_required(VERSION 3.0.2)
project(roslibrust_cpp_image_bench)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(cpp_image_bench src/image_bench.cpp)
target_compile_features(cpp_image_bench PRIVATE cxx_std_17)
target_compile_options(cpp_image_bench PRIVATE -O3 -DNDEBUG)
target_link_libraries(cpp_image_bench ${catkin_LIBRARIES})
