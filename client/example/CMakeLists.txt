cmake_minimum_required(VERSION 3.1)

set (CMAKE_CXX_STANDARD 17)

add_executable(simple simple.cpp)

set(EIGEN_PATH ../deps/Eigen3)
set(INCLUDE_LIST
  ../include
  ${EIGEN_PATH}
)
include_directories(${INCLUDE_LIST})

target_link_libraries(simple autd3)
target_link_libraries(simple autdsoem)
target_link_libraries(simple ws2_32.lib)
target_link_libraries(simple winmm.lib)
