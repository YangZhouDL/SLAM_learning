cmake_minimum_required(VERSION 2.8)
project(trajectoryError )

include_directories("/usr/include/eigen3")
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(fmt REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

# add_executable(visualizeGeometry visualizeGeometry.cpp)
# target_link_libraries(visualizeGeometry ${Pangolin_LIBRARIES})
add_executable(trajectoryError  trajectoryError.cpp)
target_link_libraries(trajectoryError  ${Pangolin_LIBRARIES})
target_link_libraries(trajectoryError  fmt)