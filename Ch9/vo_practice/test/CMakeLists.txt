cmake_minimum_required(VERSION 2.8)
project(vo_practice)

find_package(fmt REQUIRED)

add_executable(run_vo run_vo.cpp)
target_link_libraries(run_vo vo_practice_lib)
target_link_libraries(run_vo fmt)