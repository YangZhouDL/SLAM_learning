cmake_minimum_required(VERSION 2.8)
project(vo_practice)

add_library(vo_practice_lib STATIC
    frame.cpp
    mappoint.cpp
    map.cpp
    camera.cpp
    config.cpp
    g2o_types.cpp
    visual_odometry.cpp
)

target_link_libraries(vo_practice_lib ${THIRD_PARTY_LIBS})