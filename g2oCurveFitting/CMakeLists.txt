cmake_minimum_required(VERSION 2.8)
project(g2oCurveFitting)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

include_directories("/usr/include/eigen3")

add_executable(g2oCurveFitting g2oCurveFitting.cpp)
target_link_libraries(g2oCurveFitting ${OpenCV_LIBS} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${CERES_LIBRARIES})