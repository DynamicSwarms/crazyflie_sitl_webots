cmake_minimum_required(VERSION 3.8)
project(quadcopter_model)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# Build quadcopter_model library
file(GLOB_RECURSE QUADCOPTER_MODEL_SRCS
    src/common/*.cpp
    src/dynamics/*.cpp
    src/objects/*.cpp
    src/sensors/*.cpp
)

add_library(quadcopter_model
  ${QUADCOPTER_MODEL_SRCS}
)
set_target_properties(quadcopter_model PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_compile_features(quadcopter_model PUBLIC cxx_std_17)
target_include_directories(quadcopter_model PUBLIC
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)


target_link_libraries(quadcopter_model
  Eigen3::Eigen
  yaml-cpp
)