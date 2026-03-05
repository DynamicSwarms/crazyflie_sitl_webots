cmake_minimum_required(VERSION 3.8)
project(crazyflie_sitl_webots)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(crtp_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)


if(NOT DEFINED ENV{WEBOTS_HOME})
  message(FATAL_ERROR "Environment variable WEBOTS_HOME is not set. 
  Please set it to the Webots installation directory.
  Typically: export WEBOTS_HOME=/usr/local/webots")
endif()
set(WEBOTS_HOME $ENV{WEBOTS_HOME})

## Include official Webots C/C++ controller API
include_directories(
  ${WEBOTS_HOME}/include/controller/c
  ${WEBOTS_HOME}/include/controller/cpp
)
link_directories(${WEBOTS_HOME}/lib/controller)

# Automatically embed Webots controller library path in all targets
set(CMAKE_INSTALL_RPATH "${WEBOTS_HOME}/lib/controller")
set(CMAKE_BUILD_RPATH   "${WEBOTS_HOME}/lib/controller")

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_BUILD_RPATH_USE_ORIGIN TRUE)

## Build the Custom Webots Driver Library
set(WEBOTS_DRIVER_SRC
  src/webots_driver/webots_robot_driver.cpp
  src/webots_driver/webots_crazyflie_driver.cpp
)
add_library(webots_driver_lib ${WEBOTS_DRIVER_SRC})
set_target_properties(webots_driver_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(webots_driver_lib PUBLIC cxx_std_17) 
target_include_directories(webots_driver_lib PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
)

target_link_libraries(webots_driver_lib
  Controller
  CppController
  Eigen3::Eigen
)

# Build the sitl_webots_adapter node executable
add_executable(sitl_webots_adapter src/sitl_webots_adapter.cpp)

target_link_libraries(sitl_webots_adapter
  webots_driver_lib
  Controller
  CppController
)

ament_target_dependencies(sitl_webots_adapter 
  rclcpp
  crtp_interfaces)

target_include_directories(sitl_webots_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(sitl_webots_adapter PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

## Finally build all nodes
install(TARGETS sitl_webots_adapter
  DESTINATION lib/${PROJECT_NAME})

ament_package()
