cmake_minimum_required(VERSION 3.22)

project(rm2025autoaim)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake_auto REQUIRED)
find_package(OpenVINO REQUIRED)
set(OpenCV_INCLUDE "/usr/local/include/opencv4")
include_directories( ${OpenCV_INCLUDE})
ament_auto_find_build_dependencies()


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
