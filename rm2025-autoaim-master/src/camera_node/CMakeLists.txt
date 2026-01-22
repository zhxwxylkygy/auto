cmake_minimum_required(VERSION 3.8)
project(camera_node)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

set(OpenCV_INCLUDE "/usr/local/include/opencv4")
include_directories( ${OpenCV_INCLUDE})

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

include_directories(../utils/param_loader)

file(GLOB_RECURSE srcs CONFIGURE_DEPENDES src/*.cpp)

ament_auto_add_library(${PROJECT_NAME} SHARED ${srcs})
#ament_auto_add_executable(${PROJECT_NAME}  ${srcs})
target_link_libraries(${PROJECT_NAME} 
  /usr/local/lib/libopencv_world.so
)
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(${PROJECT_NAME}
  gxiapi)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN gxcamera::CameraNode
  EXECUTABLE gxcamera_node
)

install(
  FILES share/plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS camera_node gxcamera_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)
# install(TARGETS ${PROJECT_NAME}
#   DESTINATION lib/
# )
install(
  DIRECTORY ../../param/
  DESTINATION lib/param/
)
install(
  FILES ../utils/robot_mapper/robot_map.conf
  DESTINATION lib/src/utils/robot_mapper/
)
ament_auto_package()
