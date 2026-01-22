cmake_minimum_required(VERSION 3.8)
project(detect_node)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
  #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg -fsanitize=thread -g")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pg")
endif()

add_subdirectory(classic_detector)
# find dependencies
find_package(ament_cmake_auto REQUIRED)

# find_package(OpenCV REQUIRED)
set(OpenCV_INCLUDES "/usr/loacl/include/opencv4")
include_directories( ${OpenCV_INCLUDES})


set(Eigen3_DIR "/usr/include/eigen3/")
include_directories( ${Eigen3_DIR})
ament_auto_find_build_dependencies()


file(GLOB_RECURSE srcs CONFIGURE_DEPENDES src/*.cpp)

ament_auto_add_library(${PROJECT_NAME} SHARED ${srcs})

target_link_libraries(${PROJECT_NAME}
  classic_detector
  /usr/local/lib/libopencv_world.so
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN  detector::Subscriber
  EXECUTABLE detector_node
)

install(DIRECTORY include/
  DESTINATION include
)
install(TARGETS detect_node detector_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
# install(TARGETS ${PROJECT_NAME}
#   DESTINATION lib/
# )

install(
  FILES share/plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# install(
#   FILES ../../utils/robot_mapper/robot_map.conf
#   DESTINATION lib/src/utils/robot_mapper/
# )
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

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
