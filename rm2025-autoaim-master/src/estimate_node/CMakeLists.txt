cmake_minimum_required(VERSION 3.8)
project(estimate_node)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Glog REQUIRED)

set(Eigen3_DIR "/usr/include/eigen3/")
include_directories( ${Eigen3_DIR})
ament_auto_find_build_dependencies()

file(GLOB_RECURSE srcs CONFIGURE_DEPENDES src/*.cpp)

# ament_auto_add_executable(${PROJECT_NAME} ${srcs} )
ament_auto_add_library(${PROJECT_NAME} SHARED ${srcs})

ament_target_dependencies(${PROJECT_NAME}
OpenCV)

target_link_libraries(${PROJECT_NAME}
  ${Eigen3_LIBRARIES}
  TBB::tbb
  glog
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN  trackers::FeatureSubscriber
  EXECUTABLE tracker_node
)
install(TARGETS ${PROJECT_NAME} tracker_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
# install(DIRECTORY include/
#   DESTINATION include
# )

# install(TARGETS ${PROJECT_NAME}
#   DESTINATION lib/${PROJECT_NAME}
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
