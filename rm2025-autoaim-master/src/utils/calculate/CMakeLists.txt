cmake_minimum_required(VERSION 3.8)
project(calculate)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(ament_cmake REQUIRED)
find_package (ament_cmake_auto REQUIRED)
set(OpenCV_INCLUDE "/usr/local/include/opencv4")
include_directories( ${OpenCV_INCLUDE})
find_package(OpenVINO REQUIRED)
#find_package(InferenceEngine REQUIRED)
set(Eigen3_DIR "/usr/include/eigen3/")
file(GLOB_RECURSE srcs CONFIGURE_DEPENDES src/*.cpp)
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories( ${Eigen3_DIR})
include_directories(${OpenCV_INCLUDE_DIRS}/eigen3)
ament_auto_add_library(${PROJECT_NAME} SHARED ${srcs} )
target_link_libraries(${PROJECT_NAME} 
/usr/local/lib/libopencv_world.so
${Eigen3_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}  Eigen3)
target_include_directories(${PROJECT_NAME} PUBLIC include/calculate)

install(DIRECTORY include/calculate
  DESTINATION include
)

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

ament_auto_package ()
