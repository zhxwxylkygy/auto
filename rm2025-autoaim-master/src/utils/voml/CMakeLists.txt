cmake_minimum_required(VERSION 3.8)
project(voml)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
include_directories(./include/voml)
include_directories(./toml11)

# find dependencies
find_package(OpenCV 4 CONFIG REQUIRED)
find_package (ament_cmake_auto REQUIRED) 
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
file(GLOB_RECURSE srcs CONFIGURE_DEPENDES src/*.cpp include/*.h)

ament_auto_add_library(${PROJECT_NAME} SHARED ${srcs})
target_include_directories(voml PUBLIC include toml11)
target_link_libraries(${PROJECT_NAME} PRIVATE opencv_world)

install(DIRECTORY include/
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
