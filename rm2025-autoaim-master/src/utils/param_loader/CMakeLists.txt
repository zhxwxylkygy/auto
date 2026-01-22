cmake_minimum_required(VERSION 3.8)
project(param_loader)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

find_package (ament_cmake_auto REQUIRED) 
find_package (ament_cmake REQUIRED) 

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
file(GLOB_RECURSE srcs CONFIGURE_DEPENDES ./*hpp ./*cpp)


ament_auto_add_library(${PROJECT_NAME} SHARED ${srcs})

set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)


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
