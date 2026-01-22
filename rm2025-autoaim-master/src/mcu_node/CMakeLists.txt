cmake_minimum_required(VERSION 3.8)
project(mcu_node)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

macro(import_library name path)
    add_library(${name} SHARED IMPORTED)
    set_target_properties(
        ${name}
        PROPERTIES
        IMPORTED_LOCATION ${path}
    )
endmacro()
import_library(libserial /usr/lib/x86_64-linux-gnu/libserial.so)

find_package (ament_cmake_auto REQUIRED) 
find_package(OpenCV REQUIRED)

ament_auto_find_build_dependencies()


set(srcs
src/mcu_main.cpp
src/mcu_node.cpp
src/mcu.cpp
src/serial.cpp)
ament_auto_add_executable(${PROJECT_NAME} ${srcs})

ament_target_dependencies(${PROJECT_NAME}
   OpenCV)

target_link_libraries(${PROJECT_NAME}
  libserial)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include
  DESTINATION include
)
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)
install(
  DIRECTORY ../../param/
  DESTINATION lib/${PROJECT_NAME}/param/
)
install(
  FILES ../utils/robot_mapper/robot_map.conf
  DESTINATION lib/${PROJECT_NAME}/src/utils/robot_mapper/
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
