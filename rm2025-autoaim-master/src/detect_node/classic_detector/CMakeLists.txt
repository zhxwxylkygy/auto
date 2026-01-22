file(GLOB_RECURSE srcs CONFIGURE_DEPENDES src/*.cpp)
add_library(classic_detector OBJECT ${srcs})
target_include_directories(classic_detector PUBLIC include)

find_package(OpenCV CONFIG REQUIRED)
target_link_libraries(classic_detector PUBLIC opencv_world)

find_package(TBB CONFIG REQUIRED)
target_link_libraries(classic_detector PUBLIC TBB::tbb)

find_package(fmt CONFIG REQUIRED)
target_link_libraries(classic_detector PUBLIC fmt::fmt)
