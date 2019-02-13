cmake_minimum_required(VERSION 2.6)
project(object_detection)

add_compile_options(-std=c++11)
SET(CMAKE_BUILD_TYPE Release)

find_package(OpenCV 4 REQUIRED)

include_directories(
	${OpenCV_INCLUDE_DIRS}
	${PROJECT_SOURCE_DIR}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(object_detector SHARED
object_detector.cpp
)

target_link_libraries(object_detector
 ${OpenCV_LIBS}
)

