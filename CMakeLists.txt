cmake_minimum_required(VERSION 2.8)
project( Detect-Ships )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( Detect-Ships main.cpp )
target_link_libraries( Detect-Ships ${OpenCV_LIBS} )