cmake_minimum_required(VERSION 3.10)
project(solarTracker)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} include)

add_executable(solarTracker
    src/main.cpp
    src/glare_detector.cpp
    src/return_position.cpp
)

target_link_libraries(solarTracker ${OpenCV_LIBS})
