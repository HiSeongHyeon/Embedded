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

# 라이브러리 링크
target_link_libraries(solarTracker PUBLIC
    coordinate_mapping_lib
    serial_communication_lib
    ${OpenCV_LIBS}
)