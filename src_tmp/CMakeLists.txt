cmake_minimum_required(VERSION 3.10)
project(SunVisorRobotProject)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# OpenCV 패키지 찾기 (전역적으로)
find_package(OpenCV REQUIRED)
# OpenCV 헤더 경로는 하위 CMakeLists.txt에서 ${OpenCV_INCLUDE_DIRS}로 사용 가능

# 하위 모듈(라이브러리 및 실행파일) 빌드
add_subdirectory(src/coordinate_mapping)
add_subdirectory(src/serial_communication)
add_subdirectory(src/glare_detection)