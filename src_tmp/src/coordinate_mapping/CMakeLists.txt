cmake_minimum_required(VERSION 3.10)
project(CoordinateMapping)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(OpenCV REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS} # OpenCV 헤더 경로
    "${CMAKE_CURRENT_SOURCE_DIR}/include" # 현재 CMakeLists.txt 기준 include 폴더
)

# 라이브러리 생성 (예: 정적 라이브러리 "coordinate_mapping_lib")
add_library(coordinate_mapping_lib STATIC
    src/coordinates.cpp
)

# 생성된 라이브러리에 OpenCV 라이브러리 링크
target_link_libraries(coordinate_mapping_lib PUBLIC ${OpenCV_LIBS})

# include 디렉토리 공개
target_include_directories(coordinate_mapping_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> # 빌드 시 사용
)

# 이 모듈 자체를 테스트하기 위한 실행 파일
add_executable(run_mapping_test # 실행 파일 이름
    src/test_main.cpp             # 위에서 작성한 테스트용 main 파일
)

# 테스트 실행 파일에 coordinate_mapping_lib 라이브러리와 OpenCV 라이브러리 링크
target_link_libraries(run_mapping_test PUBLIC
    coordinate_mapping_lib      # 우리가 만든 라이브러리
    ${OpenCV_LIBS}              # OpenCV 라이브러리
)