project(SerialCommunication)

# 헤더 파일 검색 경로 설정
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")

# 시리얼 통신 라이브러리 생성
add_library(serial_communication_lib STATIC
    src/serial_communication.cpp
)

include_directories(
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_SOURCE_DIR}/src/coordinate_mapping/include" # coordinate_mapping 헤더 경로 추가
)

# 이 라이브러리를 사용하는 다른 타겟이 헤더를 찾을 수 있도록 공개
target_include_directories(serial_communication_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
