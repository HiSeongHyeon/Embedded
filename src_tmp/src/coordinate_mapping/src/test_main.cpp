#include "config.hpp"
#include "coordinates.hpp"
#include "get_grid_size.hpp"
#include <iostream>
#include <utility> // std::pair
#include <vector>

int main() {
  // 임의의 태양 중심점 (Python 코드의 예시 값 사용)
  std::pair<double, double> sun_center = {320.0, 120.0};

  // 1. 좌표 변환 함수 호출
  // camera_to_driver_coords 함수는 config.hpp의 기본 인자값을 사용
  std::cout << "[Test] Calling camera_to_driver_coords with sun_center: ("
            << sun_center.first << ", " << sun_center.second << ")"
            << std::endl;

  std::pair<int, int> grid = camera_to_driver_coords(sun_center);

  // 결과 출력
  std::cout << "Grid Position: (" << grid.first << ", " << grid.second << ")"
            << std::endl;

  // 2. 비트 리스트 변환 함수 호출
  std::vector<int> bit_list_vec =
      to_bit_list(grid); // coordinates.hpp에 선언된 함수 사용

  // 비트 리스트 결과 출력
  std::cout << "Grid Position as bit: [";
  for (size_t i = 0; i < bit_list_vec.size(); ++i) {
    std::cout << bit_list_vec[i] << (i == bit_list_vec.size() - 1 ? "" : ", ");
  }
  std::cout << "]" << std::endl;

  // 시각화 함수는 무시

  return 0;
}