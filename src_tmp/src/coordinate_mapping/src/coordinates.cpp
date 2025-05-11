#include "coordinates.hpp"
#include "config.hpp"
#include "get_grid_size.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

inline double to_radians(double degrees) { return degrees * CV_PI / 180.0; }

cv::Point3d ray_plane_intersection(const cv::Point3d &origin,
                                   const cv::Vec3d &direction,
                                   double plane_z) { // y에서 z로 변경
  // Ray 방정식: P = origin + t * direction
  // 평면 방정식: P.z = plane_z
  // origin.z + t * direction[2] = plane_z
  // t = (plane_z - origin.z) / direction[2]
  if (std::abs(direction[2]) <
      1e-9) { // z 방향 성분이 거의 0이면 평면과 평행 (또는 포함)
    // 적절한 에러 처리 또는 예외적 상황 반환
    // 여기서는 일단 매우 큰 값을 갖는 포인트를 반환하거나, 에러를 명시할 수
    // 있음
    return cv::Point3d(std::numeric_limits<double>::max(),
                       std::numeric_limits<double>::max(),
                       std::numeric_limits<double>::max());
  }
  double t = (plane_z - origin.z) /
             direction[2]; // origin.y -> origin.z, direction[1] -> direction[2]
  return origin + t * cv::Point3d(direction);
}

std::vector<int> to_bit_list(const std::pair<int, int> &grid_coord, int bits) {
  std::pair<int, int> grid_dims = get_grid_size();
  int num_cols = grid_dims.first;
  // Python: value = grid_coord[0] * num_cols + grid_coord[1]
  // grid_coord는 (col, row) 순서
  int value = grid_coord.first * num_cols + grid_coord.second;

  std::vector<int> bit_list;
  bit_list.reserve(bits); // 메모리 미리 할당
  for (int i = bits - 1; i >= 0; --i) {
    bit_list.push_back((value >> i) & 1);
  }
  return bit_list;
}

std::pair<int, int> camera_to_driver_coords(
    const std::pair<double, double> &sun_center,
    const std::pair<int, int> &image_size, const std::pair<double, double> &fov,
    const cv::Point3d
        &camera_pos, // 사용되지 않음 (Python 코드에서 camera_pos는 방향벡터
                     // 계산에 직접 사용 안 함)
    const cv::Point3d &driver_eye_pos,
    double windshield_z, // y에서 z로 변경
    const std::pair<double, double> &glass_size,
    const std::pair<double, double> &glass_origin) {

  cv::Mat sun_center_mat(1, 1, CV_64FC2);
  sun_center_mat.at<cv::Vec2d>(0, 0)[0] = sun_center.first;
  sun_center_mat.at<cv::Vec2d>(0, 0)[1] = sun_center.second;

  cv::Mat undistorted_points;
  cv::undistortPoints(sun_center_mat, undistorted_points, DEFAULT_CAMERA_MATRIX,
                      DEFAULT_DISTORTION_COEFFICIENTS, cv::noArray(),
                      DEFAULT_CAMERA_MATRIX);

  cv::Vec2d corrected_sun_vec = undistorted_points.at<cv::Vec2d>(0, 0);
  std::pair<double, double> corrected_sun_center = {corrected_sun_vec[0],
                                                    corrected_sun_vec[1]};

  double img_w = static_cast<double>(image_size.first);
  double img_h = static_cast<double>(image_size.second);
  double fov_x_deg = fov.first;
  double fov_y_deg = fov.second;
  double x_center_px = corrected_sun_center.first;
  double y_center_px = corrected_sun_center.second;

  double norm_x = (x_center_px / img_w - 0.5) * 2.0;
  double norm_y = (y_center_px / img_h - 0.5) * 2.0;

  double angle_x_rad = to_radians(norm_x * (fov_x_deg / 2.0));
  double angle_y_rad = to_radians(norm_y * (fov_y_deg / 2.0));

  // (du,dv,dw)는 방향 벡터, 좌표계 축은 (x,y,z)와 동일
  // Python 코드: du = math.tan(angle_x), dv = -math.tan(angle_y), dw = 1.0
  double du = std::tan(angle_x_rad);
  double dv = -std::tan(angle_y_rad); // y 방향 반전 주의
  double dw = 1.0; // 카메라 기본 전방은 +z(+w) 방향 (Python 코드와 동일)

  cv::Vec3d D_vec(du, dv, dw);
  D_vec = cv::normalize(D_vec);

  // 광선의 원점은 driver_eye_pos로 설정
  cv::Point3d intersection =
      ray_plane_intersection(driver_eye_pos, D_vec, windshield_z);
  double px = intersection.x;
  double py = intersection.y; // 이전에는 _ 로 무시, 이제 py 사용
  // double pz = intersection.z; // windshield_z 와 동일

  // 물리적 교차점 (px, py)을 glass_origin과 glass_size를 기반으로 grid 좌표로
  // 매핑
  std::pair<int, int> grid_dims = get_grid_size();
  int grid_cols = grid_dims.first;
  int grid_rows = grid_dims.second;

  double glass_width = glass_size.first;
  double glass_height = glass_size.second;
  double x_left_glass =
      glass_origin.first; // glass_origin은 좌상단 기준 (x_left, y_top)
  double y_top_glass = glass_origin.second;

  int grid_x =
      static_cast<int>(((px - x_left_glass) / glass_width) * grid_cols);
  // Python 코드: grid_y = int(((y_top - py) / glass_height) * grid_rows)
  // C++: y축 방향 일치 여부 확인 필요. 이미지 y는 아래로, 3D y는 위로 가정 시
  int grid_y =
      static_cast<int>(((y_top_glass - py) / glass_height) * grid_rows);

  grid_x = std::max(0, std::min(grid_x, grid_cols - 1));
  grid_y = std::max(0, std::min(grid_y, grid_rows - 1));

  return {grid_x, grid_y};
}