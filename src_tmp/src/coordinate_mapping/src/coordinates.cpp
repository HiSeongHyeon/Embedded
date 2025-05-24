#include "coordinates.hpp"
#include "config.hpp"
#include "get_grid_size.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

inline double to_radians(double degrees) { return degrees * CV_PI / 180.0; }

cv::Point3d ray_plane_intersection(const cv::Point3d &origin,
                                   const cv::Vec3d &direction, double plane_z) {
  // Ray 방정식: P = origin + t * direction
  // 평면 방정식: P.z = plane_z
  // origin.z + t * direction[2] = plane_z
  // t = (plane_z - origin.z) / direction[2]
  if (std::abs(direction[2]) <
      1e-9) { // z 방향 성분이 거의 0이면 평면과 평행 (또는 포함)
    return cv::Point3d(std::numeric_limits<double>::max(),
                       std::numeric_limits<double>::max(),
                       std::numeric_limits<double>::max());
  }
  double t = (plane_z - origin.z) /
             direction[2]; // origin.y -> origin.z, direction[1] -> direction[2]
  return origin + t * cv::Point3d(direction);
}

std::vector<int> to_bit_list(const std::pair<int, int> &grid_coord, int bits) {
  // grid_coord.first는 col (x 좌표), grid_coord.second는 row (y 좌표)
  int col = grid_coord.first;
  int row = grid_coord.second;

  if (col < 0 || col > 2 || row < 0 || row > 2) {
    // Handle error: invalid input for 3x3 grid
    return {0, 0, 0, 0}; // Example error return
  }

  std::vector<int> bit_list;
  bit_list.reserve(4); // 항상 4개의 비트

  // 상위 2비트: Column (x 좌표) [Col_MSB, Col_LSB]
  bit_list.push_back((col >> 1) & 1); // Col_MSB (col의 1번째 비트)
  bit_list.push_back(col & 1);        // Col_LSB (col의 0번째 비트)

  // 하위 2비트: Row (y 좌표) [Row_MSB, Row_LSB]
  bit_list.push_back((row >> 1) & 1); // Row_MSB (row의 1번째 비트)
  bit_list.push_back(row & 1);        // Row_LSB (row의 0번째 비트)

  return bit_list;
}

std::pair<int, int> camera_to_driver_coords(
    const std::pair<double, double> &sun_center,
    const std::pair<int, int> &image_size, const std::pair<double, double> &fov,
    const cv::Point3d &camera_pos, const cv::Point3d &driver_eye_pos,
    double windshield_z, const std::pair<double, double> &glass_size,
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
  double du = std::tan(angle_x_rad);
  double dv = -std::tan(angle_y_rad);
  double dw = 1.0; // 카메라 기본 전방은 +z(+w) 방향

  cv::Vec3d D_vec(du, dv, dw);
  D_vec = cv::normalize(D_vec);

  // 광선의 원점은 driver_eye_pos로 설정
  cv::Point3d intersection =
      ray_plane_intersection(driver_eye_pos, D_vec, windshield_z);
  double px = intersection.x;
  double py = intersection.y; // 이전에는 _ 로 무시, 이제 py 사용
  double pz = intersection.z; // windshield_z 와 동일

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
  int grid_y =
      static_cast<int>(((y_top_glass - py) / glass_height) * grid_rows);

  grid_x = std::max(0, std::min(grid_x, grid_cols - 1));
  grid_y = std::max(0, std::min(grid_y, grid_rows - 1));

  return {grid_x, grid_y};
}

void visualize_grid_on_frame(
    cv::Mat &display_frame,
    const std::pair<int, int> &target_grid_coords, // (target_col, target_row)
    const std::pair<int, int> &total_grid_dims,    // (total_cols, total_rows)
    int image_width, int image_height) {
  int total_cols = total_grid_dims.first;
  int total_rows = total_grid_dims.second;

  if (total_cols <= 0 || total_rows <= 0)
    return; // 유효하지 않은 그리드 크기

  // 각 그리드 셀의 화면상 너비와 높이 계산
  float cell_pixel_width = static_cast<float>(image_width) / total_cols;
  float cell_pixel_height = static_cast<float>(image_height) / total_rows;

  // 1. 전체 그리드 경계선 그리기
  for (int i = 1; i < total_cols; ++i) { // 세로선
    float x = i * cell_pixel_width;
    cv::line(display_frame, cv::Point(x, 0), cv::Point(x, image_height - 1),
             cv::Scalar(128, 128, 128), 1); // 회색
  }
  for (int i = 1; i < total_rows; ++i) { // 가로선
    float y = i * cell_pixel_height;
    cv::line(display_frame, cv::Point(0, y), cv::Point(image_width - 1, y),
             cv::Scalar(128, 128, 128), 1); // 회색
  }

  // 2. 현재 타겟 그리드 셀 강조
  int target_col = target_grid_coords.first;
  int target_row = target_grid_coords.second;

  if (target_col >= 0 && target_col < total_cols && target_row >= 0 &&
      target_row < total_rows) {
    float rect_x = target_col * cell_pixel_width;
    float rect_y = target_row * cell_pixel_height;

    cv::Rect roi(static_cast<int>(rect_x), static_cast<int>(rect_y),
                 static_cast<int>(cell_pixel_width),
                 static_cast<int>(cell_pixel_height));

    // 반투명한 사각형으로 강조
    cv::Mat overlay;
    display_frame.copyTo(overlay);
    cv::rectangle(overlay, roi, cv::Scalar(0, 255, 0, 100),
                  -1); // 초록색, 약간 투명하게 (alpha는 PNG 저장 시 의미)
    double alpha = 0.3; // 투명도
    cv::addWeighted(overlay, alpha, display_frame, 1 - alpha, 0, display_frame);

    // 또는 간단히 테두리만
    // cv::rectangle(display_frame, roi, cv::Scalar(0, 255, 0), 2); // 초록색
    // 테두리

    // 그리드 셀 중앙에 텍스트 표시
    // std::string text = std::to_string(target_col) + "," +
    // std::to_string(target_row); cv::Point text_origin(static_cast<int>(rect_x
    // + cell_pixel_width / 3), static_cast<int>(rect_y + cell_pixel_height /
    // 2)); cv::putText(display_frame, text, text_origin,
    // cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
  }
}