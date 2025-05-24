#ifndef COORDINATES_HPP
#define COORDINATES_HPP

#include <opencv2/opencv.hpp>
#include <utility>
#include <vector> // to_bit_list 반환 타입 위해

#include <config.hpp>

// ray_plane_intersection 함수 선언
cv::Point3d ray_plane_intersection(const cv::Point3d &origin,
                                   const cv::Vec3d &direction, double plane_z);

// to_bit_list 함수 선언
std::vector<int> to_bit_list(const std::pair<int, int> &grid_coord,
                             int bits = 4);

// camera_to_driver_coords 함수 선언
std::pair<int, int> camera_to_driver_coords(
    const std::pair<double, double> &sun_center,
    const std::pair<int, int> &image_size = DEFAULT_IMAGE_SIZE,
    const std::pair<double, double> &fov = DEFAULT_FOV,
    const cv::Point3d &camera_pos = DEFAULT_CAMERA_POS,
    const cv::Point3d &driver_eye_pos = DEFAULT_DRIVER_POS,
    double windshield_z = DEFAULT_WINDSHIELD_Z,
    const std::pair<double, double> &glass_size = DEFAULT_GLASS_SIZE,
    const std::pair<double, double> &glass_origin = DEFAULT_GLASS_ORIGIN);

void visualize_grid_on_frame(cv::Mat &display_frame,
                             const std::pair<int, int> &target_grid_coords,
                             const std::pair<int, int> &total_grid_dims,
                             int image_width = DEFAULT_IMAGE_SIZE.first,
                             int image_height = DEFAULT_IMAGE_SIZE.second);

#endif // COORDINATES_HPP