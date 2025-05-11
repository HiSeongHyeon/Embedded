#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <opencv2/opencv.hpp>
#include <utility>

// 이미지 해상도 (픽셀 단위): (width, height)
const std::pair<int, int> DEFAULT_IMAGE_SIZE = {640, 480};

// 카메라 시야각 (수평, 수직) in degrees
const std::pair<double, double> DEFAULT_FOV = {70.0, 60.0};

// 차량 내 카메라 위치 (미터 단위): (x, y, z)
const cv::Point3d DEFAULT_CAMERA_POS(0.5, 1.0, 0.5);

// 운전자 눈 위치 (미터 단위): (x, y, z)
const cv::Point3d DEFAULT_DRIVER_POS(0.25, 1.2, 0.0);

// 운전자 좌표계에서 앞유리(윈드실드) 평면의 z 좌표 (미터 단위)
const double DEFAULT_WINDSHIELD_Z = 1.0;

// 윈드실드(유리창)의 물리적 크기 (미터 단위): (width, height)
const std::pair<double, double> DEFAULT_GLASS_SIZE = {1.0, 0.5};

// 윈드실드(유리창) 좌상단 물리 좌표 (미터 단위): (x_left, y_top)
const std::pair<double, double> DEFAULT_GLASS_ORIGIN = {0.0, 1.5};

// 라즈베리 파이 카메라 모듈 v3에 대한 내부 매개변수(예시값)
const cv::Mat DEFAULT_CAMERA_MATRIX =
    (cv::Mat_<double>(3, 3) << 600, 0, 320, 0, 600, 240, 0, 0, 1);

// 왜곡 계수 (k1, k2, p1, p2, k3) – 예시값
const cv::Mat DEFAULT_DISTORTION_COEFFICIENTS =
    (cv::Mat_<double>(1, 5) << -0.2, 0.1, 0.0, 0.0, 0.0);

#endif // CONFIG_HPP