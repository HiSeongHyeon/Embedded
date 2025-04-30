#ifndef GPS_UTILS_HPP
#define GPS_UTILS_HPP

#include <opencv2/opencv.hpp>
#include <ctime>

// GPSUtils 클래스는 위도, 경도, 시간 기반으로 태양의 위치를 계산하고,
// 해당 위치에 Gaussian blob을 생성하여 Gsun 맵을 제공합니다.
class GPSUtils {
public:
    // 현재 프레임 해상도 및 GPS 정보(위도, 경도, heading)를 바탕으로
    // 태양 위치 추정 맵(Gsun)을 생성합니다.
    cv::Mat computeSunMap(const cv::Size& imageSize,
                          double latitude = 37.5665,
                          double longitude = 126.9780,
                          double heading = 0.0);
};

#endif // GPS_UTILS_HPP
