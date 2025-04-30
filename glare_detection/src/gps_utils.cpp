#include "gps_utils.hpp"
#include <cmath>

inline double deg2rad(double deg) { return deg * M_PI / 180.0; }
inline double rad2deg(double rad) { return rad * 180.0 / M_PI; }

// 현재 시간 및 위치로부터 태양의 고도와 방위각 계산 → 이미지 좌표 변환
cv::Mat GPSUtils::computeSunMap(const cv::Size& imageSize, double latitude, double longitude, double heading) {
    cv::Mat gsun = cv::Mat::zeros(imageSize, CV_32F);

    time_t now = time(nullptr);
    struct tm *utc = gmtime(&now);
    double day_of_year = utc->tm_yday + 1;
    double hour = utc->tm_hour + utc->tm_min / 60.0 + utc->tm_sec / 3600.0;

    double decl = 23.45 * sin(deg2rad(360.0 * (284 + day_of_year) / 365.0));
    double time_offset = 4 * (longitude - 15 * int(longitude / 15.0));
    double tst = hour * 60 + time_offset;
    double ha = (tst / 4.0 - 180.0);

    double sin_el = sin(deg2rad(latitude)) * sin(deg2rad(decl)) +
                    cos(deg2rad(latitude)) * cos(deg2rad(decl)) * cos(deg2rad(ha));
    double elevation = rad2deg(asin(sin_el));

    double cos_az = (sin(deg2rad(decl)) - sin(deg2rad(elevation)) * sin(deg2rad(latitude))) /
                    (cos(deg2rad(elevation)) * cos(deg2rad(latitude)));
    double azimuth = rad2deg(acos(cos_az));
    if (ha > 0) azimuth = 360 - azimuth;

    double hfov = 90.0, vfov = 60.0;
    double x_angle = azimuth - heading;
    double y_angle = elevation;

    int x = static_cast<int>(imageSize.width / 2 * (1 + x_angle / (hfov / 2)));
    int y = static_cast<int>(imageSize.height / 2 * (1 - y_angle / (vfov / 2)));
    x = std::min(std::max(x, 0), imageSize.width - 1);
    y = std::min(std::max(y, 0), imageSize.height - 1);

    int radius = imageSize.height / 10;
    cv::circle(gsun, cv::Point(x, y), radius, cv::Scalar::all(1.0), -1);
    cv::GaussianBlur(gsun, gsun, cv::Size(21, 21), imageSize.height / 12.0);
    cv::normalize(gsun, gsun, 0.0, 1.0, cv::NORM_MINMAX);
    return gsun;
}
