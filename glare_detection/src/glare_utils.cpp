#include "glare_utils.hpp"

// glare mask의 non-zero 픽셀들로 중심 좌표 계산
cv::Point2f GlareUtils::computeCentroid(const cv::Mat& binaryMask) {
    std::vector<cv::Point> nonzero_points;
    cv::findNonZero(binaryMask, nonzero_points);
    if (nonzero_points.empty()) return cv::Point2f(-1, -1);

    cv::Point2f center(0, 0);
    for (const auto& pt : nonzero_points) center += pt;
    center.x /= nonzero_points.size();
    center.y /= nonzero_points.size();
    return center;
}

// glare mask의 외곽선에서 최소 외접 원을 구해 시각적으로 표시
void GlareUtils::drawGlareContours(const cv::Mat& binaryMask, cv::Mat& frame) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binaryMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (const auto& contour : contours) {
        if (contour.size() > 5) {
            cv::Point2f circ_center;
            float radius;
            cv::minEnclosingCircle(contour, circ_center, radius);
            cv::circle(frame, circ_center, static_cast<int>(radius), cv::Scalar(0, 255, 0), 2);
        }
    }
}
