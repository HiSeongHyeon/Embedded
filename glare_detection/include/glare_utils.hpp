#ifndef GLARE_UTILS_HPP
#define GLARE_UTILS_HPP

#include <opencv2/opencv.hpp>

// GlareUtils 네임스페이스는 glare mask로부터 중심좌표 계산,
// 그리고 glare 영역을 시각적으로 표시하기 위한 유틸리티 함수를 제공합니다.
namespace GlareUtils {
    // glare mask에서 모든 non-zero 픽셀들의 중심좌표 계산
    cv::Point2f computeCentroid(const cv::Mat& binaryMask);

    // glare mask로부터 contour를 추출하고, 각 영역에 최소 외접원(circle) 그리기
    void drawGlareContours(const cv::Mat& binaryMask, cv::Mat& frame);
}

#endif // GLARE_UTILS_HPP
