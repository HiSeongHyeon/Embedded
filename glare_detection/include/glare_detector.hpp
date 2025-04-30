#ifndef GLARE_DETECTOR_HPP
#define GLARE_DETECTOR_HPP

#include <opencv2/opencv.hpp>

// GlareDetector 클래스는 입력 이미지로부터 glare 후보 영역을 계산하는 기능을 제공합니다.
// 논문 기반의 photometric (intensity, saturation, contrast) 및 geometric (원형 검출) 방법 사용.
class GlareDetector {
public:
    // Photometric feature map 계산:
    // HSV의 intensity(V), saturation(S), local contrast를 기반으로 계산.
    cv::Mat computePhotometricMap(const cv::Mat& inputRGB);

    // Geometric feature map 계산:
    // Gphoto 맵에 Gaussian Blur를 적용한 뒤, Hough Circle Transform을 통해 glare 영역 검출.
    cv::Mat computeGeometricMap(const cv::Mat& gphoto);

    // 위 두 맵과 (선택적으로) 태양 위치 기반 Gsun 맵을 결합하여 최종 glare map을 생성.
    cv::Mat combineMaps(const cv::Mat& gphoto, const cv::Mat& ggeo, const cv::Mat& gsun);

private:
    // Local Contrast 계산: 17x17 블록 단위로 RMS 대비 계산
    cv::Mat computeLocalContrast(const cv::Mat& intensity);
};

#endif // GLARE_DETECTOR_HPP
