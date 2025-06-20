#include "glare_detector.h"

int debug_color = 0;

glare_detector::glare_detector() : glareCenter(-1, -1), detectedArea(0.0), glareFound(false) {}

void glare_detector::startVideo(const cv::Mat& frame) {
    currentFrame = frame.clone();
}

void glare_detector::endVideo() {
    currentFrame.release();
}

// Photometric map 생성: Intensity(V), Saturation(S), Local Contrast(C) 기반
cv::Mat glare_detector::computePhotometricMap(const cv::Mat& inputRGB) {
    cv::Mat hsv, intensity, saturation;
    cv::cvtColor(inputRGB, hsv, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> hsv_channels;
    cv::split(hsv, hsv_channels);

    intensity = hsv_channels[2];
    saturation = hsv_channels[1];

    intensity.convertTo(intensity, CV_32F, 1.0 / 255.0);
    saturation.convertTo(saturation, CV_32F, 1.0 / 255.0);

    cv::Mat contrast = computeLocalContrast(intensity);
    cv::Mat gphoto = intensity.mul(1.0 - saturation).mul(1.0 - contrast);
    cv::normalize(gphoto, gphoto, 0.0, 1.0, cv::NORM_MINMAX);
    return gphoto;
}

// Local Contrast를 계산하는 함수
cv::Mat glare_detector::computeLocalContrast(const cv::Mat& intensity) {

    cv::Mat intensityFloat;
    intensity.convertTo(intensityFloat, CV_32F);

    int blockSize = 17;
    float minMean = 10.0f;

    // 로컬 평균
    cv::Mat mean;
    cv::boxFilter(intensityFloat, mean, CV_32F, cv::Size(blockSize, blockSize));

    // 로컬 제곱 평균
    cv::Mat sqr, meanSqr;
    cv::multiply(intensityFloat, intensityFloat, sqr);
    cv::boxFilter(sqr, meanSqr, CV_32F, cv::Size(blockSize, blockSize));

    // 표준편차 계산
    cv::Mat stddev;
    cv::sqrt(meanSqr - mean.mul(mean), stddev);

    // 대비 계산 (stddev / mean)
    cv::Mat contrast;
    cv::divide(stddev, mean + minMean, contrast); // mean이 너무 작을 경우 대비 폭주 방지

    return contrast;  // CV_32F 타입, 필요시 normalize해서 시각화 가능
}


// Geometric map 생성: Gaussian Blur 후 Hough Circle로 glare 후보 검출
cv::Mat glare_detector::computeGeometricMap(const cv::Mat& gphoto) {
    cv::Mat blurred, blurred8u;
    cv::GaussianBlur(gphoto, blurred, cv::Size(13, 13), 5);
    blurred.convertTo(blurred8u, CV_8U, 255);

    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(blurred8u, circles, cv::HOUGH_GRADIENT, 1, 20, 75, 30, 10, 200);

    cv::Mat ggeo = cv::Mat::zeros(gphoto.size(), CV_32F);
    for (const auto& circle : circles) {
        cv::Point center(cvRound(circle[0]), cvRound(circle[1]));
        int radius = cvRound(circle[2]);
        cv::circle(ggeo, center, int(1.5 * radius), cv::Scalar::all(1.0), -1);
    }
    cv::normalize(ggeo, ggeo, 0.0, 1.0, cv::NORM_MINMAX);
    return ggeo;
}

// 최종 glare map 결합: Gphoto, Ggeo
cv::Mat glare_detector::combineMaps(const cv::Mat& gphoto, const cv::Mat& ggeo) {
    return 1.0 * gphoto + 1.0 * ggeo;
}

// glare의 priority 계산
cv::Mat glare_detector::computePriorityMap(const cv::Mat& gphoto, const cv::Mat& ggeo) {
    cv::Mat priority = cv::Mat::ones(gphoto.size(), CV_8U) * 3;

    for (int y = 0; y < gphoto.rows; ++y) {
        for (int x = 0; x < gphoto.cols; ++x) {
            float p = gphoto.at<float>(y, x);
            float c = ggeo.at<float>(y, x);
            
            // 밝고 원형인 glare 존재
            if (p >= 0.95f && c >= 0.99f) {
                priority.at<uchar>(y, x) = 1;
            } 
            // 밝지만 원형은 아닌 glare 존재
            else if (p >= 0.95f) {
                priority.at<uchar>(y, x) = 2;
            } 
        }
    }
    
    return priority;
}

double glare_detector::getDetectedArea() const {
    return detectedArea;
}

// glare 영역 도식화
void glare_detector::drawGlareContours(const cv::Mat& inputImage, cv::Mat& frame) {
    cv::Mat gray;

    if (inputImage.channels() == 3) {
        // 컬러 → 그레이스케일
        cv::cvtColor(inputImage, gray, cv::COLOR_BGR2GRAY);
    } 
    else if (inputImage.channels() == 1 && inputImage.depth() != CV_8U) {
        // 단일 채널이지만 float 같은 경우
        inputImage.convertTo(gray, CV_8U, 255.0);
    } 
    else if (inputImage.channels() == 1 && inputImage.depth() == CV_8U) {
        // 이미 CV_8UC1이면 복사만
        gray = inputImage.clone();
    } 
    else {
        std::cerr << "지원되지 않는 이미지 형식입니다.\n";
        return;
    }

    // 이진화
    cv::Mat binary;
    cv::threshold(gray, binary, 200, 255, cv::THRESH_BINARY);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours) {
        if (contour.size() > 5) {
            cv::Point2f circ_center;
            float radius;
            cv::minEnclosingCircle(contour, circ_center, radius);
            cv::circle(frame, circ_center, static_cast<int>(radius), cv::Scalar(0, 255, 0), 2);
        }
    }
}

// 차량 전방 평균 밝기 상태 판단
double glare_detector::isBrightArea(const cv::Mat& frame) {
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    return cv::mean(gray)[0]/255;
}

// 차량 전방 밝기 표준편차 계산
double glare_detector::isStandardArea(const cv::Mat& frame) {
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    // 평균과 표준편차 계산
    cv::Scalar mean, stddev;
    cv::meanStdDev(gray, mean, stddev);

    // 정규화된 표준편차 반환 (0~1 범위)
    return stddev[0] / 128.0;  // 128은 대략적인 최대 대비 기준값
}
