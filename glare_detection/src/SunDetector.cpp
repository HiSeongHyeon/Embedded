#include "SunDetector.h"

sunDetector::sunDetector() : sunCenter(-1, -1), detectedArea(0.0), sunFound(false) {}

void sunDetector::startVideo(const cv::Mat& frame) {
    currentFrame = frame.clone();
}

void sunDetector::endVideo() {
    currentFrame.release();
}

void sunDetector::findSun(const cv::Mat& frame) {
    currentFrame = frame.clone();
    cv::Mat hsv, mask;
    cv::cvtColor(currentFrame, hsv, cv::COLOR_BGR2HSV);

    // HSV 범위에서 밝은 태양 탐지
    cv::Scalar lower(0, 0, 250);
    cv::Scalar upper(180, 30, 255);
    cv::inRange(hsv, lower, upper, mask);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double maxArea = 0.0;
    cv::Point maxCenter(-1, -1);

    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area > maxArea) {
            cv::Moments m = cv::moments(contour);
            if (m.m00 != 0) {
                maxCenter = cv::Point(static_cast<int>(m.m10 / m.m00), static_cast<int>(m.m01 / m.m00));
                maxArea = area;
            }
        }
    }

    sunCenter = maxCenter;
    detectedArea = maxArea;
    sunFound = (maxArea > 0);
}

std::pair<int, int> sunDetector::getSunCoordinates() const {
    return sunFound ? std::make_pair(sunCenter.x, sunCenter.y) : std::make_pair(-1, -1);
}

double sunDetector::getDetectedArea() const {
    return detectedArea;
}

void sunDetector::drawSun(cv::Mat& frame) const {
    if (sunFound) {
        int radius = std::clamp(static_cast<int>(sqrt(detectedArea / CV_PI)), 10, 100);
        cv::circle(frame, sunCenter, radius, cv::Scalar(0, 255, 255), 3);
    }
}

int sunDetector::isBrightArea(const cv::Mat& frame) {
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    if (cv::mean(gray)[0]>50)
        return 1;
    else
        return 0;
}