#ifndef SUN_DETECTOR_H
#define SUN_DETECTOR_H

#include <opencv2/opencv.hpp>

class sunDetector {
private:
    cv::Mat currentFrame;
    cv::Point sunCenter;
    double detectedArea;
    bool sunFound;

public:
    sunDetector();
    void startVideo(const cv::Mat& frame);
    void endVideo();
    void findSun(const cv::Mat& frame);
    std::pair<int, int> getSunCoordinates() const;
    double getDetectedArea() const;
    void drawSun(cv::Mat& frame) const;

    int isBrightArea(const cv::Mat& frame);
};

#endif
