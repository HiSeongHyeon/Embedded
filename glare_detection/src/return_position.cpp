#include "return_position.h"

std::pair<int, int> solar_position::getSunCoordinates(const cv::Mat& frame) {
    sd.findSun(frame);
    return sd.getSunCoordinates();
}

double solar_position::getSunArea() const {
    return sd.getDetectedArea();
}
