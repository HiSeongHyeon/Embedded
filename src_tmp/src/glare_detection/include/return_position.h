#ifndef RETURN_POSITION_H
#define RETURN_POSITION_H

#include "SunDetector.h"

class solar_position {
public:
  sunDetector sd;
  std::pair<int, int> getSunCoordinates(const cv::Mat &frame);
  double getSunArea() const;
};

#endif
