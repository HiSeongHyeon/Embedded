#include "return_position.h"
#include <cmath>

std::pair<int, int> solar_position::getSunCoordinates(const cv::Mat& frame) {
    sd.findSun(frame);
    return sd.getSunCoordinates();
}

double solar_position::getSunArea() const {
    return sd.getDetectedArea();
}

position_queue::position_queue(size_t max_size) : max_size_(max_size) {}

void position_queue::push(const Coord& coord) {
    auto avg = computeAverageOfValid();
    bool is_valid = isWithinRange(coord, avg, 10);

    if (queue_.size() >= max_size_) {
        queue_.pop_front();
    }

    queue_.emplace_back(coord, is_valid);

    // 평균 좌표 갱신
    avgCoord = computeAverageOfValid();
}

bool position_queue::shouldReturnAverage() const {
    int valid_count = 0;

    for (const auto& [coord, valid] : queue_) {
        if (valid) valid_count++;
    }

    return valid_count >= 15;
}

position_queue::Coord position_queue::computeAverageOfValid() const {
    int sum_x = 0, sum_y = 0;
    int count = 0;

    for (const auto& [coord, valid] : queue_) {
        if (valid) {
            sum_x += coord.first;
            sum_y += coord.second;
            count++;
        }
    }

    if (count == 0) return {0, 0};
    return {sum_x / count, sum_y / count};
}

bool position_queue::isWithinRange(const Coord& a, const Coord& b, int threshold) const {
    if (b.first == 0 && b.second == 0) return 1;
    return std::abs(a.first - b.first) <= threshold && std::abs(a.second - b.second) <= threshold;
}

position_queue::Coord position_queue::getAvgCoord() const{
    return avgCoord;
}