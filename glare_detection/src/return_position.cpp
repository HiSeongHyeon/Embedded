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
    bool valid = false;
    int valid_count = 0;

    avgCoord = computeAverageOfValid();
    valid = isWithinRange(coord, avgCoord);

    if (queue_.size() >= max_size_) {
        queue_.pop_front();
    }

    queue_.emplace_back(coord, valid);
}

// check duration
bool position_queue::shouldReturnAverage() const {
    int valid_count = 0;
    for (const auto& entry : queue_) {
        if (entry.second) valid_count++;
    }
    return valid_count >= 15;
}

// // get avg at main loop
// position_queue::Coord position_queue::getAveragePosition() const {
//     int sum_x = 0, sum_y = 0, count = 0;
//     for (const auto& [coord, valid] : queue_) {
//         if (valid) {
//             sum_x += coord.first;
//             sum_y += coord.second;
//             count++;
//         }
//     }
//     if (count == 0) return {0, 0};
//     return {sum_x / count, sum_y / count};
// }

//compute avg in queue for push
position_queue::Coord position_queue::computeAverageOfValid() const {
    int sum_x = 0, sum_y = 0, count = 0;
    for (const auto& [coord, valid] : queue_) {
        if (valid) {
            sum_x += coord.first;
            sum_y += coord.second;
            count++;
        }
    }
    if (count == 0) return {0, 0};  // 최초 입력은 항상 무효가 아님
    return {sum_x / count, sum_y / count};
}

bool position_queue::isWithinRange(const Coord& a, const Coord& b, int threshold) const {
    return std::abs(a.first - b.first) <= threshold && std::abs(a.second - b.second) <= threshold;
}
