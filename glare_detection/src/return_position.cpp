#include "return_position.h"
#include <cmath>

cv::Point2f glare_position::getGlareCoordinates(const cv::Mat& binaryMask) {
    std::vector<cv::Point> nonzero_points;
    cv::findNonZero(binaryMask, nonzero_points);
    if (nonzero_points.empty()) return cv::Point2f(-1.0f, -1.0f);

    cv::Point2f center(0.0f, 0.0f);
    for (const auto& pt : nonzero_points) {
        center.x += pt.x;
        center.y += pt.y;
    }
    center.x /= nonzero_points.size();
    center.y /= nonzero_points.size();
    return center;
}

double glare_position::getGlareArea() const {
    return gd.getDetectedArea();
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

//compute avg in queue for push
position_queue::Coord position_queue::computeAverageOfValid() const {
    int sum_x = 0, sum_y = 0, count = 0;
    for (const auto& [coord, valid] : queue_) {
        if (valid) {
            sum_x += coord.x;
            sum_y += coord.y;
            count++;
        }
    }
    if (count == 0) return {0, 0};  // 최초 입력은 항상 무효가 아님
    return {sum_x / (float)count, sum_y / (float)count};
}

bool position_queue::isWithinRange(const Coord& a, const Coord& b, int threshold) const {
    return std::abs(a.x - b.x) <= threshold && std::abs(a.y - b.y) <= threshold;
}
