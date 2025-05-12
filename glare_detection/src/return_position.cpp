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
    auto avg = computeAverageOfValid();
    bool is_valid = isWithinRange(coord, avg, 10);

    if (queue_.size() >= max_size_) {
        queue_.pop_front();
    }

    queue_.emplace_back(coord, is_valid);

    // 평균 좌표 갱신
    avgCoord = computeAverageOfValid();
}

// check duration
bool position_queue::shouldReturnAverage() const {
    int valid_count = 0;

    for (const auto& [coord, valid] : queue_) {
        if (valid) valid_count++;
    }

    return valid_count >= 15;
}

//compute avg in queue for push
position_queue::Coord position_queue::computeAverageOfValid() const {
    int sum_x = 0, sum_y = 0;
    int count = 0;

    for (const auto& [coord, valid] : queue_) {
        if (valid) {
            sum_x += coord.x;
            sum_y += coord.y;
            count++;
        }
    }

    if (count == 0) return {0, 0};
    return {sum_x / (float)count, sum_y / (float)count};
}

bool position_queue::isWithinRange(const Coord& a, const Coord& b, int threshold) const {
    if (b.x ==0 && b.y==0) return 1;
    return std::abs(a.x - b.x) <= threshold && std::abs(a.y - b.y) <= threshold;
}

position_queue::Coord position_queue::getAvgCoord() const{
    return avgCoord;
}