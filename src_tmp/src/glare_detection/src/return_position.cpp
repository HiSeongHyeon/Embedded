#include "return_position.h"
#include <cmath>

position_queue::position_queue(size_t max_size) : max_size_(max_size) {}

// glare 좌표 queue에 추가
void position_queue::push(const Coord& coord) {
    bool valid = true;
    int valid_count = 0;

    avgCoord = computeAverageOfValid();
    valid = isWithinRange(coord, avgCoord);

    if (queue_.size() >= max_size_) {
        queue_.pop_front();
    }
    std::cout << "[DEBUG] queue size: " << queue_.size() << std::endl;

    queue_.emplace_back(coord, valid);
}

// check duration
int position_queue::shouldReturnAverage() const {
    int valid_count = 0;
    for (const auto& entry : queue_) {
        if (entry.second) valid_count++;
    }

    if (valid_count >= 7){      // glare가 존재하는 경우
        return 1;
    }
    else if (valid_count <4){   // galre가 존재하다가 사라진 경우 or 사라졌다가 다시 나타난 경우
        return -1;
    }
    else{                       // glare가 존재하지 않는 경우
        return 0;
    }

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
    if (count == 0) return {-1, -1};  // glare가 없으면 무효한 좌표 반환
    return {sum_x / (float)count, sum_y / (float)count};
}

// glare 좌표의 유효성을 판단
bool position_queue::isWithinRange(const Coord& pos, const Coord& avg_pos, int threshold) const {
    if(avg_pos.x <0 || avg_pos.y < 0) threshold = 2000; // glare가 없는 상태에서 새롭게 인식하면 항상 유효한 좌표로 받음

    if(pos.x < 0 || pos.y < 0) return false;            
    
    if(avg_pos.x ==0 && avg_pos.y==0) return true; // 최초 입력이 음수가 아니라면 true로 받음

    return std::abs(pos.x - avg_pos.x) <= threshold && std::abs(pos.y - avg_pos.y) <= threshold;
}

// glare 평균 좌표 반환
position_queue::Coord position_queue::getAvgCoord() const {
    return avgCoord;    
}

// max contour 기반 glare 중심 좌표 계산
cv::Point2f glare_position::getGlareCoordinates(const cv::Mat& binaryMask) {
    std::vector<cv::Point> nonzero_points;
    cv::findNonZero(binaryMask, nonzero_points);
    if (nonzero_points.empty()) return cv::Point2f(-7.0f, -7.0f);

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

// glare로 인식한 영역 중 가장 큰 contour 선택하여 최종 glare로 판단
cv::Point2f glare_position::getMaxCombinedCenter(const cv::Mat& combined) {
    if (combined.empty()){
        return cv::Point2f(-5, -5);
    }
    // 유효한 값(> 0)을 가진 영역을 binary로 만듦
    cv::Mat bin;
    cv::threshold(combined, bin, 1e-3, 1.0, cv::THRESH_BINARY);
    bin.convertTo(bin, CV_8U);  // findContours는 8-bit 이미지 필요

    // 연결된 blob들 추출
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bin, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) {
        return cv::Point2f(-3, -3);  // 유효한 영역 없음
    }

    // 가장 큰 contour 선택
    double max_area = 0.0;
    std::vector<cv::Point> maxContour;
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area > max_area) {
            max_area = area;
            maxContour = contour;
        }
    }

    // 선택된 contour를 기반으로 mask 생성
    cv::Mat largestRegion = cv::Mat::zeros(combined.size(), CV_8U);
    if(maxContour.empty()){
        return cv::Point2f(-1, -1);
    }
    else{
        cv::drawContours(largestRegion, std::vector<std::vector<cv::Point>>{maxContour}, -1, cv::Scalar(255), cv::FILLED);
    }

    return getGlareCoordinates(largestRegion);
}


// priority 기반 glare 판단
cv::Point2f glare_position::getPriorityBasedGlareCenter(const cv::Mat& priority, const cv::Mat& gphoto, const cv::Mat& ggeo, glare_detector& gd) {
    for (int level = 1; level < 3; ++level) {
        debug_color = (level - 1)*255;
        cv::Mat priority_mask = (priority == level);
        if (cv::countNonZero(priority_mask) == 0) continue;

        return getMaxCombinedCenter(priority_mask);
    }
    return cv::Point2f(-1, -1);  // No glare detected
}

