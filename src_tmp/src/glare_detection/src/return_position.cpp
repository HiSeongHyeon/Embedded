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
    if(b.x ==0 && b.y==0) return 1;
    return std::abs(a.x - b.x) <= threshold && std::abs(a.y - b.y) <= threshold;
}

// cv::Point2f glare_position::getMaxCombinedCenter(const cv::Mat& combined) {
//     double minVal, maxVal;
//     cv::Point maxLoc;
//     cv::minMaxLoc(combined, &minVal, &maxVal, nullptr, &maxLoc);

//     // 최대값 주변 영역 추출
//     cv::Mat bin;
//     cv::threshold(combined, bin, maxVal - 1e-5, 1.0, cv::THRESH_BINARY);  // max 영역만
//     return getGlareCoordinates(bin);
// }

// cv::Point2f glare_position::getMaxCombinedCenter(const cv::Mat& combined) {
//     double minVal, maxVal;
//     cv::Point maxLoc;
//     cv::minMaxLoc(combined, &minVal, &maxVal, nullptr, &maxLoc);

//     // max 값에 가까운 영역만 추출
//     cv::Mat bin;
//     cv::threshold(combined, bin, maxVal - 1e-5, 1.0, cv::THRESH_BINARY);
//     bin.convertTo(bin, CV_8U);  // contour 분석을 위해 8비트로 변환

//     // contour를 통해 연결된 영역 찾기
//     std::vector<std::vector<cv::Point>> contours;
//     cv::findContours(bin, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

//     const double AREA_THRESHOLD = 10.0;  // 최소 면적 조건
//     bool valid_region_found = false;

//     for (const auto& contour : contours) {
//         if (cv::contourArea(contour) >= AREA_THRESHOLD) {
//             valid_region_found = true;
//             break;
//         }
//     }

//     if (!valid_region_found) {
//         return cv::Point2f(-1, -1);  // 조건 만족하는 glare 없음
//     }

//     // 조건 만족 시만 glare 중심 계산
//     return getGlareCoordinates(bin);
// }

cv::Point2f glare_position::getMaxCombinedCenter(const cv::Mat& combined) {
    // Step 1: 유효한 값(> 0)을 가진 영역을 binary로 만듦
    cv::Mat bin;
    cv::threshold(combined, bin, 1e-5, 1.0, cv::THRESH_BINARY);
    bin.convertTo(bin, CV_8U);  // findContours는 8-bit 이미지 필요

    // Step 2: 연결된 blob들 추출
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bin, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) {
        return cv::Point2f(-1, -1);  // 유효한 영역 없음
    }

    // Step 3: 가장 큰 contour 선택
    double max_area = 0.0;
    std::vector<cv::Point> maxContour;
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area > max_area) {
            max_area = area;
            maxContour = contour;
        }
    }

    // Step 4: 선택된 contour를 기반으로 mask 생성
    cv::Mat largestRegion = cv::Mat::zeros(combined.size(), CV_8U);

    // if (!contours.empty()) {
    //     cv::drawContours(largestRegion, std::vector<std::vector<cv::Point>>{maxContour}, -1, cv::Scalar(0,255,0), 2);
    // }
    cv::drawContours(largestRegion, std::vector<std::vector<cv::Point>>{maxContour}, -1, 255, cv::FILLED);

    // Step 5: getGlareCoordinates()에 binary mask 전달
    return getGlareCoordinates(largestRegion);
}

cv::Point2f glare_position::getPriorityBasedGlareCenter(const cv::Mat& priority, const cv::Mat& gphoto, const cv::Mat& ggeo, glare_detector& gd) {
    for (int level = 1; level <= 3; ++level) {
        cv::Mat priority_mask = (priority == level);
        if (cv::countNonZero(priority_mask) == 0) continue;

        // cv::Mat combined = gd.combineMapsbyprod(gphoto, ggeo);
        // cv::Mat masked_combined;
        // combined.copyTo(masked_combined, priority_mask);

        return getMaxCombinedCenter(priority_mask); // 픽셀 하나 뱉는게 아니라 영역에 대해 판단해보게끔
    }
    return cv::Point2f(-1, -1);  // No glare detected
}

position_queue::Coord position_queue::getAvgCoord() const {
    return avgCoord;    
}
