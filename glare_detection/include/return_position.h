#ifndef RETURN_POSITION_H
#define RETURN_POSITION_H

#include "SunDetector.h"
#include <deque>
#include <utility>

class solar_position {
public:
    sunDetector sd;
    std::pair<int, int> getSunCoordinates(const cv::Mat& frame);
    double getSunArea() const;
};

class position_queue {
    public:
        using Coord = std::pair<int, int>;
        using Entry = std::pair<Coord, bool>;
    
        position_queue(size_t max_size = 20);
    
        void push(const Coord& coord);
        bool shouldReturnAverage() const;
    
       
        Coord getAvgCoord() const;
    
    private:
        std::deque<Entry> queue_;
        size_t max_size_;

        Coord avgCoord;

        Coord computeAverageOfValid() const;
        bool isWithinRange(const Coord& a, const Coord& b, int threshold) const;
        
    };
    
#endif
