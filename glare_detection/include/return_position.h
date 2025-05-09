#ifndef RETURN_POSITION_H
#define RETURN_POSITION_H

#include "glare_detector.h"
#include <deque>
#include <utility>

class glare_position {
    public:
        glare_detector gd;
        cv::Point2f getGlareCoordinates(const cv::Mat& frame);
        double getGlareArea() const;
    };

class position_queue {
    public:
        using Coord = cv::Point2f;
        using Entry = std::pair<Coord, bool>;

        Coord avgCoord;
        position_queue(size_t max_size = 20);
    
        void push(const Coord& coord);
        bool shouldReturnAverage() const;
        //Coord getAveragePosition() const;
        
    private:
        std::deque<Entry> queue_;
        size_t max_size_;

        Coord computeAverageOfValid() const;
        bool isWithinRange(const Coord& a, const Coord& b, int threshold = 2) const;
    };

#endif
