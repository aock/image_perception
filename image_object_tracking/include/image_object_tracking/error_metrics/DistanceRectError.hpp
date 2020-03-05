#pragma once
#ifndef IMAGE_PERCEPTION_TRACKING_DISTANCE_RECT_ERROR_HPP
#define IMAGE_PERCEPTION_TRACKING_DISTANCE_RECT_ERROR_HPP

#include "RectErrorMetricBase.hpp"

namespace image_object_tracking {

class DistanceRectError : public RectErrorMetricBase {
public:
    virtual float e(const cv::Rect2f& a, const cv::Rect2f& b) const
    {
        return std::pow(a.x - b.x, 2.0) + std::pow(a.y - b.y, 2.0)
        + std::pow(a.height - b.height, 2.0) + std::pow(a.width - b.width, 2.0);
    }
};

} // namespace image_object_tracking

#endif // IMAGE_PERCEPTION_TRACKING_DISTANCE_RECT_ERROR_HPP