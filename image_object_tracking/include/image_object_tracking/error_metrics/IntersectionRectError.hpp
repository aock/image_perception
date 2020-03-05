#pragma once
#ifndef IMAGE_PERCEPTION_TRACKING_INTERSECTION_RECT_ERROR_HPP
#define IMAGE_PERCEPTION_TRACKING_INTERSECTION_RECT_ERROR_HPP

#include "RectErrorMetricBase.hpp"


namespace image_object_tracking {

class IntersectionRectError : public RectErrorMetricBase {
public:
    virtual float e(const cv::Rect2f& a, const cv::Rect2f& b) const
    {
        const cv::Rect2f int_rect = a & b;
        const cv::Rect2f union_rect = a | b;
        return 1.0 - (int_rect.area() / union_rect.area());
    }
};

} // namespace image_object_tracking

#endif // IMAGE_PERCEPTION_TRACKING_INTERSECTION_RECT_ERROR_HPP