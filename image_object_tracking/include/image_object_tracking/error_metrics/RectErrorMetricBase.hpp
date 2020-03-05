#pragma once
#ifndef IMAGE_PERCEPTION_TRACKING_RECT_ERROR_BASE_HPP
#define IMAGE_PERCEPTION_TRACKING_RECT_ERROR_BASE_HPP

#include <opencv2/core.hpp>

namespace image_object_tracking {

class RectErrorMetricBase {
public:
    virtual ~RectErrorMetricBase() {}
    virtual float e(const cv::Rect2f& a, const cv::Rect2f& b) const = 0;
};

} // namespace image_object_tracking

#endif // IMAGE_PERCEPTION_TRACKING_RECT_ERROR_BASE_HPP