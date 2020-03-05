#pragma once
#ifndef IMAGE_PERCEPTION_OBJECT_TRACKER_HPP
#define IMAGE_PERCEPTION_OBJECT_TRACKER_HPP

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <image_perception_msgs/CvObject.h>

#include "error_metrics/RectErrorMetricBase.hpp"

namespace image_object_tracking {

class CvTracker;

class ImageObjectTracker {
public:
    ImageObjectTracker();

    void setErrorThreshold(float error_thresh)
    {
        m_error_thresh = error_thresh;
    }

    void setRemoveTime(double remove_time)
    {
        m_remove_time = remove_time;
    }

    void setErrorMetric(std::string error_metric);

    void update(const std::vector<image_perception_msgs::CvObject>& objects);   
    void update(const std::vector<image_perception_msgs::CvObject>& objects, double dt);

    std::unordered_map<unsigned int, image_perception_msgs::CvObject> get() const;

private:

    inline float objectError(
        const image_perception_msgs::CvObject& a, const image_perception_msgs::CvObject& b) const;

    std::chrono::time_point<std::chrono::steady_clock> m_last_updated;

    std::unordered_map<
        unsigned int, std::shared_ptr<CvTracker>
        > m_trackers;
    

    float m_error_thresh;
    double m_remove_time;

    std::shared_ptr<RectErrorMetricBase> m_error_metric;

};

}

#endif // IMAGE_PERCEPTION_OBJECT_TRACKER_HPP