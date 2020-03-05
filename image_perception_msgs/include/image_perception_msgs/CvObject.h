#pragma once
#ifndef IMAGE_PERCEPTION_MSGS_CV_OBJECT
#define IMAGE_PERCEPTION_MSGS_CV_OBJECT

#include <opencv2/core.hpp>
#include <string>

namespace image_perception_msgs {

struct CvObject {
    int id;
    cv::Rect2f rect;
    float confidence;
    std::string name;
};

} // namespace image_perception_msgs


#endif // IMAGE_PERCEPTION_MSGS_CV_OBJECT