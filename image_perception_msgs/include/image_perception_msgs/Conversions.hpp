#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <memory>
#include <unordered_map>
#include <utility>

#include <image_perception_msgs/CvObject.h>
#include <image_perception_msgs/LabelsStamped.h>
#include <image_perception_msgs/Label.h>
#include <image_perception_msgs/TrackedObject.h>

namespace image_perception_msgs {

// DEFINITIONS

/**
 * @brief Convert Ros Image <-> Cv Mat
 * 
 */
cv::Mat ros_to_cv(const sensor_msgs::ImageConstPtr& msg);
sensor_msgs::ImagePtr cv_to_ros(const cv::Mat& img);

CvObject ros_to_cv(const Label& label);
Label cv_to_ros(const CvObject& object);

std::vector<CvObject> ros_to_cv(const std::vector<Label>& labels);
std::vector<Label> cv_to_ros(const std::vector<CvObject>& objects);

TrackedObject cv_to_ros(const std::pair<unsigned int, CvObject>& object);
std::pair<unsigned int, CvObject> ros_to_cv(const TrackedObject& ros_obj);

std::vector<TrackedObject> cv_to_ros(
    const std::unordered_map<unsigned int, CvObject>& tracked_objects);

std::unordered_map<unsigned int, CvObject> ros_to_cv(
    const std::vector<TrackedObject>& ros_objects);

} // namespace image_perception_msgs