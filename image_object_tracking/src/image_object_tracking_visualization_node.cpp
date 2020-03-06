#include <ros/ros.h>
#include <memory>
#include <unordered_map>
#include <sstream>
#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

// ros packages
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>

// image_perception_msgs
#include <image_perception_msgs/Label.h>
#include <image_perception_msgs/LabelsStamped.h>
#include <image_perception_msgs/TrackedObjectsStamped.h>
#include <image_perception_msgs/CvObject.h>
#include <image_perception_msgs/Conversions.hpp>

using namespace image_perception_msgs;

image_transport::Publisher tracked_image_pub;


void drawTrackedObjects(
    cv::Mat& img,
    const std::unordered_map<unsigned int, CvObject>& objects)
{
    for(const auto& obj : objects)
    {
        unsigned int obj_id = obj.first;
        const CvObject& label = obj.second;
        const int left = label.rect.x * img.cols;
        const int top = label.rect.y * img.rows;
        const int right = (label.rect.x + label.rect.width) * img.cols;
        const int bottom =  (label.rect.y + label.rect.height) * img.rows;
        
        cv::rectangle(img,
            cv::Point(left, top),
            cv::Point(right, bottom),
            cv::Scalar(0, 255, 0)
        );
        
        std::string confidence = cv::format("%.2f", label.confidence);
        std::stringstream ss;
        
        if(!label.name.empty())
        {
            std::string name = label.name;
            ss << obj_id << ", " << name << ": " << confidence;
        } else {
            ss << obj_id << ", " << label.id << ": " << confidence;
        }

        std::string label_name = ss.str();

        // TODO choose these params depending on image size
        double fontScale = 0.7;
        int fontThickness = 2;

        int baseLine;

        cv::Size labelSize = cv::getTextSize(label_name, cv::FONT_HERSHEY_SIMPLEX, fontScale, fontThickness, &baseLine);
        int top_label = cv::max(top, labelSize.height);
        cv::rectangle(img, cv::Point(left, top - labelSize.height),
              cv::Point(left + labelSize.width, top + baseLine), cv::Scalar(255, 255, 255), cv::FILLED);
        cv::putText(img, label_name, cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX, fontScale, cv::Scalar(), fontThickness);
    }
}


void trackedObjectsDrawingCallback(
    const sensor_msgs::ImageConstPtr& image,
    const TrackedObjectsStampedConstPtr& objects)
{
    cv::Mat image_cv = ros_to_cv(image);
    auto tracked_objects = ros_to_cv(objects->objects);
    drawTrackedObjects(image_cv, tracked_objects);
    sensor_msgs::ImagePtr labeled_image_msg = cv_to_ros(image_cv);
    labeled_image_msg->header.frame_id = image->header.frame_id;
    labeled_image_msg->header.stamp = image->header.stamp;
    tracked_image_pub.publish(labeled_image_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_object_tracking_vizualization_node");

    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    image_transport::ImageTransport it(nh_p);
    tracked_image_pub = it.advertise("image_with_tracked_objects", 1);

    // Image Subscriber
    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/camera/image_raw", 1);
    
    // Tracked Objects Subscriber
    message_filters::Subscriber<TrackedObjectsStamped> 
        tracked_objects_sub(nh, "/image_object_tracking_node/tracked_objects", 1);

    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::Image, TrackedObjectsStamped> 
            ApproxImageTrackedObjectsSync;

    message_filters::Synchronizer<ApproxImageTrackedObjectsSync> sync2(
        ApproxImageTrackedObjectsSync(10), image_sub, tracked_objects_sub
    );

    sync2.registerCallback(boost::bind(&trackedObjectsDrawingCallback, _1, _2));

    ros::spin();
    return 0;
}