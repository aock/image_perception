#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include <memory>

#include "image_object_detection/ObjectDetector.hpp"

#include "image_perception_msgs/Label.h"
#include "image_perception_msgs/LabelsStamped.h"
#include "image_perception_msgs/Conversions.hpp"

using namespace image_object_detection;
using namespace image_perception_msgs;

std::shared_ptr<ObjectDetector> object_detector;
ros::Publisher labels_pub;
std::shared_ptr<ros::NodeHandle> nh;
std::shared_ptr<ros::NodeHandle> nh_p;
std::shared_ptr<image_transport::ImageTransport> it;


std::string model_dir;

// reinit needed if these params change
std::string model_name;
cv::Size inputSize;

// no reinit needed
float scale;
// float mean;
std::vector<float> mean;
bool swapRB;
float confidenceThreshold;
std::string computingUnit;

void refresh_params()
{
    // std::cout << "refresh_params" << std::endl;
    nh_p->getParam("modelName", model_name);
    nh_p->getParam(model_name + "/computingUnit", computingUnit);
    nh_p->getParam(model_name + "/scale", scale);
    nh_p->getParam(model_name + "/mean", mean);
    nh_p->getParam(model_name + "/swapRB", swapRB);
    nh_p->getParam(model_name + "/confidenceThreshold", confidenceThreshold);
    
    std::vector<int> inputSizeParam;
    if(nh_p->getParam(model_name + "/inputSize", inputSizeParam))
    {
        inputSize = cv::Size(inputSizeParam[0], inputSizeParam[1]);
    } else {
        std::cout << "ERROR: inputSize" << std::endl;
    }
}

void refresh_object_detector()
{
    if(object_detector)
    {
        if(model_name != object_detector->modelName() || inputSize != object_detector->inputSize())
        {
            std::cout << "Model Name was changed!" << std::endl;
            object_detector.reset(new ObjectDetector(model_dir, model_name, inputSize));
        }
        object_detector->setScale(scale);
        object_detector->setMean(cv::Scalar(mean[0], mean[1], mean[2]));
        object_detector->setConfidenceThreshold(confidenceThreshold);
        object_detector->setComputingUnit(computingUnit);
    }
}

void image_cb(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat frame = ros_to_cv(msg);

    if(!frame.empty())
    {
        if(!object_detector)
        {
            // set object detector
            object_detector.reset(new ObjectDetector(model_dir, model_name, inputSize));
        }

        refresh_params();
        refresh_object_detector();

        auto cv_objects = object_detector->detect(frame);

        LabelsStamped labels;
        labels.header.stamp = msg->header.stamp;
        labels.header.frame_id = msg->header.frame_id;
        labels.labels = cv_to_ros(cv_objects);

        if(labels.labels.size() > 0)
        {
            labels_pub.publish(labels);
        }

    } else {
        ROS_ERROR("Image Ros -> OpenCV failed");
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_object_detection_node");

    model_dir = ros::package::getPath("image_object_detection") + "/models";

    nh.reset(new ros::NodeHandle);
    nh_p.reset(new ros::NodeHandle("~"));
    it.reset(new image_transport::ImageTransport(*nh));

    inputSize = cv::Size(0,0);
    nh_p->param<std::string>("modelName", model_name, "YoloV3Tiny");
    refresh_params();

    object_detector.reset(new ObjectDetector(model_dir, model_name, inputSize));
    refresh_object_detector();

    // Publisher
    labels_pub = nh_p->advertise<LabelsStamped>("labels", 1);

    // Subscriber
    image_transport::Subscriber image_sub = it->subscribe("/camera/image_raw", 1, image_cb);

    ros::spin();

    nh.reset();
    nh_p.reset();
    it.reset();
    object_detector.reset();

    return 0;
}