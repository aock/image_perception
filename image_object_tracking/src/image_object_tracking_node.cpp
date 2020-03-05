#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include <image_perception_msgs/LabelsStamped.h>
#include <image_perception_msgs/CvObject.h>
#include <image_perception_msgs/TrackedObjectsStamped.h>
#include <image_perception_msgs/Conversions.hpp>


#include "image_object_tracking/ObjectTracker.hpp"

using namespace image_object_tracking;
using namespace image_perception_msgs;

std::shared_ptr<ImageObjectTracker> object_tracker;
std::shared_ptr<image_transport::ImageTransport> it;
std::shared_ptr<image_transport::ImageTransport> it_p;

ros::Publisher tracked_objects_pub;
std::shared_ptr<ros::NodeHandle> nh_p;



float forgetTime;
float rectErrorTheshold;
std::string rectErrorMetric;
bool dynamicParams;

void updateParams()
{
    nh_p->param<float>("rectErrorThresh", rectErrorTheshold, 0.5);
    nh_p->param<std::string>("rectErrorMetric", rectErrorMetric, "intersection");
    nh_p->param<float>("forgetTime", forgetTime, 2.0);
    nh_p->param<bool>("dynamicParams", dynamicParams, true);
}

void updateTracker()
{
    if(object_tracker)
    {
        object_tracker->setErrorMetric(rectErrorMetric);
        object_tracker->setErrorThreshold(rectErrorTheshold);
        object_tracker->setRemoveTime(forgetTime);
    }
}

void objectsCallback(const LabelsStamped::ConstPtr& msg)
{
    if(!object_tracker)
    {
        object_tracker.reset(new ImageObjectTracker);
    }
    if(dynamicParams)
    {
        updateParams();
        updateTracker();
    }

    std::vector<CvObject> cv_objects
        = ros_to_cv(msg->labels);
    object_tracker->update(cv_objects);

    auto results = object_tracker->get();

    TrackedObjectsStamped tracked_objects;
    tracked_objects.header = msg->header;
    tracked_objects.objects = cv_to_ros(results);
    tracked_objects_pub.publish(tracked_objects);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_object_tracking_node");

    ROS_INFO_STREAM("Image Object Tracker started.");
    ros::NodeHandle nh;
    nh_p.reset(new ros::NodeHandle("~"));

    updateParams();

    tracked_objects_pub = nh_p->advertise<TrackedObjectsStamped>("tracked_objects", 1);

    ros::Subscriber sub = nh.subscribe<LabelsStamped>(
        "/image_object_detection_node/labels", 1, objectsCallback);

    ros::spin();

    return 0;
}