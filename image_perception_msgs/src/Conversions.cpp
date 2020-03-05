#include <image_perception_msgs/Conversions.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h> 
#include <boost/regex.hpp>
#include "boost/endian/conversion.hpp"

namespace enc = sensor_msgs::image_encodings;

// IMPLEMENTATIONS
namespace image_perception_msgs {

static int depthStrToInt(const std::string depth) {
  if (depth == "8U") {
    return 0;
  } else if (depth == "8S") {
    return 1;
  } else if (depth == "16U") {
    return 2;
  } else if (depth == "16S") {
    return 3;
  } else if (depth == "32S") {
    return 4;
  } else if (depth == "32F") {
    return 5;
  }
  return 6;
}

int getCvType(const std::string& encoding)
{
    // Check for the most common encodings first
    if (encoding == enc::BGR8)   return CV_8UC3;
    if (encoding == enc::MONO8)  return CV_8UC1;
    if (encoding == enc::RGB8)   return CV_8UC3;
    if (encoding == enc::MONO16) return CV_16UC1;
    if (encoding == enc::BGR16)  return CV_16UC3;
    if (encoding == enc::RGB16)  return CV_16UC3;
    if (encoding == enc::BGRA8)  return CV_8UC4;
    if (encoding == enc::RGBA8)  return CV_8UC4;
    if (encoding == enc::BGRA16) return CV_16UC4;
    if (encoding == enc::RGBA16) return CV_16UC4;

    // For bayer, return one-channel
    if (encoding == enc::BAYER_RGGB8) return CV_8UC1;
    if (encoding == enc::BAYER_BGGR8) return CV_8UC1;
    if (encoding == enc::BAYER_GBRG8) return CV_8UC1;
    if (encoding == enc::BAYER_GRBG8) return CV_8UC1;
    if (encoding == enc::BAYER_RGGB16) return CV_16UC1;
    if (encoding == enc::BAYER_BGGR16) return CV_16UC1;
    if (encoding == enc::BAYER_GBRG16) return CV_16UC1;
    if (encoding == enc::BAYER_GRBG16) return CV_16UC1;

    // Miscellaneous
    if (encoding == enc::YUV422) return CV_8UC2;

    // Check all the generic content encodings
    boost::cmatch m;

    if (boost::regex_match(encoding.c_str(), m,
            boost::regex("(8U|8S|16U|16S|32S|32F|64F)C([0-9]+)"))) {
        return CV_MAKETYPE(depthStrToInt(m[1].str()), atoi(m[2].str().c_str()));
    }

    if (boost::regex_match(encoding.c_str(), m,
            boost::regex("(8U|8S|16U|16S|32S|32F|64F)"))) {
        return CV_MAKETYPE(depthStrToInt(m[1].str()), 1);
    }

    
    // throw Exception("Unrecognized image encoding [" + encoding + "]");
}

cv::Mat ros_to_cv(
    const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat ret;

    int source_type = getCvType(msg->encoding);
    
    int byte_depth = enc::bitDepth(msg->encoding) / 8;
    int num_channels = enc::numChannels(msg->encoding);


    if (msg->step < msg->width * byte_depth * num_channels)
    {
        std::stringstream ss;
        ss << "Image is wrongly formed: step < width * byte_depth * num_channels  or  " << msg->step << " != " <<
            msg->width << " * " << byte_depth << " * " << num_channels;
        std::cerr << ss.str() << std::endl;
        return ret;
    }

    if (msg->height * msg->step != msg->data.size())
    {
        std::stringstream ss;
        ss << "Image is wrongly formed: height * step != size  or  " << msg->height << " * " <<
                msg->step << " != " << msg->data.size();
        
        std::cerr << ss.str() << std::endl;
        return ret;
    }

    // If the endianness is the same as locally, share the data
    ret = cv::Mat(msg->height, msg->width, source_type, const_cast<uchar*>(&msg->data[0]), msg->step);
    if ((boost::endian::order::native == boost::endian::order::big && msg->is_bigendian) ||
        (boost::endian::order::native == boost::endian::order::little && !msg->is_bigendian) ||
        byte_depth == 1)
        return ret;

    // Otherwise, reinterpret the data as bytes and switch the channels accordingly
    ret = cv::Mat(msg->height, msg->width, CV_MAKETYPE(CV_8U, num_channels*byte_depth),
                    const_cast<uchar*>(&msg->data[0]), msg->step);
    cv::Mat mat_swap(msg->height, msg->width, ret.type());

    std::vector<int> fromTo;
    fromTo.reserve(num_channels*byte_depth);
    for(int i = 0; i < num_channels; ++i)
        for(int j = 0; j < byte_depth; ++j)
        {
        fromTo.push_back(byte_depth*i + j);
        fromTo.push_back(byte_depth*i + byte_depth - 1 - j);
        }
    cv::mixChannels(std::vector<cv::Mat>(1, ret), std::vector<cv::Mat>(1, mat_swap), fromTo);

    // Interpret mat_swap back as the proper type
    mat_swap.reshape(num_channels);

    return mat_swap;
}


sensor_msgs::ImagePtr cv_to_ros(
    const cv::Mat& img)
{
    // TODO without cv_bridge

    // if(img.channels() == 3)
    // {
    //     return cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    // } else if(img.channels() == 1) {
    //     return cv_bridge::CvImage(std_msgs::Header(), "mono8", img).toImageMsg();
    // } else {
    //     throw std::runtime_error("unkown number of image channels");
    // }
}

CvObject ros_to_cv(
    const Label& label)
{
    CvObject obj;
    obj.id = label.id;
    obj.rect.x = label.rect.x;
    obj.rect.y = label.rect.y;
    obj.rect.width = label.rect.w;
    obj.rect.height = label.rect.h;
    obj.confidence = label.confidence;
    obj.name = label.name;
    return obj;
}

Label cv_to_ros(
    const CvObject& object)
{
    Label label;
    label.id = object.id;
    label.rect.x = object.rect.x;
    label.rect.y = object.rect.y;
    label.rect.w = object.rect.width;
    label.rect.h = object.rect.height;
    label.confidence = object.confidence;
    label.name = object.name;
    return label;
}

std::vector<CvObject> ros_to_cv(
    const std::vector<Label>& labels)
{
    std::vector<CvObject> ret;
    ret.reserve(labels.size());
    
    for(const Label& l : labels)
    {
        ret.push_back(ros_to_cv(l));
    }

    return ret;
}

std::vector<Label> cv_to_ros(const std::vector<CvObject>& objects)
{
    std::vector<Label> ret;
    ret.reserve(objects.size());
    
    for(const CvObject& obj : objects)
    {
        ret.push_back(cv_to_ros(obj));
    }
    return ret;
}


TrackedObject cv_to_ros(const std::pair<unsigned int, CvObject>& object)
{
    TrackedObject ros_obj;
    ros_obj.obj_id = object.first;
    ros_obj.label = cv_to_ros(object.second);
    return ros_obj;
}

std::pair<unsigned int, CvObject> ros_to_cv(const TrackedObject& ros_obj)
{
    return {ros_obj.obj_id, ros_to_cv(ros_obj.label)};
}

std::vector<TrackedObject> cv_to_ros(
    const std::unordered_map<unsigned int, CvObject>& tracked_objects)
{
    std::vector<TrackedObject> ret;

    for(const std::pair<unsigned int, CvObject>& tracked_object : tracked_objects)
    {
        ret.push_back(cv_to_ros(tracked_object));
    }

    return ret;
}

std::unordered_map<unsigned int, CvObject> ros_to_cv(
    const std::vector<TrackedObject>& ros_objects)
{
    std::unordered_map<unsigned int, CvObject> ret;

    for(const TrackedObject& ros_obj : ros_objects)
    {
        ret.insert(ros_to_cv(ros_obj));
    }

    return ret;
}

} // namespace image_perception_msgs