#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <unordered_set>

#include <opencv2/core.hpp>

#include <boost/filesystem.hpp>

#include <image_perception_msgs/CvObject.h>

namespace fs = ::boost::filesystem;

namespace image_object_detection {

class CvNeuralNetwork;

/**
 * @brief ObjectDetector - Ros less ObjectDetector
 * 
 */
class ObjectDetector {
public:
    ObjectDetector(
        std::string model_dir,
        std::string model_name,
        cv::Size input_size = cv::Size(0,0)
    );

    // PUBLIC FUNCTIONS

    std::vector<image_perception_msgs::CvObject> detect(
        cv::Mat& img,
        bool drawLabels = true
    );

    // GETTER SETTER

    std::string modelName() const;

    void setConfidenceThreshold(float ct);
    float confidenceThreshold() const;

    void setScale(float scale);
    float scale() const;

    void setMean(cv::Scalar mean);
    cv::Scalar mean() const;

    void setSwapRB(bool swap);
    bool swapRB() const;

    void setComputingUnit(std::string computing_unit);
    std::string computingUnit() const;

    cv::Size inputSize() const;

protected:
    void initNetwork();

    void preprocess(const cv::Mat& frame);
    std::vector<std::string> readClasses();
    std::shared_ptr<CvNeuralNetwork> buildNet();
    void fillLabelNames(std::vector<image_perception_msgs::CvObject>& labels);

    std::vector<image_perception_msgs::CvObject> filterLabels(const std::vector<image_perception_msgs::CvObject>& labels);
    std::vector<image_perception_msgs::CvObject> generateLabels(const std::vector<cv::Mat>& outs);

    
    const std::string m_model_dir;
    const std::string m_model_name;
    cv::Size m_input_size;

    std::vector<std::string> m_classes;
    std::vector<cv::String> m_output_layer_names;
    std::vector<cv::String> m_output_layer_types;

    std::shared_ptr<CvNeuralNetwork> m_cv_net;

private:

    cv::Scalar colorFromPercentage(double percent)
    {
        // TODO
        return cv::Scalar::all(0);
    }

    float m_confidence_threshold;
    float m_scale;
    cv::Scalar m_mean;
    bool m_swap_rb;
    std::string m_computing_unit;
    
};

} // namespace image_object_detection