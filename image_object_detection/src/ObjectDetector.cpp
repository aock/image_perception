#include <image_object_detection/ObjectDetector.hpp>

// includes that should be compiled with opencv 3.4.1 or higher
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>

namespace image_object_detection {

using namespace image_perception_msgs;

class CvNeuralNetwork {
public:
    CvNeuralNetwork(cv::dnn::Net net):net(net){}
    cv::dnn::Net net;
};

ObjectDetector::ObjectDetector(
    std::string model_dir,
    std::string model_name,
    cv::Size input_size)
:m_model_dir(model_dir)
,m_model_name(model_name)
,m_confidence_threshold(0.5)
,m_scale(0.00392)
,m_mean(0.0)
,m_input_size(input_size)
,m_computing_unit("")
{
    m_classes = readClasses();
    if(m_classes.size() == 0)
    {
        std::cout << "[ObjectDetector] WARNING: could not load label names. using ids only." << std::endl; 
    }
    initNetwork();
    if(m_cv_net && !m_cv_net->net.empty())
    {
        setComputingUnit("CPU");
    } else {
        std::cerr << "Cant initialize. Brain is not loaded!" << std::endl;
    }
}

std::vector<CvObject> ObjectDetector::detect(
    cv::Mat& img,
    bool drawLabels)
{
    std::vector<CvObject> labels;

    if(m_cv_net && !m_cv_net->net.empty())
    {   
        cv::Mat img_detection = img.clone();
        preprocess(img_detection);
        
        std::vector<cv::Mat> outs;
        m_cv_net->net.forward(outs, m_output_layer_names);
        
        // std::cout << "Raw Detections: " << outs.size() << std::endl;

        labels = generateLabels(outs);
        labels = filterLabels(labels);

        if(m_classes.size() > 0)
        {
            fillLabelNames(labels);
        }
    } else {
        std::cerr << "Cant detect. Brain is not loaded!" << std::endl;
    }

    return labels;
}

std::string ObjectDetector::modelName() const {
    return m_model_name;
}

void ObjectDetector::setConfidenceThreshold(float ct)
{
    m_confidence_threshold = ct;
}

float ObjectDetector::confidenceThreshold() const
{
    return m_confidence_threshold;
}

void ObjectDetector::setScale(float scale)
{
    m_scale = scale;
}

float ObjectDetector::scale() const
{
    return m_scale;
}

void ObjectDetector::setMean(cv::Scalar mean)
{
    m_mean = mean;
}

cv::Scalar ObjectDetector::mean() const
{
    return m_mean;
}

void ObjectDetector::setSwapRB(bool swap)
{
    m_swap_rb = swap;
}

bool ObjectDetector::swapRB() const
{
    return m_swap_rb;
}

void ObjectDetector::setComputingUnit(std::string computing_unit)
{
    if(m_computing_unit == computing_unit)
    {
        return;
    }

    std::cout << "CHANGING COMPUTING UNIT!" << std::endl;

    if(computing_unit == "CPU") {
        m_cv_net->net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    } else if(computing_unit == "OPENCL") {
        m_cv_net->net.setPreferableTarget(cv::dnn::DNN_TARGET_OPENCL);
    } else if(computing_unit == "OPENCL_FP16") {
        m_cv_net->net.setPreferableTarget(cv::dnn::DNN_TARGET_OPENCL_FP16);
    } else if(computing_unit == "MYRIAD") {
        m_cv_net->net.setPreferableTarget(cv::dnn::DNN_TARGET_MYRIAD);
    } else {
        std::stringstream ss;
        ss << "Computing unit not implemented: " << computing_unit << std::endl;
        throw std::runtime_error(ss.str());
    } 
    

    m_computing_unit = computing_unit;
    // else if(computing_unit == "FPGA") {
    //     // m_cv_net->net.setPreferableTarget(cv::dnn::DNN_TARGET_FPGA);
    // }
}

std::string ObjectDetector::computingUnit() const
{
    return m_computing_unit;
}

cv::Size ObjectDetector::inputSize() const
{
    return m_input_size;
}

// PROTECTED
void ObjectDetector::initNetwork()
{
    m_cv_net = buildNet();
    if(m_cv_net && !m_cv_net->net.empty())
    {
        std::vector<int> outLayersIds = m_cv_net->net.getUnconnectedOutLayers();
        m_output_layer_names = m_cv_net->net.getUnconnectedOutLayersNames();

        for(int outLayerId : outLayersIds)
        {
            auto outLayer = m_cv_net->net.getLayer(outLayerId);
            m_output_layer_types.push_back(outLayer->type);
            std::cout << "Output layer found: " << outLayer->type << std::endl;
        }
    } else {
        throw std::runtime_error("Could not create Neural Network");
    }
}

std::vector<std::string> ObjectDetector::readClasses()
{
    std::vector<std::string> ret;
    fs::path model_dir(m_model_dir);
    model_dir /= m_model_name;
    model_dir /= "classes.txt";
    std::string filename = model_dir.string();

    std::ifstream infile(filename);

    for( std::string class_name; std::getline( infile, class_name ); )
    {
        ret.push_back(class_name);
    }

    infile.close();

    return ret;
}

std::shared_ptr<CvNeuralNetwork> ObjectDetector::buildNet()
{
    std::shared_ptr<CvNeuralNetwork> ret;
    fs::path model_dir(m_model_dir);
    model_dir /= m_model_name;
    
    fs::recursive_directory_iterator it(model_dir);
    fs::recursive_directory_iterator endit;
    bool flag_file_found = false;
    while(it != endit && !flag_file_found)
    {
        const std::string filename = it->path().filename().string();
        if(filename == "DARKNET")
        {
            fs::path weights_file = model_dir / "deploy.weights";
            fs::path config_file = model_dir / "deploy.cfg";
            ret.reset(new CvNeuralNetwork(
                cv::dnn::readNetFromDarknet(config_file.string(), weights_file.string())
            ));
            flag_file_found = true;
        } else if(filename == "CAFFE") {
            fs::path weights_file = model_dir / "deploy.caffemodel";
            fs::path config_file = model_dir / "deploy.prototxt";
            ret.reset(new CvNeuralNetwork(
                cv::dnn::readNetFromCaffe(config_file.string(), weights_file.string())
            ));
            flag_file_found = true;
        } else if(filename == "TENSORFLOW") {
            fs::path weights_file = model_dir / "deploy.pb";
            fs::path config_file = model_dir / "deploy.pbtxt";

            ret.reset(new CvNeuralNetwork(
                cv::dnn::readNetFromTensorflow(weights_file.string(), config_file.string())
            ));
            flag_file_found = true;
        }
        ++it;
    }
    if(!flag_file_found)
    {
        std::stringstream ss;
        ss << "Could not find flag file in " << m_model_name;
        throw std::runtime_error(ss.str());
    }
    
    return ret;
}

void ObjectDetector::preprocess(const cv::Mat& frame)
{
    cv::Size inpImgSize = m_input_size;

    if (inpImgSize.width <= 0) { 
        inpImgSize.width = frame.cols;
    }
    if (inpImgSize.height <= 0) {
        inpImgSize.height = frame.rows;
    }

    // std::cout << "preprocess stats:" << std::endl;
    // std::cout << frame.size() << std::endl;
    // std::cout << inpImgSize << std::endl;
    // std::cout << m_swap_rb << std::endl;
    // std::cout << std::endl;

    // Create a 4D blob from a frame.
    cv::Mat blob;
    cv::dnn::blobFromImage(frame, blob, 1.0, inpImgSize, cv::Scalar(), m_swap_rb, false, CV_8U);

    // Run a model.
    m_cv_net->net.setInput(blob, "", scale(), mean());

    if (m_cv_net->net.getLayer(0)->outputNameToIndex("im_info") != -1)  // Faster-RCNN or R-FCN
    {
        cv::Mat imInfo = (cv::Mat_<float>(1, 3) << inpImgSize.height, inpImgSize.width, 1.6f);
        m_cv_net->net.setInput(imInfo, "im_info");
    }
}

void ObjectDetector::fillLabelNames(
    std::vector<CvObject>& labels)
{
    for(int i=0; i<labels.size(); i++)
    {
        if(labels[i].id < m_classes.size())
        {
            labels[i].name = m_classes[labels[i].id];
        } else {
            labels[i].name.clear();
        }
    }
}

std::vector<CvObject> ObjectDetector::filterLabels(
    const std::vector<CvObject>& labels)
{
    std::vector<CvObject> ret;
    for(const CvObject& label : labels)
    {
        if(label.confidence > m_confidence_threshold)
        {
            ret.push_back(label);
        }
    }
    return ret;
}


std::vector<CvObject> ObjectDetector::generateLabels(const std::vector<cv::Mat>& outs)
{
    std::vector<CvObject> ret;

    for(int i=0; i<m_output_layer_types.size(); i++)
    {
        const std::string outLayerType = m_output_layer_types[i];
        if (outLayerType == "DetectionOutput")
        {
            float* data = (float*)outs[i].data;
            for(int j=0; j<outs[i].total(); j += 7)
            {
                float confidence = data[j + 2];
                float left   = data[j + 3];
                float top    = data[j + 4];
                float right  = data[j + 5];
                float bottom = data[j + 6];
                float width  = right - left;
                float height = bottom - top;
                
                CvObject label;
                // std::cout << confidence << std::endl;
                label.id = (int)(data[j + 1]);
                label.confidence = confidence;
                label.rect.x = left;
                label.rect.y = top;
                label.rect.width = width;
                label.rect.height = height;
                ret.push_back(label);
            }
        } else if (outLayerType == "Region") {
            float* data = (float*)outs[i].data;
            for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
            {
                cv::Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
                cv::Point classIdPoint;
                double confidence;
                cv::minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
                
                float centerX = data[0];
                float centerY = data[1];
                float width = data[2];
                float height = data[3];
                float left = centerX - width / 2.0;
                float top = centerY - height / 2.0;

                CvObject label;
                label.id = classIdPoint.x;
                label.confidence = (float)confidence;
                label.rect.x = left;
                label.rect.y = top;
                label.rect.width = width;
                label.rect.height = height;
                ret.push_back(label);
            }
        } else {
            throw std::runtime_error("outLayerType not implemented");
        }
    }

    return ret;
}

// PRIVATE

} // namespace image_perception
