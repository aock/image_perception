#include "image_object_tracking/ObjectTracker.hpp"


#include <iostream>
#include <opencv2/tracking.hpp>
#include <deque>
#include <unordered_map>
#include <utility>

// Rectangle Error metrics
#include "image_object_tracking/error_metrics/DistanceRectError.hpp"
#include "image_object_tracking/error_metrics/IntersectionRectError.hpp"

namespace image_object_tracking {

using namespace image_perception_msgs;

class CvTracker {
public:
    CvTracker(CvObject object)
    :init_object(object)
    ,state_size(6)
    ,meas_size(4)
    ,contr_size(0)
    ,last_updated(std::chrono::steady_clock::now())
    {
        kf.reset(new cv::KalmanFilter(state_size, meas_size, contr_size, CV_32F));

        cv::Mat state(state_size, 1, CV_32F);  // [x,y,v_x,v_y,w,h]
        cv::Mat meas(meas_size, 1, CV_32F);    // [z_x,z_y,z_w,z_h]

        // Transition State Matrix A
        // Note: set dT at each processing step!
        // [ 1 0 dT 0  0 0 ]
        // [ 0 1 0  dT 0 0 ]
        // [ 0 0 1  0  0 0 ]
        // [ 0 0 0  1  0 0 ]
        // [ 0 0 0  0  1 0 ]
        // [ 0 0 0  0  0 1 ]
        cv::setIdentity(kf->transitionMatrix);

        // Measure Matrix H
        // [ 1 0 0 0 0 0 ]
        // [ 0 1 0 0 0 0 ]
        // [ 0 0 0 0 1 0 ]
        // [ 0 0 0 0 0 1 ]
        kf->measurementMatrix = cv::Mat::zeros(meas_size, state_size, CV_32F);
        kf->measurementMatrix.at<float>(0) = 1.0f;
        kf->measurementMatrix.at<float>(7) = 1.0f;
        kf->measurementMatrix.at<float>(16) = 1.0f;
        kf->measurementMatrix.at<float>(23) = 1.0f;

        // Process Noise Covariance Matrix Q
        // [ Ex   0   0     0     0    0  ]
        // [ 0    Ey  0     0     0    0  ]
        // [ 0    0   Ev_x  0     0    0  ]
        // [ 0    0   0     Ev_y  0    0  ]
        // [ 0    0   0     0     Ew   0  ]
        // [ 0    0   0     0     0    Eh ]
        //cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
        kf->processNoiseCov.at<float>(0) = 1e-2;
        kf->processNoiseCov.at<float>(7) = 1e-2;
        kf->processNoiseCov.at<float>(14) = 1e-2;
        kf->processNoiseCov.at<float>(21) = 1e-2;
        kf->processNoiseCov.at<float>(28) = 1e-2;
        kf->processNoiseCov.at<float>(35) = 1e-2;

        // Measures Noise Covariance Matrix R
        cv::setIdentity(kf->measurementNoiseCov, cv::Scalar(1e-1));
    
        // INIT

        kf->errorCovPre.at<float>(0) = 1; // px
        kf->errorCovPre.at<float>(7) = 1; // px
        kf->errorCovPre.at<float>(14) = 1;
        kf->errorCovPre.at<float>(21) = 1;
        kf->errorCovPre.at<float>(28) = 1; // px
        kf->errorCovPre.at<float>(35) = 1; // px

        state.at<float>(0) = object.rect.x + object.rect.width / 2.0;
        state.at<float>(1) = object.rect.y + object.rect.height / 2.0;
        state.at<float>(2) = 0.0;
        state.at<float>(3) = 0.0;
        state.at<float>(4) = object.rect.width;
        state.at<float>(5) = object.rect.height;
        // // <<<< Initialization

        kf->statePost = state;
        // std::cout << "initial state: " << kf->statePost << std::endl;
    }

    CvObject predict(double dt)
    {
        CvObject pred = init_object;
        kf->transitionMatrix.at<float>(2) = dt;
        kf->transitionMatrix.at<float>(9) = dt;
        cv::Mat state = kf->predict();
        pred.rect.width = state.at<float>(4);
        pred.rect.height = state.at<float>(5);
        pred.rect.x = state.at<float>(0) - pred.rect.width / 2.0;
        pred.rect.y = state.at<float>(1) - pred.rect.height / 2.0;
        
        return pred;
    }

    void correct(const CvObject& rect)
    {
        cv::Mat meas(4, 1, CV_32F);
        meas.at<float>(0) = rect.rect.x + rect.rect.width / 2.0;
        meas.at<float>(1) = rect.rect.y + rect.rect.height / 2.0;
        meas.at<float>(2) = rect.rect.width;
        meas.at<float>(3) = rect.rect.height;
        kf->correct(meas);
        // hard correct. 
        // TODO: correct with respect to confidence
        // get more confident if the label was always the same
        init_object.id = rect.id;
        init_object.name = rect.name;
        init_object.confidence = rect.confidence;
    }

    CvObject get() const {
        CvObject ret = init_object;

        ret.rect.width = kf->statePost.at<float>(4);
        ret.rect.height = kf->statePost.at<float>(5);
        ret.rect.x = kf->statePost.at<float>(0) - ret.rect.width / 2.0;
        ret.rect.y = kf->statePost.at<float>(1) - ret.rect.height / 2.0;
        
        return ret;
    }

    int id() const {
        return init_object.id;
    }

    cv::Ptr<cv::KalmanFilter> kf;
    int state_size;
    int meas_size;
    int contr_size;

    std::chrono::time_point<std::chrono::steady_clock> last_updated;
private:
    CvObject init_object;
};

ImageObjectTracker::ImageObjectTracker()
:m_last_updated(std::chrono::steady_clock::now())
,m_error_thresh(0.5)
,m_remove_time(2.0)
{
    setErrorMetric("distance");
}

void ImageObjectTracker::setErrorMetric(std::string error_metric)
{
    if(error_metric == "distance")
    {
        m_error_metric.reset(new DistanceRectError());
    } else if(error_metric == "intersection") {
        m_error_metric.reset(new IntersectionRectError());
    } else {
        throw std::runtime_error("[ImageObjectTracker]: error_metric not implemented");
    }
}

void ImageObjectTracker::update(const std::vector<CvObject>& objects)
{
    auto current_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> diff = current_time-m_last_updated;
    update(objects, diff.count());
}

void ImageObjectTracker::update(const std::vector<CvObject>& objects, double dt)
{
    // std::cout << "update tracked objects, dt: " << dt << std::endl;
    m_last_updated = std::chrono::steady_clock::now();

    std::list<CvObject> observations;
    observations.assign(objects.begin(), objects.end());
    
    // 1) Prediction: id -> CvObject
    std::unordered_map<unsigned int, CvObject> predictions;
    for(auto tracker_it : m_trackers)
    {
        predictions.insert({
            tracker_it.first, tracker_it.second->predict(dt)
        });
    }
    // std::cout << "1) Predicted: " << m_trackers.size() << std::endl;

    // 2) Matching
    unsigned int num_matches = 0;
    for(auto pred : predictions)
    {
        // search for objects near rects 
        float best_error = std::numeric_limits<float>::max();
        std::list<CvObject>::iterator best_it = observations.end();

        // find nearest match -> error
        for(std::list<CvObject>::iterator it = observations.begin();
            it != observations.end(); ++it)
        {
            float error = objectError(pred.second, *it);
            if(error < best_error && it->id == pred.second.id)
            {
                best_error = error;
                best_it = it;
            }
        }
        
        
        // if match was found and error is smaller threshold
        if(best_it != observations.end() && best_error < m_error_thresh)
        {
            // std::cout << "error: " << best_error << std::endl;
            // update measurement
            auto& track_it = m_trackers[pred.first];
            track_it->correct(*best_it);
            track_it->last_updated = m_last_updated;
            // track_it.second = m_last_upm_last_updateddated;
            // remove observation
            observations.erase(best_it);
            num_matches++;
        }
    }
    // std::cout << "2) Matched: " << num_matches << "/" << objects.size() << " observations" << std::endl;

    // 3) remove old tracker
    unsigned int num_removed = 0;
    auto it = m_trackers.begin();
    
    // for(auto it = m_trackers.begin(); it != m_trackers.end(); ++it)
    while(it != m_trackers.end())
    {
        std::chrono::duration<double> diff(m_last_updated - it->second->last_updated);
        if(diff.count() > m_remove_time)
        {
            // std::cout << "try to erase" << std::endl;
            it = m_trackers.erase(it);
            // std::cout << "erased" << std::endl;
            num_removed++;
        } else {
            ++it;
        }
    }

    // std::cout << "3) Removed Tracker: " << num_removed << std::endl;
    
    // 4) add new tracker from remaining observations
    for(CvObject observation : observations)
    {
        // std::cout << "insert " << observation.id << std::endl;
        // std::cout << observation.rect.x
        //     << " " << observation.rect.y
        //     << " " << observation.rect.width
        //     << " " << observation.rect.height << std::endl;
        unsigned int random_int = rand();

        std::shared_ptr<CvTracker> tracker(new CvTracker(observation));
        tracker->last_updated = m_last_updated;

        m_trackers.insert({
            random_int, tracker
        });
    }
    // std::cout << "4) Added Tracker: " << observations.size() << std::endl;
    
    // std::cout << "Object Tracker updated." << std::endl;
}

std::unordered_map<unsigned int, CvObject> ImageObjectTracker::get() const
{
    std::unordered_map<unsigned int, CvObject> ret;

    for(auto entry : m_trackers)
    {
        ret.insert({entry.first, entry.second->get()});
    }

    return ret;
}

/// PRIVATE ////

inline float ImageObjectTracker::objectError(
    const CvObject& a, const CvObject& b) const
{
    return m_error_metric->e(a.rect, b.rect);
}

} // namespace image_object_tracking