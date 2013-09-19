#ifndef EVALUATE_H
#define EVALUATE_H
#include "ramp_msgs/EvaluationRequest.h"
#include "euclidean_distance.h"
#include "time.h"
#include "collision_detection.h"
#include "utility.h"
#include "corobot_msgs/SensorMsg.h"




class Evaluate {
  public:
    Evaluate(const ramp_msgs::EvaluationRequest::Request& req);
    

    const double perform(obstacle_struct obstacle);

    /** Different evaluation criteria */
    EuclideanDistance euc_dist_;
    Time time_;

    //Information sent by the request
    ramp_msgs::Trajectory trajectory_;
    std::vector<unsigned int> i_segments_;
    
};

#endif
