#ifndef EVALUATE_H
#define EVALUATE_H
#include "ramp_msgs/EvaluationRequest.h"
#include "euclidean_distance.h"
#include "time.h"
#include "utility.h"


class Evaluate {
  public:
    Evaluate(const ramp_msgs::EvaluationRequest::Request& req);
    

    const double perform();

    EuclideanDistance euc_dist_;
    Time time_;

    //Information sent by the request
    ramp_msgs::Trajectory trajectory_;
    std::vector<unsigned int> i_segments_;
    
};

#endif
