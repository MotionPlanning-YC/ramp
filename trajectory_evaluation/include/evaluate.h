#ifndef EVALUATE_H
#define EVALUATE_H
#include "ramp_msgs/EvaluationRequest.h"
#include "euclidean_distance.h"
#include "orientation.h"
#include "collision_detection.h"
#include "utility.h"
#include "collision_detection.h"




class Evaluate {
  public:
    Evaluate();
    Evaluate(const ramp_msgs::EvaluationRequest::Request& req);
    
    void setRequest(const ramp_msgs::EvaluationRequest::Request req);

    const double performFitness(CollisionDetection::QueryResult feasible);

    /** Different evaluation criteria */
    EuclideanDistance eucDist_;
    Orientation orientation_;

    

    //Information sent by the request
    ramp_msgs::RampTrajectory trajectory_;
    double currentTheta_;

    float Q;

  private:
    Utility utility_;
};

#endif
