#ifndef EVALUATE_H
#define EVALUATE_H
#include "ramp_msgs/EvaluationSrv.h"
#include "euclidean_distance.h"
#include "orientation.h"
#include "collision_detection.h"
#include "utility.h"




class Evaluate {
  public:
    Evaluate();
    Evaluate(const ramp_msgs::EvaluationRequest& req);
    
    void setRequest(const ramp_msgs::EvaluationRequest req);

    const ramp_msgs::EvaluationResponse perform();
    bool performFeasibility();
    const double performFitness(bool feasible);

    /** Different evaluation criteria */
    EuclideanDistance eucDist_;
    Orientation orientation_;

    ramp_msgs::EvaluationResponse res_;
    
    CollisionDetection cd_;
    CollisionDetection::QueryResult qr_;

    //Information sent by the request
    ramp_msgs::RampTrajectory trajectory_;
    double currentTheta_;

    float Q;

  private:
    Utility utility_;
    bool orientation_infeasible_;
};

#endif
