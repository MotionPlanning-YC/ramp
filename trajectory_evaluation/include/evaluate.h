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
    
    void setRequest(const ramp_msgs::EvaluationRequest& req);

    void perform(ramp_msgs::EvaluationRequest& req, ramp_msgs::EvaluationResponse& res);
    void performFeasibility(ramp_msgs::EvaluationRequest& er);
    void performFitness(ramp_msgs::RampTrajectory& trj, double& result);

    /** Different evaluation criteria */
    EuclideanDistance eucDist_;
    Orientation orientation_;

    ramp_msgs::EvaluationResponse res_;
    
    CollisionDetection cd_;
    CollisionDetection::QueryResult qr_;

    //Information sent by the request
    ramp_msgs::RampTrajectory trajectory_;
    std::vector<ramp_msgs::RampTrajectory> ob_trjs_;

    float Q;

  private:
    Utility utility_;
    bool orientation_infeasible_;
};

#endif
