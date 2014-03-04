#ifndef EVALUATE_H
#define EVALUATE_H
#include "ramp_msgs/EvaluationRequest.h"
#include "euclidean_distance.h"
#include "time.h"
#include "collision_detection.h"
#include "utility.h"
#include "corobot_msgs/SensorMsg.h"
#include "ramp_msgs/ObstacleList.h"
#include "collision_detection.h"




class Evaluate {
  public:
    Evaluate();
    Evaluate(const ramp_msgs::EvaluationRequest::Request& req);
    
    void setRequest(const ramp_msgs::EvaluationRequest::Request& req);

    const double performFitness(CollisionDetection::QueryResult feasible);

    /** Different evaluation criteria */
    EuclideanDistance euc_dist_;
    Time time_;


    /** Collision detection */
    //CollisionDetection collision_;
    //ramp_msgs::ObstacleList obstacleList_;
    //const bool performCollisionDetection();
    

    //Information sent by the request
    ramp_msgs::Trajectory trajectory_;
    std::vector<unsigned int> i_segments_;

    float Q;

};

#endif
