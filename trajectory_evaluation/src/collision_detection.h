#ifndef COLLISION_DETECTION
#define COLLISION_DETECTION
#include "utility.h"
#include "ramp_msgs/ObstacleList.h"
#include "ramp_msgs/Population.h"
#include "motion_type.h"
#include "tf/transform_datatypes.h"
#include "ramp_msgs/TrajectoryRequest.h"
#include "trajectory_request_handler.h"




class CollisionDetection {
  public:
  
    /** Struct to hold information about a query */
    struct QueryResult {
      QueryResult() : collision_(false), time_until_collision_(9999.0f), 
                      i_obstacle(-1) {}
      bool  collision_;
      float time_until_collision_;
      int   i_obstacle;
    };  // End QueryResult


    /***** Constructor and Destructor *****/
    CollisionDetection(); 
    ~CollisionDetection();

    /***** Methods *****/ 
    void                        init(ros::NodeHandle& h);
    const QueryResult           perform() const;
    const QueryResult           query(const ramp_msgs::RampTrajectory ob_trajectory) const;
    const ramp_msgs::RampTrajectory getPredictedTrajectory(const ramp_msgs::Obstacle) const;


    /***** Data Members ****/
    int                   id;
    tf::Transform         ob_T_w_b_;
    ramp_msgs::Obstacle   obstacle_;
    ramp_msgs::RampTrajectory trajectory_;
    ros::Duration         predictionTime_;
  


  private:

    /***** Methods *****/
    void                  setOb_T_w_b(int id);
    const MotionType      findMotionType(const ramp_msgs::Obstacle) const;
    const ramp_msgs::Path getObstaclePath(const ramp_msgs::Obstacle ob, const MotionType mt) const;

    /***** Data Members *****/
    TrajectoryRequestHandler* h_traj_req_;
    Utility                   utility;
};

#endif
