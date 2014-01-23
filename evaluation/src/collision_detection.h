#ifndef COLLISION_DETECTION
#define COLLISION_DETECTION
#include "utility.h"
#include "ramp_msgs/ObstacleList.h"
#include "ramp_msgs/Trajectory.h"
#include "motion_type.h"
#include "tf/transform_datatypes.h"
#include "ramp_msgs/TrajectoryRequest.h"
#include "trajectory_request_handler.h"




class CollisionDetection {
  public:
  
    struct QueryResult {
      QueryResult() : collision_(false), time_until_collision_(-1), 
                      i_obstacle(-1) {}

      bool  collision_;
      float time_until_collision_;
      int i_obstacle;
    };


    CollisionDetection(); 
    ~CollisionDetection();

    void init(const ros::NodeHandle& h, int id);

    const QueryResult perform() const;
    const ramp_msgs::Trajectory getPredictedTrajectory(const ramp_msgs::Obstacle, const ros::Duration) const;
    const QueryResult query(const ramp_msgs::Trajectory ob_trajectory) const;
    const std::vector<float> getCenter(std::vector<float> p, float orientation) const;

    ramp_msgs::Trajectory trajectory_;
    //ramp_msgs::ObstacleList obstacleList_;
    ramp_msgs::Obstacle obstacle_;

    TrajectoryRequestHandler* h_traj_req_;
    
    int id;
    std::vector< std::vector<float> > T_od_w;
    void setT_od_w(int id);
  
  
  private:
    const MotionType findMotionType(const ramp_msgs::Obstacle) const;
    const bool onSegment(const tf::Point p_i, const tf::Point p_j, const tf::Point p_k) const;
    Utility u;
};

#endif
