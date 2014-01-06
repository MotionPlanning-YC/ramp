#ifndef COLLISION_DETECTION
#define COLLISION_DETECTION
#include "utility.h"
#include "ramp_msgs/ObstacleList.h"
#include "ramp_msgs/Trajectory.h"
#include "motion_type.h"
#include "tf/transform_datatypes.h"
#include "ramp_msgs/TrajectoryRequest.h"


class CollisionDetection {
  public:
    CollisionDetection() {}

    const bool perform() const;
    const ramp_msgs::Trajectory getPredictedTrajectory(const ramp_msgs::Obstacle, const ros::Duration) const;
    const bool query(const ramp_msgs::Trajectory ob_trajectory) const;

    ramp_msgs::Trajectory trajectory_;
    ramp_msgs::ObstacleList obstacleList_;

  private:
    const MotionType findMotionType(const ramp_msgs::Obstacle) const;
    const bool onSegment(const tf::Point p_i, const tf::Point p_j, const tf::Point p_k) const;
    Utility u;
};

#endif
