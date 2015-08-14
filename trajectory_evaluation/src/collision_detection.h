#ifndef COLLISION_DETECTION
#define COLLISION_DETECTION
#include "utility.h"
#include "ramp_msgs/Population.h"
#include "motion_type.h"
#include "tf/transform_datatypes.h"
#include "ramp_msgs/TrajectoryRequest.h"
#include "ramp_msgs/Obstacle.h"
#include "trajectory_request_handler.h"




class CollisionDetection {
  public:
  
    /** Struct to hold information about a query */
    struct QueryResult {
      QueryResult() : collision_(false), t_firstCollision_(9999.0f), 
                      i_obstacle_(-1) {}
      bool  collision_;
      float t_firstCollision_;
      int   i_obstacle_;
    };  // End QueryResult


    /***** Constructor and Destructor *****/
    CollisionDetection(); 
    ~CollisionDetection();

    /***** Methods *****/ 
    void                        init(ros::NodeHandle& h);
    const QueryResult           perform() const;
    const QueryResult           query(const ramp_msgs::RampTrajectory ob_trajectory) const;


    /***** Data Members ****/
    ramp_msgs::RampTrajectory trajectory_;
    std::vector<ramp_msgs::RampTrajectory> obstacle_trjs_;
  

    ros::Publisher pub_population;

  private:

    /***** Methods *****/

    /***** Data Members *****/
    Utility                   utility_;
};

#endif
