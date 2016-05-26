#ifndef COLLISION_DETECTION
#define COLLISION_DETECTION
#include "utility.h"
#include "ramp_msgs/Population.h"
#include "motion_type.h"
#include "tf/transform_datatypes.h"
#include "ramp_msgs/TrajectoryRequest.h"
#include "ramp_msgs/Obstacle.h"
#include <chrono>




class CollisionDetection 
{
  public:
  
    /* Struct to hold information about a query */
    struct QueryResult 
    {
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
    void                        perform(const ramp_msgs::RampTrajectory& trajectory, const std::vector<ramp_msgs::RampTrajectory>& obstacle_trjs, QueryResult& result); 
    

    const QueryResult           query(const ramp_msgs::RampTrajectory& trajectory, const ramp_msgs::RampTrajectory& ob_trajectory) const;


    std::vector< std::vector<ramp_msgs::MotionState> > buildTree(const std::vector<ramp_msgs::MotionState> 
        control_poly, const int depth) const;
    std::vector< std::vector<ramp_msgs::MotionState> > deCasteljau(const std::vector<ramp_msgs::MotionState> 
        control_poly) const;
    bool ControlPolyArc(const std::vector<ramp_msgs::MotionState> con_poly_vert, const std::vector<double> cir_cent, 
        const double cir_r, const ramp_msgs::RampTrajectory& ob_tr) const;

    //void LineLine(const ramp_msgs::RampTrajectory& trajectory, const ramp_msgs::RampTrajectory& ob_trajectory, QueryResult& qr) const;
    void LineLine
      (const ramp_msgs::RampTrajectory& trajectory, const int& segment, const ramp_msgs::RampTrajectory& ob_trajectory, QueryResult& qr) const;

    void LineArc(const ramp_msgs::RampTrajectory& trajectory, const int& segment, const ramp_msgs::RampTrajectory& ob_trajectory, QueryResult& qr) const;
    
    void BezierLine(const std::vector<ramp_msgs::MotionState>& control_points, const ramp_msgs::RampTrajectory& ob_trajectory, QueryResult& qr) const;
    
    void BezierArc(const std::vector<ramp_msgs::MotionState>& control_points, const ramp_msgs::RampTrajectory& ob_trajectory, QueryResult& qr) const;

    /***** Data Members ****/
    //ramp_msgs::RampTrajectory trajectory_;
    //std::vector<ramp_msgs::RampTrajectory> obstacle_trjs_;
  

    ros::Time t_for;
    ros::Duration d_for;
    ros::Time t_inner_for;
    ros::Duration d_inner_for;

    ros::Publisher pub_population;

  private:

    /***** Methods *****/

    /***** Data Members *****/
    Utility                   utility_;
};

#endif
