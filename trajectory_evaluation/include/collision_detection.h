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


    void buildTree(const std::vector<ramp_msgs::MotionState>& control_poly, const int& depth, std::vector< std::vector<ramp_msgs::MotionState> >& result) const;
    void deCasteljau(const std::vector<ramp_msgs::MotionState>& control_poly, std::vector< std::vector<ramp_msgs::MotionState> >& result) const;


    void ControlPolyArc(const std::vector<ramp_msgs::MotionState>& con_poly_vert, const ramp_msgs::RampTrajectory& ob_tr, bool& result, std::vector< std::vector<double> >& points_of_collision) const;

    //void LineLine(const ramp_msgs::RampTrajectory& trajectory, const ramp_msgs::RampTrajectory& ob_trajectory, QueryResult& qr) const;
    void LineLine
      (const ramp_msgs::RampTrajectory& trajectory, const int& segment, const ramp_msgs::RampTrajectory& ob_trajectory, QueryResult& qr, std::vector< std::vector<double> >& points_of_collision) const;


    void LineArc(const std::vector<double> l_p1, const std::vector<double> l_p2, const ramp_msgs::RampTrajectory& ob_trajectory, bool& result, std::vector< std::vector<double> >& points_of_collision) const;


    void LineArc(const ramp_msgs::RampTrajectory& trajectory, const int& segment, const ramp_msgs::RampTrajectory& ob_trajectory, QueryResult& result, std::vector< std::vector<double> >& points_of_collision) const;
    
    
    void BezierLine(const std::vector<ramp_msgs::MotionState>& control_points, const ramp_msgs::RampTrajectory& ob_trajectory, QueryResult& qr, std::vector< std::vector<double> >& points_of_collision) const;
    
    void BezierArc(const std::vector<ramp_msgs::MotionState>& control_points, const ramp_msgs::RampTrajectory& ob_trajectory, QueryResult& qr, std::vector< std::vector<double> >& points_of_collision) const;

    void getCircleInfo(const ramp_msgs::RampTrajectory& traj, double& r, double& h, double& k) const;


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
