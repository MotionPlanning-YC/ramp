#ifndef PLANNER_H
#define PLANNER_H
#include "ros/ros.h"
#include "path.h"
#include "ramp_trajectory.h"
#include "evaluation_request_handler.h"
#include "trajectory_request_handler.h"
#include "ramp_msgs/Update.h"
#include "nav_msgs/Odometry.h"
#include "modifier.h"
#include "population.h"
#include "control_handler.h"
#include "parameter_handler.h"

struct ModifiedTrajectory {
  RampTrajectory trajec_;
};

class Planner {
  public:
    Planner();
    Planner(const ros::NodeHandle& h);
    Planner(const unsigned int r, const int p);
    ~Planner();


    /*******************************************
     ************** Data Members ***************
     *******************************************/
    

    // Hold the population of trajectories, 
    // the velocities of each trajectory's segments,
    // the trajectory's path,
    // and the resolution rate for the trajectories
    Population                          population_;
    std::vector<Path>                   paths_;
    const unsigned int                  resolutionRate_;
    
    // Hold the start and goal configurations
    // and the ranges for each DOF
    Configuration start_;
    Configuration goal_;
    std::vector<Range> ranges_;
    
    
    // The most fit trajectory in the population
    RampTrajectory bestTrajec_;


    /* Members for cycles */

    // Timer for sending the best trajec
    // Control cycle - used for determining when to update P(t)
    ros::Timer    controlCycleTimer_;
    ros::Duration controlCycle_;
    
    // Timer for doing a modification
    // Planning cycle - used for determining when to update P(t)
    ros::Timer    planningCycleTimer_;
    ros::Duration planningCycle_;
    
    // Sensing cycle
    ros::Duration sensingCycle_;

    // Cycle to check imminent collision
    ros::Timer imminentCollisionTimer_;
    ros::Duration imminentCollisionCycle_;

    /**/


    // Transformation of the initial pose of the robot 
    // We use this to transform the odometry updates into world CS
    tf::Transform T_od_w_;
   
    
    // Robot ID
    unsigned int id_;


    
    
    /********************************************
     ***************** Methods ******************
     ********************************************/


    
    // Start planning
    void go();
    
    // Initialization steps
    void init_population();
    void init(const ros::NodeHandle& h);
    
    // Send the best trajectory to the control package
    void sendBest();
    
    //  Send the whole population to the trajectory viewer
    void sendPopulation();

    // Evaluate the population 
    void                  evaluateTrajectory(RampTrajectory& trajec);
    void                  evaluatePopulation();
    const RampTrajectory  evaluateAndObtainBest();
    
    // Modify trajectory or path
    const std::vector<Path> modifyPath();
    const std::vector<ModifiedTrajectory> modifyTrajec();

    // Request information from other packages
    // Cannot make the request srvs const because they have no serialize/deserialize
    bool requestTrajectory(ramp_msgs::TrajectoryRequest& tr);
    bool requestEvaluation(ramp_msgs::EvaluationRequest& er);

    // Get the starting configuration
    Configuration getStartConfiguration();

    // Update the population 
    void updatePopulation(ros::Duration d);

    // Display all of the paths
    const std::string pathsToString() const;

    // Set the transformation from odometry to world CS
    void setT_od_w(std::vector<float> od_info);

    // Callback for receiving updates from the ramp_control
    void updateCallback(const ramp_msgs::Update& msg);


  
  private:
    /** These are (mostly) utility members that are only used by Planner and should not be used by other classes*/


    /***** Methods *****/

    // This gets the new velocities for path segments after a path has been updated
    const std::vector< std::vector<float> > getNewVelocities(std::vector<Path> new_path, std::vector<int> i_old);
    
    // Updates the paths in P(t) so we can get new trajectories
    void updatePaths(Configuration start, ros::Duration dur);

    // Returns an id for a RampTrajectory 
    unsigned int getIRT();

    // Adjust the trajectory so that the robot does not
    // completely stop to change to it
    void gradualTrajectory(RampTrajectory& t);

    // Modification procedure
    void modification();

    // Callback methods for ros::Timers
    void controlCycleCallback(const ros::TimerEvent& t);
    void planningCycleCallback(const ros::TimerEvent& t);
    void imminentCollisionCallback(const ros::TimerEvent& t);

    // Msg building methods
    //const ramp_msgs::TrajectoryRequest buildTrajectoryRequest(const unsigned int i_path, const std::vector<float> v_s, const std::vector<float> v_e) const;
    //const ramp_msgs::TrajectoryRequest buildTrajectoryRequest(const Path path, const std::vector<float> v_s, const std::vector<float> v_e) const;
    const ramp_msgs::TrajectoryRequest buildTrajectoryRequest(
              const unsigned int i_path ) const;
    const ramp_msgs::TrajectoryRequest buildTrajectoryRequest(
              const Path path ) const;
    const ramp_msgs::EvaluationRequest buildEvaluationRequest(const unsigned int i_path);
    const ramp_msgs::EvaluationRequest buildEvaluationRequest(const RampTrajectory trajec);



    /***** Data members *****/

    // Utility instance
    Utility utility; 

    // Size of population
    const unsigned int populationSize_;

    // Generation counter
    unsigned int generation_;
    
    // Mutex for start_ and population
    bool mutex_start_;
    bool mutex_pop_;

    // ID counter for trajectories
    unsigned int i_rt;

    // Last time P(t) was updated
    ros::Time lastUpdate_;

    // How far we need to get to the goal before stopping
    float goalThreshold_;

    // Number of modification operators 
    unsigned int num_ops_;

    // Distance threshold for imminent collision
    float D_;
    
    // Handlers to communicate with other packages
    TrajectoryRequestHandler*   h_traj_req_;
    EvaluationRequestHandler*   h_eval_req_;
    ControlHandler*             h_control_;
    Modifier*                   modifier_;

    // Parameter handler
    ParameterHandler p_handler_;
};

#endif
