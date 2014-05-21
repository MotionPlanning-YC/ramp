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

class Planner {
  public:
    Planner();
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
    const float                         resolutionRate_;
    
    // Hold the start and goal configurations
    // and the ranges for each DOF
    MotionState start_;
    MotionState goal_;
    std::vector<Range> ranges_;

    // Starting motion state for planning cycle
    MotionState startPlanning_;

    MotionState latestUpdate_;
    
    
    // The most fit trajectory in the population
    RampTrajectory bestTrajec_;


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


    // Transformation of the initial pose of the robot 
    // We use this to transform the odometry updates into world CS
    tf::Transform T_base_w_;
   
    
    // Robot ID
    int id_;


    
    
    /********************************************
     ***************** Methods ******************
     ********************************************/


    
    // Start planning
    void go();
    
    // Initialization 
    void initPopulation();
    void init(const ros::NodeHandle& h, const MotionState s, const MotionState g, const std::vector<Range> r);
    
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
    const std::vector<RampTrajectory> modifyTrajec();

    // Request information from other packages
    // Cannot make the request srvs const because they have no serialize/deserialize
    bool requestTrajectory(ramp_msgs::TrajectoryRequest& tr);
    bool requestEvaluation(ramp_msgs::EvaluationRequest& er);

    // Update the population 
    void adaptPopulation(ros::Duration d);

    // Display all of the paths
    const std::string pathsToString() const;

    // Set the transformation from odometry to world CS
    void setT_base_w(std::vector<float> base_pos);

    // Callback for receiving updates from the ramp_control
    void updateCallback(const ramp_msgs::MotionState& msg);


  
  private:
    /** These are (mostly) utility members that are only used by Planner and should not be used by other classes */


    /***** Methods *****/
    
    // Initialize start and goal
    void initStartGoal(const MotionState s, const MotionState g);
    
    // Updates the paths in P(t) so we can get new trajectories
    void adaptPaths(MotionState start, ros::Duration dur);

    // Returns a unique id for a RampTrajectory 
    unsigned int getIRT();

    // Adjust the trajectory so that the robot does not
    // completely stop to change to it
    void gradualTrajectory(RampTrajectory& t);

    // Modification procedure
    void modification();

    // Callback methods for ros::Timers
    void controlCycleCallback     (const ros::TimerEvent& t);
    void planningCycleCallback    (const ros::TimerEvent& t);
    void imminentCollisionCallback(const ros::TimerEvent& t);

    // Msg building methods
    const ramp_msgs::TrajectoryRequest buildTrajectoryRequest(
              const unsigned int i_path ) const ;
    const ramp_msgs::TrajectoryRequest buildTrajectoryRequest(
              const Path path ) const           ;
    const ramp_msgs::EvaluationRequest buildEvaluationRequest(
              const unsigned int i_path)        ;
    const ramp_msgs::EvaluationRequest buildEvaluationRequest(
              const RampTrajectory trajec)      ;

    // Misc
    const bool checkOrientation()                           const ; 
    const void randomizeMSPositions(MotionState& ms)        const ;
          void updateWithModifier(const int index, const Path p)  ;
          void checkTrajChange()                                  ;
          void seedPopulation()                                   ;
    const RampTrajectory getChangingTrajectory() const;



    /***** Data members *****/

    // Utility instance
    Utility             utility_; 

    // Size of population
    const unsigned int  populationSize_;

    // Generation counter
    unsigned int        generation_;

    // ID counter for trajectories
    unsigned int        i_rt;

    // Last time P(t) was updated
    ros::Time           lastUpdate_;

    // How far we need to get to the goal before stopping
    float               goalThreshold_;

    // Number of modification operators 
    unsigned int        num_ops_;

    // Distance threshold for imminent collision
    float               D_;

    // Index of previous best trajectory
    unsigned int        i_best_prev_;


    bool                init_evaluation_;

    // Handlers to communicate with other packages
    TrajectoryRequestHandler*   h_traj_req_;
    EvaluationRequestHandler*   h_eval_req_;
    ControlHandler*             h_control_;
    Modifier*                   modifier_;

    // Parameter handler
    ParameterHandler            h_parameters_;



    bool stop_;
};

#endif
