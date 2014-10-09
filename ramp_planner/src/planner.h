#ifndef PLANNER_H
#define PLANNER_H

#include "ros/ros.h"
#include "path.h"
#include "ramp_trajectory.h"
#include "evaluation_request_handler.h"
#include "trajectory_request_handler.h"
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
    tf::Transform T_w_odom_;
   
    
    // Robot ID
    int id_;


    
    
    /********************************************
     ***************** Methods ******************
     ********************************************/


    
    // Start planning
    void go();
    
    // Initialization 
    void initPopulation();
    void init(const uint8_t i, const ros::NodeHandle& h, 
              const MotionState s, const MotionState g, 
              const std::vector<Range> r, const int population_size, 
              const bool sub_populations);
    
    // Send the best trajectory to the control package
    void sendBest();
    
    // Send the whole population to the trajectory viewer
    void sendPopulation();
    void displayTrajectory(const ramp_msgs::RampTrajectory traj) const;

    // Evaluate the population 
    const RampTrajectory  evaluateTrajectory(RampTrajectory trajec);
    void                  evaluatePopulation();
    const RampTrajectory  evaluateAndObtainBest();
    
    // Modify trajectory or path
    const std::vector<Path> modifyPath();
    const std::vector<RampTrajectory> modifyTrajec();

    // Request information from other packages
    // Cannot make the request srvs const because they have no serialize/deserialize
    const RampTrajectory requestTrajectory(ramp_msgs::TrajectoryRequest& tr, const int id=-1);
    const RampTrajectory requestTrajectory(const Path p, const int id=-1);
    const RampTrajectory requestEvaluation(ramp_msgs::EvaluationRequest& er);
    const RampTrajectory requestEvaluation(const RampTrajectory traj);


    // Update the population 
    void adaptPopulation(ros::Duration d);

    // Display all of the paths
    const std::string pathsToString() const;

    // Set the transformation from odometry to world CS
    void setT_base_w(std::vector<double> base_pos);

    // Callback for receiving updates from the ramp_control
    void updateCallback(const ramp_msgs::MotionState& msg);

    // Sets the m_i vector
    void setMi();

    // Motion state that should be reached by next control cycle
    MotionState m_cc_;

    // Each element is the target motion state 
    // for each of i planning cycles
    std::vector<MotionState> m_i;
    



    // Hold the difference between previous startPlanning 
    // and latestUpdate for each control cycle
    std::vector<MotionState> SP_LU_diffs_;
    const MotionState findAverageDiff();
  

    // Debugging variables
    bool modifications_;
    bool evaluations_;
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
              const Path path, const std::vector<ramp_msgs::BezierInfo> curves,
              const int id=0);
    const ramp_msgs::TrajectoryRequest buildTrajectoryRequest(
              const Path path, const int id=0);

    const ramp_msgs::EvaluationRequest buildEvaluationRequest(
              const RampTrajectory trajec)      ;

    // Misc
    const bool checkOrientation()                           const ; 
    const void randomizeMSPositions(MotionState& ms)        const ;
          void checkTrajChange()                                  ;
          void seedPopulation()                                   ;
          void seedPopulationLine()                               ;

    const RampTrajectory  getTransitionTrajectory(const RampTrajectory trgt_traj)     ;
    const RampTrajectory  getTrajectoryWithCurve(const RampTrajectory trgt_traj);
    const MotionState     predictStartPlanning() const  ;


    const std::vector<RampTrajectory> getTrajectories(const std::vector<Path> p);
    const std::vector<RampTrajectory> getTrajectories(std::vector<ramp_msgs::TrajectoryRequest> tr);
    void updatePathsStart(const MotionState s);

    const bool compareSwitchToBest(const RampTrajectory traj) const;

    void stopForDebugging();
    void restartAfterDebugging();

    const bool estimateIfOnCurve() const;

    /***** Data members *****/

    // Utility instance
    Utility             utility_; 

    // Size of population
    unsigned int  populationSize_;

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

    // Number of generations to wait before starting control cycles
    unsigned int        generationsBeforeCC_;

    // Maximum number of generations to occur between control cycles
    unsigned int        generationsPerCC_;

    // Flag for if the control cycles have started
    bool                cc_started_;

    // Flag for if sub-populations are being used
    bool                subPopulations_;
    
    // Number of planning cycles since last control cycle
    int                 c_pc_;

    // Threshold for getting transition trajectory
    double              transThreshold_;

    // Handlers to communicate with other packages
    TrajectoryRequestHandler*   h_traj_req_;
    EvaluationRequestHandler*   h_eval_req_;
    ControlHandler*             h_control_;
    Modifier*                   modifier_;

    // Parameter handler
    ParameterHandler            h_parameters_;



    // Stop things for debugging
    bool stop_;

    uint16_t num_controlCycles_;
};

#endif
