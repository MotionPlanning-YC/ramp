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
#include "tf/transform_datatypes.h"

struct ModifiedTrajectory {
  RampTrajectory trajec_;
  std::vector<float> velocities_;
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
    

    //Hold the population of trajectories, 
    //the velocities of each trajectory's segments,
    //the trajectory's path,
    //and the resolution rate for the trajectories
    Population population_;
    std::vector<std::vector<float> >    velocities_;
    std::vector<Path>                   paths_;
    const unsigned int resolutionRate_;
    
    //Hold the start and goal configurations
    //and the ranges for each DOF
    Configuration start_;
    Configuration goal_;
    std::vector<Range> ranges_;
    
    
    //The best trajectory
    RampTrajectory bestTrajec_;


    //Timer for sending the best trajec
    ros::Timer timer_;
    
   
    //Control cycle - used for determining when to update P(t)
    ros::Duration controlCycle_;
    
    /********************************************
     ***************** Methods ******************
     ********************************************/
    
    //Start planning
    void go();
    
    //Initialization steps
    void init_population();
    void init(const ros::NodeHandle& h);
    
    //Send the best trajectory to the control package
    void sendBest();

    //Evaluate the population 
    void evaluateTrajectory(RampTrajectory& trajec, const std::vector<unsigned int> i_segments);
    void evaluatePopulation();
    const RampTrajectory evaluateAndObtainBest();
    
    //Modify trajectory or path
    const std::vector<Path> modifyPath();
    const std::vector<ModifiedTrajectory> modifyTrajec();

    //Request information from other packages
    //Cannot make the request srvs const because they have no serialize/deserialize
    const bool requestTrajectory(ramp_msgs::TrajectoryRequest& tr);
    const bool requestEvaluation(ramp_msgs::EvaluationRequest& er);

    //Msg building methods
    const ramp_msgs::TrajectoryRequest buildTrajectoryRequest(const unsigned int i_path, const std::vector<float> v_s, const std::vector<float> v_e) const;
    const ramp_msgs::TrajectoryRequest buildTrajectoryRequest(const Path path, const std::vector<float> v_s, const std::vector<float> v_e) const;
    const ramp_msgs::EvaluationRequest buildEvaluationRequest(const unsigned int i_path, const std::vector<unsigned int> i_segments);
    const ramp_msgs::EvaluationRequest buildEvaluationRequest(const RampTrajectory trajec, const std::vector<unsigned int> i_segments);

    //Get the starting configuration
    Configuration getStartConfiguration();

    //Update the population 
    void updatePopulation(ros::Duration d);

    //Callback for updating the robot's current configuration 
    //This method changes the start_ member
    void updateCallback(const ramp_msgs::Update::ConstPtr& msg);

    void controlCycleCallback(const ros::TimerEvent& t);
    
    
  
  private:
    /** These are (mostly) utility members that are only used by Planner and should not be used by other classes*/


    /** Methods */

    //This gets the new velocities for path segments after a path has been updated
    const std::vector< std::vector<float> > getNewVelocities(std::vector<Path> new_path, std::vector<int> i_old);

    //Modification procedure
    void modification();
    
    //Updates the paths in P(t) so we can get new trajectories
    void updatePaths(Configuration start, ros::Duration dur);

    /** Data members */

    //Utility instance
    Utility u; 
    
    //Mutex for start_ member
    bool mutex_start_;
    bool mutex_pop_;

    //Size of population
    const int populationSize_;

    //Generation counter
    unsigned int generation_;

    //Last time P(t) was updated
    ros::Time lastUpdate_;
    
    //Handlers to communicate with other packages
    TrajectoryRequestHandler*   h_traj_req_;
    ControlHandler*             h_control_;
    EvaluationRequestHandler*   h_eval_req_;

    //Modifier_ communicates with the path_modification package
    Modifier* modifier_;
};

#endif
