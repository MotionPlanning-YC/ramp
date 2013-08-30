#ifndef PLANNER_H
#define PLANNER_H
#include "ros/ros.h"
#include "path.h"
#include "ramp_trajectory.h"
#include "evaluation_request_handler.h"
#include "trajectory_request_handler.h"
#include "update_request_handler.h"
#include "modifier.h"
#include "population.h"
#include "control_handler.h"

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
    
    
    
    //modifier_ should be private...
    Modifier* modifier_;
    
    /********************************************
     ***************** Methods ******************
     ********************************************/
    
    //Start planning
    void go();
    
    //Initialization steps
    void init_population();
    void init_handlers(const ros::NodeHandle& h);
    
    //Send the best trajectory to the control package
    void sendBest();

    //Evaluate the population 
    void evaluatePopulation();
    const RampTrajectory evaluateAndObtainBest();
    
    //Modify trajectory or path
    const std::vector<Path> modifyPath();
    const std::vector<RampTrajectory> modifyTrajec();

    //Request information from other packages
    //Cannot make the request srvs const because they have no serialize/deserialize
    const bool requestTrajectory(ramp_msgs::TrajectoryRequest& tr);
    const bool requestEvaluation(ramp_msgs::EvaluationRequest& er);

    //Msg building methods
    const ramp_msgs::TrajectoryRequest buildTrajectoryRequest(const unsigned int i_path, const std::vector<float> v_s, const std::vector<float> v_e) const;
    const ramp_msgs::TrajectoryRequest buildTrajectoryRequest(const Path path, const std::vector<float> v_s, const std::vector<float> v_e) const;
    const ramp_msgs::EvaluationRequest buildEvaluationRequest(const unsigned int i_path, const std::vector<unsigned int> i_segments);

    //Get the starting configuration
    Configuration getStartConfiguration();
    void updatePopulation(ros::Duration d);

    //Callback for 
    void updateCallback(const ramp_msgs::Configuration::ConstPtr& msg);

    
    


  
    void updatePaths(Configuration start, ros::Duration dur);
  private:
    
    const std::vector< std::vector<float> > getNewVelocities(std::vector<Path> new_path, std::vector<int> i_old);

    //Modification procedure
    void modification();

    bool mutex_start_;
    Utility u; 
    const int populationSize_;
    unsigned int generation_;

    //Hold the handlers to communicate with other packages
    TrajectoryRequestHandler*   h_traj_req_;
    ControlHandler*             h_control_;
    EvaluationRequestHandler*   h_eval_req_;
    UpdateRequestHandler*       h_update_req_;

};

#endif
