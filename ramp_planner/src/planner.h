#ifndef PLANNER_H
#define PLANNER
#include "ros/ros.h"
#include "path.h"
#include "ramp_trajectory.h"
#include "trajectory_request_handler.h"
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
    
    
    
    /********************************************
     ***************** Methods ******************
     ********************************************/
    
    //Initialization steps
    void init_population();
    void init_handlers(const ros::NodeHandle& h);
    
    //Modify trajectory or path
    const std::vector<Path> modifyPath();
    const std::vector<RampTrajectory> modifyTrajec();

    //Request information from other packages
    //Cannot make the request srvs const because they have no serialize/deserialize
    const bool requestTrajectory(ramp_msgs::TrajectoryRequest& tr);

    //Msg building methods
    const ramp_msgs::TrajectoryRequest buildTrajectoryRequest(const unsigned int i_path, const std::vector<float> v_s, const std::vector<float> v_e) const;
    const ramp_msgs::TrajectoryRequest buildTrajectoryRequest(const Path path, const std::vector<float> v_s, const std::vector<float> v_e) const;



    //Start planning
    void go();


    void sendBest();

    //The best trajectory
    RampTrajectory bestTrajec_;

    //modifier_ should be private...
    Modifier* modifier_;
  private:
    const std::vector< std::vector<float> > getNewVelocities(std::vector<Path> new_path, std::vector<int> i_old);

    //Modification procedure
    void modification();


    Utility u; 
    const int populationSize_;
    Configuration current_;
    unsigned int generation_;

    //Hold the handlers to communicate with other packages
    TrajectoryRequestHandler*   h_traj_req_;
    ControlHandler*             h_control_;
};

#endif
