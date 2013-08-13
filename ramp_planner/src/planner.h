#ifndef PLANNER_H
#define PLANNER
#include "ros/ros.h"
#include "path.h"
#include "trajectory_request_handler.h"
#include "modification_request_handler.h"

class Planner {
  public:
    Planner();
    Planner(const int p);
    Planner(const unsigned int r);
    Planner(const unsigned int r, const int p);
    ~Planner();

    /** Data Members */
    std::vector<ramp_msgs::Trajectory> population_;
    std::vector<Path> paths_;
    std::vector<std::vector<float> > times_;

    std::vector<Range> ranges_;
    Configuration start_;
    Configuration goal_;
    TrajectoryRequestHandler*   h_traj_req_;
    ModificationRequestHandler* h_mod_req_;
    const unsigned int resolutionRate_;
    
    /** Methods */
    void initialization();
    void init_handlers(const ros::NodeHandle& h);
    
    const ramp_msgs::Path modifyPath(const unsigned int i) const;
    const ramp_msgs::Trajectory modifyTraj(const unsigned int i) const;

    //Cannot make tr const because it has no serialize/deserialize
    const bool requestTrajectory(ramp_msgs::TrajectoryRequest& tr) const;
    
    //Cannot make mr const because it has no serialize/deserialize
    const bool requestModification(ramp_msgs::ModificationRequest& mr) const;

    //msg building methods...
    const ramp_msgs::ModificationRequest buildModificationRequestMsg(const unsigned int i_path) const;
    const ramp_msgs::TrajectoryRequest buildTrajectoryRequestMsg(const unsigned int i_path, const std::vector<float> times) const;
    const ramp_msgs::TrajectoryRequest buildTrajectoryRequestMsg(const Path path, const std::vector<float> times) const;



  private:
    
    const int populationSize_;
    ramp_msgs::Trajectory traj_;
};

#endif