#ifndef PLANNER_H
#define PLANNER
#include "ros/ros.h"
#include "path.h"
#include "ramp_msgs/Trajectory.h"
#include "ramp_msgs/TrajectoryRequest.h"

class Planner {
  public:
    Planner();
    Planner(const int p);
    Planner(const unsigned int r);
    Planner(const unsigned int r, const int p);
    ~Planner();

    //Data Members
    std::vector<Path> paths_;
    std::vector<ramp_msgs::Trajectory> population_;
    std::vector<Range> ranges_;
    Configuration start_;
    Configuration goal_;
    const unsigned int resolutionRate_;
    
    ros::Publisher pub_traj_request;



    //Methods
    void initialization();
    const ramp_msgs::TrajectoryRequest buildTrajectoryRequestMsg(int i_path, std::vector<float> times) const;
    void trajCallback(const ramp_msgs::Trajectory::ConstPtr& msg);

  private:
    const int populationSize_;
    ramp_msgs::Trajectory traj_;
    bool ready_;
    void waitForTraj();
};

#endif
