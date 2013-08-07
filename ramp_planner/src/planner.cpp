#include "planner.h"


Planner::Planner() : populationSize_(7) {}

Planner::Planner(const int p) : populationSize_(p) {}

Planner::~Planner() {}

void Planner::trajCallback(const ramp_msgs::Trajectory::ConstPtr& msg) {
  std::cout<<"\nIn trajCallback!";
}

/** This function generates the initial population of trajectories */
void Planner::initialization() { 
  
  //Create n random paths, where n=populationSize
  for(unsigned int i=0;i<populationSize_;i++) {
    
    //Create the path with the start and goal
    Path temp_path(start_, goal_);

    //Each trajectory will have a random number of knot points
    //Put a max of 10 knot points for practicality...
    unsigned int num = rand() % 10;

    //For each knot point to be created...
    for(unsigned int j=0;j<num;j++) {

      //Create a random configuration
      Configuration temp_config;
      temp_config.ranges_ = ranges_;
      temp_config.random();
      
      //Add the random configuration to the path
      temp_path.Add(temp_config); 
    }

    //Add the path to the list of paths
    paths_.push_back(temp_path);
  }

}


const ramp_msgs::TrajectoryRequest Planner::buildTrajectoryRequestMsg(int i_path, std::vector<float> times, int resolution) const {
  ramp_msgs::TrajectoryRequest result;

  result.path = paths_.at(i_path).buildPathMsg();
  result.t    = times;
  result.resolutionRate = resolution;

  return result;
}
