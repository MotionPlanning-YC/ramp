#include "ros/ros.h"
#include "planner.h"
 



Planner             my_planner; 
MotionState         start, goal;
std::vector<Range>  ranges;
int                 population_size;
bool                sub_populations;



// Initializes a vector of Ranges that the Planner is initialized with
void initDOF(const std::vector<double> dof_min, const std::vector<double> dof_max) {
  
  for(unsigned int i=0;i<dof_min.size();i++) {
    Range temp(dof_min.at(i), dof_max.at(i));
    ranges.push_back(temp); 
  }

} // End initDOF




// Initializes global start and goal variables
void initStartGoal(const std::vector<float> s, const std::vector<float> g) {
  for(unsigned int i=0;i<s.size();i++) {
    start.positions_.push_back(s.at(i));
    goal.positions_.push_back(g.at(i));

    start.velocities_.push_back(0);
    goal.velocities_.push_back(0);

    start.accelerations_.push_back(0);
    goal.accelerations_.push_back(0);

    start.jerks_.push_back(0);
    goal.jerks_.push_back(0);
  }
} // End initStartGoal





/** Loads all of the ros parameters from .yaml 
 *  Calls initDOF, initStartGoal */
void loadParameters(const ros::NodeHandle handle) {
  std::cout<<"\nLoading parameters\n";

  std::string key;
  int id;
  std::vector<double> dof_min;
  std::vector<double> dof_max;


  // Get the id of the robot
  if(handle.hasParam("robot_info/id")) {
    handle.getParam("robot_info/id", my_planner.id_);
  }



  // Get the dofs
  if(handle.hasParam("robot_info/DOF_min") && 
      handle.hasParam("robot_info/DOF_max")) 
  {

    handle.getParam("robot_info/DOF_min", dof_min); 
    handle.getParam("robot_info/DOF_max", dof_max); 

    initDOF(dof_min, dof_max);
  }



  // Get the start and goal vectors
  if(handle.hasParam("robot_info/start") &&
      handle.hasParam("robot_info/goal"))
  {
    std::vector<float> start;
    std::vector<float> goal;
    handle.getParam("/robot_info/start", start);
    handle.getParam("/robot_info/goal",  goal );
    initStartGoal(start, goal);
  }
  


  if(handle.hasParam("ramp_planner/population_size")) {
    handle.getParam("ramp_planner/population_size", population_size); 
    std::cout<<"\npopulation_size: "<<population_size;
  }


  
  if(handle.hasParam("ramp_planner/sub_populations")) {
    handle.getParam("ramp_planner/sub_populations", sub_populations);
    std::cout<<"\nsub_populations: "<<sub_populations;
  }


  std::cout<<"\nDone loading parameters. Press Enter to continue\n";
  std::cin.get();
}






int main(int argc, char** argv) {
  ros::init(argc, argv, "ramp_planner");
  ros::NodeHandle handle;
  
  ros::Subscriber sub_update_ = handle.subscribe("update", 1000, &Planner::updateCallback, &my_planner);


  // Load ros parameters
  loadParameters(handle);

  std::cout<<"\nmy_planner.id: "<<my_planner.id_;
  
  exit(0); 
  
  // Pretty sure this isn't needed any longer. 
  // Can't remember what it was used for
  // Will leave it for now in case there are issues and it sparks ideas
  /*std::string update_topic;
  handle.getParam("ramp_planner/robot_update", update_topic);
  std::cout<<"\nupdate_topic:"<<update_topic;*/


  /** Initialize the Planner's handlers */ 
  my_planner.init(handle, start, goal, ranges); 

  
  
  

  /******* Start the planner *******/
  std::cout<<"\nPress Enter to start the planner\n";
  std::cin.get(); 
  
  my_planner.go();
  
  
  ros::spinOnce();
  std::cout<<"\nExiting Normally\n";
  return 0;
}

