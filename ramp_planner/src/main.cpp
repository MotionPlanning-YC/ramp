#include "ros/ros.h"
#include "planner.h"

 


Planner             my_planner; 
MotionState         start, goal;
std::vector<Range>  ranges;
int                 id;



// Initializes a vector of Ranges that the Planner is initialized with
void initDOF(const std::vector<double> dof_min, const std::vector<double> dof_max) {
  
  for(unsigned int i=0;i<dof_min.size();i++) {
    Range temp(dof_min.at(i), dof_max.at(i));
    ranges.push_back(temp); 
  }


} // End initDOF




// Initializes global start and goal variables
void initStartGoal(const std::vector<float> s, const std::vector<float> g) {
  std::cout<<"\ns.size(): "<<s.size();
  std::cout<<"\ng.size(): "<<g.size();
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
  std::cout<<"\nHandle NS: "<<handle.getNamespace();

  std::string key;
  std::vector<double> dof_min;
  std::vector<double> dof_max;


  // Get the id of the robot
  if(handle.hasParam("robot_info/id")) {
    handle.getParam("robot_info/id", id);
  }
  else {
    ROS_ERROR("Did not find parameter robot_info/id");
  }


  // Get the dofs
  if(handle.hasParam("robot_info/DOF_min") && 
      handle.hasParam("robot_info/DOF_max")) 
  {

    handle.getParam("robot_info/DOF_min", dof_min); 
    handle.getParam("robot_info/DOF_max", dof_max); 

    initDOF(dof_min, dof_max);
  }
  else {
    ROS_ERROR("Did not find parameters robot_info/DOF_min, robot_info/DOF_max");
  }


  // Get the start and goal vectors
  if(handle.hasParam("robot_info/start") &&
      handle.hasParam("robot_info/goal"))
  {
    std::vector<float> p_start;
    std::vector<float> p_goal;
    handle.getParam("robot_info/start", p_start);
    handle.getParam("robot_info/goal",  p_goal );
    initStartGoal(p_start, p_goal);
  }
  else {
    ROS_ERROR("Did not find parameters robot_info/start, robot_info/goal");
  }
  

  std::cout<<"\n------- Done loading parameters -------\n";
    std::cout<<"\nID: "<<id;
    std::cout<<"\nStart: "<<start.toString();
    std::cout<<"\nGoal: "<<goal.toString();
    std::cout<<"\nRanges: ";
    for(uint8_t i=0;i<ranges.size();i++) {
      std::cout<<"\n  "<<i<<": "<<ranges.at(i).toString();
    }
  std::cout<<"\n---------------------------------------";
}






int main(int argc, char** argv) {
  srand( time(0));

  ros::init(argc, argv, "ramp_planner");
  ros::NodeHandle handle;

  std::cout<<"\nHandle namespace: "<<handle.getNamespace();
  
  ros::Subscriber sub_update_ = handle.subscribe("update", 1000, &Planner::updateCallback, &my_planner);


  // Load ros parameters
  loadParameters(handle);
  
  


  /** Initialize the Planner's handlers */ 
  my_planner.init(id, handle, start, goal, ranges); 

  std::cout<<"\nStart: "<<my_planner.start_.toString();
  std::cout<<"\nGoal: "<<my_planner.goal_.toString();
  
  

  /******* Start the planner *******/
  std::cout<<"\nPress Enter to start the planner\n";
  std::cin.get(); 
  
  my_planner.go();


  MotionState exp_results = my_planner.findAverageDiff();
  std::cout<<"\n\nAverage Difference: "<<exp_results.toString();
  
  
  std::cout<<"\n\nExiting Normally\n";
  return 0;
}

