#include "ros/ros.h"
#include "ramp_msgs/Configuration.h"
#include "planner.h"
#include "range.h"
#include "ramp_msgs/ModificationRequest.h"
#include "ramp_msgs/EvaluationRequest.h"
#include "yaml-cpp/yaml.h"
#include <XmlRpc.h>
#include <fstream>
#include <map>
#include <iterator>
 



Planner             my_planner; 
Utility             utility;
MotionState         start, goal;
std::vector<Range>  ranges;



// Broken
void initDOF(const XmlRpc::XmlRpcValue dof) {
  
  /*double a; 
  std::string s;
  for(unsigned int i=0;i<dof.size();i++) {
    //std::string s = static_cast<std::string>(dof[i][0]);
    //Range temp(dof[i][0].TypeDouble, dof[i][1].TypeDouble); 
    //std::vector< std::vector<double> > d = static_cast< std::vector< std::vector<double> > >(dof);
    //double* d = (double*) dof[i];
    //std::cout<<"\nd[0]: "<<d[0];
    //std::cout<<"\nd[1]: "<<d[1];
    //double d = static_cast<double>(dof[i][0]);
    //double a = static_cast<double>(dof[i][0]);
    std::cout<<"\nRange "<<i<<": "<<temp.toString();
  }*/


  /** Doing this dynamically isn't working
   *  so hardcode values in until it works */
  
  // Make some Ranges 
  Range range0(0, 3.5);
  Range range1(0, 3.5);
  Range range2(-PI, PI);

  // Set my_planner's ranges
  ranges.push_back(range0);
  ranges.push_back(range1);
  ranges.push_back(range2); 
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
  std::vector<float> ranges;
  XmlRpc::XmlRpcValue dof;


  // Get the id of the robot
  if(handle.searchParam("robot_info/id", key)) {
    handle.getParam(key, my_planner.id_);
    std::cout<<"\nPlanner ID: "<<my_planner.id_;
  }
  else {
    std::cout<<"\nCannot find ID parameter\n";
  }

  if(handle.hasParam("robot_info/DOF")) {
    handle.getParam("robot_info/DOF", dof); 
    initDOF(dof);


    //for(unsigned int i=0;i<dof.size();i++) {
      //std::cout<<"\ndd["<<i<<"]: ("<<dof[i][0]<<", "<<dof[i][1]<<")";
    //}
  }


  // Get the start and goal vectors
  std::vector<float> start;
  std::vector<float> goal;
  handle.getParam("robot_info/start", start);
  handle.getParam("robot_info/goal",  goal );
  initStartGoal(start, goal);
  
 
  


  std::cout<<"\nDone loading parameters. Press Enter to continue\n";
  //std::cin.get();
}






int main(int argc, char** argv) {
  srand( time(0));

  ros::init(argc, argv, "ramp_planner");
  ros::NodeHandle handle;
  
  ros::Subscriber sub_update_ = handle.subscribe("update", 1000, &Planner::updateCallback, &my_planner);



  /*for(int i=0;i<3;i++) {
    start.positions_.push_back( (float)rand() / ((float)RAND_MAX / 10) );
    start.velocities_.push_back((float)rand() / ((float)RAND_MAX / 10) );
    start.accelerations_.push_back((float)rand() / ((float)RAND_MAX / 10) );
    start.jerks_.push_back((float)rand() / ((float)RAND_MAX / 10) );
    
    goal.positions_.push_back( (float)rand() / ((float)RAND_MAX / 10) );
    goal.velocities_.push_back((float)rand() / ((float)RAND_MAX / 10) );
    goal.accelerations_.push_back((float)rand() / ((float)RAND_MAX / 10) );
    goal.jerks_.push_back((float)rand() / ((float)RAND_MAX / 10) );
  }
  std::cout<<"\nStart: "<<start.toString();
  std::cout<<"\nGoal: "<<goal.toString();

  std::cout<<"\nStart + Goal: "<<start.add(goal).toString();
  std::cout<<"\nStart - Goal: "<<start.subtract(goal).toString();

  std::cout<<"\nStart * 2: "<<start.multiply(2).toString();
  std::cout<<"\nStart / 2: "<<start.divide(2).toString();


  my_planner.start_ = start;
  my_planner.m_cc = goal;
  std::cout<<"\n\nstart: "<<my_planner.start_.toString();
  std::cout<<"\nm_cc: "<<my_planner.m_cc.toString();
  my_planner.setMi();

  std::cin.get();
  exit(1);*/



  // Load ros parameters
  loadParameters(handle);
  


  /** Initialize the Planner's handlers */ 
  my_planner.init(handle, start, goal, ranges); 

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

