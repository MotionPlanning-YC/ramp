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
 



Planner my_planner; 
Utility u;


const std::vector<Configuration> getStartGoal(bool robot0) {
  std::cout<<"\nIn getStartGoal";
  std::cout<<"\nrobot0: "<<robot0<<"\n";
  std::vector<Configuration> result;

  Configuration s, g;
  
  if(!robot0) {
    s.K_.push_back(0);
    s.K_.push_back(0);
    s.K_.push_back(0);
    
    g.K_.push_back(1.5);
    g.K_.push_back(1.5);
    g.K_.push_back(PI/4);
  }
  else {
    s.K_.push_back(3.5f);
    s.K_.push_back(2.f);
    s.K_.push_back(PI);
    
    g.K_.push_back(0.f);
    g.K_.push_back(3.5f);
    g.K_.push_back(PI);
  }

  result.push_back(s);
  result.push_back(g);

  return result;
}


void handleConfig(YAML::Node node) {
  std::cout<<"\nIn handleConfig: "<<node.Type()<<"\n";
}


void setDOF(const XmlRpc::XmlRpcValue dof) {
  
  double a; 
  std::string s;
  for(unsigned int i=0;i<dof.size();i++) {
    Range temp(dof[i][0].TypeDouble, dof[i][1].TypeDouble); 
    std::string s = static_cast<std::string>(dof);
    //double d = static_cast<double>(dof[i][0]);
    //double a = static_cast<double>(dof[i][0]);
    std::cout<<"\nRange "<<i<<": "<<temp.toString();
  }
}


void init_start_goal(const std::vector<float> s, const std::vector<float> g) {
  Configuration start, goal;

  for(unsigned int i=0;i<s.size();i++) {
    start.K_.push_back(s.at(i));
    goal.K_.push_back(g.at(i));
  }
}


void loadParameters(const ros::NodeHandle handle) {
  std::cout<<"\nLoading parameters\n";

  std::string key;
  int id;
  std::vector<float> ranges;
  XmlRpc::XmlRpcValue dof;


  // Get the id of the robot
  if(handle.searchParam("robot_info/id", key)) {
    handle.getParam(key, id);
    std::cout<<"\nkey: "<<key<<" val(id): "<<id;
  }

  if(handle.hasParam("robot_info/DOF")) {
    handle.getParam("robot_info/DOF", dof); 
    setDOF(dof);


    for(unsigned int i=0;i<dof.size();i++) {
      std::cout<<"\ndd["<<i<<"]: "<<dof[i][0];
    }
  }


  // Get the start and goal vectors
  std::vector<float> start;
  std::vector<float> goal;
  handle.getParam("/robot_info/start", start);
  handle.getParam("/robot_info/goal",  goal );
  init_start_goal(start, goal);
  
  

  std::cout<<"\nDone loading parameters. Press Enter to continue\n";
  std::cin.get();
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "ramp_planner");
  ros::NodeHandle handle;
  
  ros::Subscriber sub_update_ = handle.subscribe("update", 1000, &Planner::updateCallback, &my_planner);


  loadParameters(handle);
  
  
  std::string update_topic;
  handle.getParam("ramp_planner/robot_update", update_topic);
  std::cout<<"\nupdate_topic:"<<update_topic;

  
  // Make some Ranges 
  srand( time(0));
  Range range0(0, 3.5);
  Range range1(0, 3.5);
  Range range2(-PI, PI);

  // Set my_planner's ranges
  my_planner.ranges_.push_back(range0);
  my_planner.ranges_.push_back(range1);
  my_planner.ranges_.push_back(range2);
  
  // Make Configurations
  std::vector<Configuration> s_g = getStartGoal(update_topic == "/robot_0/update");
  Configuration s = s_g.at(0);
  Configuration g = s_g.at(1);

  s.ranges_ = my_planner.ranges_;
  g.ranges_ = my_planner.ranges_;
  
  // Set start and goal
  my_planner.start_   = s;
  my_planner.goal_    = g;
  std::cout<<"\nStart:"<<s.toString();
  std::cout<<"\nGoal:"<<g.toString();
  
  // Set transform matrix for odometry
  my_planner.setT_od_w(s.K_);


  /** Initialize the Planner's handlers */ 
  my_planner.init(handle); 

  /** Set Planner Robot ID */
  if(update_topic == "/robot_0/update") {
    my_planner.id_ = 0;
  }
  else {
    my_planner.id_ = 1;
  }
  /** End building Planner */


  // Don't start planner, just wait for updates
  //while(ros::ok()) {ros::spinOnce();}
  
  
  

  /******* Start the planner *******/
  std::cout<<"\nPress Enter to start the planner\n";
  std::cin.get(); 
  
  my_planner.go();
  
  
  ros::spinOnce();
  std::cout<<"\nExiting Normally\n";
  return 0;
}

