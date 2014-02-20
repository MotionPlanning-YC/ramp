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


const std::vector<Configuration> getStartGoal(bool robot1) {
  std::cout<<"\nIn getStartGoal";
  std::cout<<"\nrobot1: "<<robot1<<"\n";
  std::vector<Configuration> result;

  Configuration s, g;
  
  if(!robot1) {
    s.K_.push_back(0);
    s.K_.push_back(2);
    s.K_.push_back(0);
    
    g.K_.push_back(3.5f);
    g.K_.push_back(2.f);
    g.K_.push_back(0);
  }
  else {
    s.K_.push_back(2.f);
    s.K_.push_back(3.5f);
    s.K_.push_back(-PI/2.);
    
    g.K_.push_back(2.f);
    g.K_.push_back(0.f);
    g.K_.push_back(-PI/2.);
  }

  result.push_back(s);
  result.push_back(g);

  return result;
}


void handleConfig(YAML::Node node) {
  std::cout<<"\nIn handleConfig: "<<node.Type()<<"\n";
  std::cout<<"\nnode[\"id\"]: "<<node["id"];
}


void loadParameters(const ros::NodeHandle handle) {
  std::string key;
  int id;
  std::vector<float> ranges;
  std::vector<float> start;
  std::vector<float> goal;


  // Get the id of the robot
  if(handle.searchParam("id", key)) {
    handle.getParam(key, id);
    std::cout<<"\nkey: "<<key<<" val(id): "<<id;
  }

 
  
  std::vector<float> dof;
  // Get the ranges for the degrees of freedom
  if(handle.searchParam("DOF", key)) {
    handle.getParam(key, dof);
    for(unsigned int i=0;i<dof.size();i++) {
      std::cout<<"\ndof["<<i<<"]: "<<dof.at(i);
    }
  }

  

  if(handle.searchParam("", key)) {
    int val;
    handle.getParam(key, val);
    std::cout<<"\nkey: "<<key<<" val: "<<val;
  }

}


int main(int argc, char** argv) {
  ros::init(argc, argv, "ramp_planner");
  ros::NodeHandle handle;
  
  ros::Subscriber sub_update_ = handle.subscribe("update", 1000, &Planner::updateCallback, &my_planner);


  //loadParameters(handle);
  
  
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
  std::vector<Configuration> s_g = getStartGoal(update_topic == "/robot1/update");
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
  if(update_topic == "/robot1/update") {
    my_planner.id_ = 1;
  }
  else {
    my_planner.id_ = 2;
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

