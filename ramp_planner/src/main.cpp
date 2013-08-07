#include "ros/ros.h"
#include "ramp_msgs/Configuration.h"
#include "planner.h"
#include "range.h"
 
Planner my_planner; 
ros::Publisher  pub_path;
ros::Subscriber sub_traj;


void init_pub_sub(ros::NodeHandle& handle, Planner& planner) {
  pub_path = handle.advertise<ramp_msgs::TrajectoryRequest>("traj_requests", 1000);  
  sub_traj = handle.subscribe("trajs", 1000, &Planner::trajCallback, &planner);
}


void send_paths() {
  
  //For each path...
  for(unsigned int i=0;i<my_planner;i++) {
    
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "planner");

  ros::NodeHandle handle;
  
  
  init_pub_sub(handle, my_planner);

  srand( time(NULL));
  Range range0(5.2, 911.7);
  Range range1(0, 180);
  Range range2(30, 150);


  
  my_planner.ranges_.push_back(range0);
  my_planner.ranges_.push_back(range1);
  my_planner.ranges_.push_back(range2);
  
  Configuration s;
  Configuration g;
  s.ranges_ = my_planner.ranges_;
  g.ranges_ = my_planner.ranges_;
  s.random();
  g.random();
  std::cout<<"\nStart:"<<s.toString();
  std::cout<<"\nGoal:"<<g.toString();

  my_planner.start_ = s;
  my_planner.goal_ = g;

  my_planner.initialization();

  std::cout<<"\nAfter initialization!\n";
  
  /*for(unsigned int i=0;i<my_planner.paths_.size();i++) {
    std::cout<<"\n\nPath "<<i;
    std::cout<<my_planner.paths_.at(i).toString();
  }*/


  std::cout<<"\n\nPress Enter to publish a Trajectory Request msg!\n";
  std::cin.get();

  //Publish
  std::vector<float> t;
  for(unsigned int i=0;i<my_planner.paths_.at(0).all_.size()-1;i++) {
    t.push_back(1);
  }

  ramp_msgs::TrajectoryRequest msg_r = my_planner.buildTrajectoryRequestMsg(0, t, 5);
  //ramp_msgs::Path msg_p = my_planner.paths_.at(0).buildPathMsg(); 
  pub_path.publish(msg_r);


  ros::spinOnce();

  std::cout<<"\nExiting Normally\n";
  return 0;
}

