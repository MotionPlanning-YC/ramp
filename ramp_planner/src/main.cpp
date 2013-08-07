#include "ros/ros.h"
#include "ramp_msgs/Configuration.h"
#include "planner.h"
#include "range.h"
 
Planner my_planner; 
ros::Publisher  pub_traj_requests;
ros::Subscriber sub_traj;


void init_pub_sub(ros::NodeHandle& handle, Planner& planner) {
  pub_traj_requests = handle.advertise<ramp_msgs::TrajectoryRequest>("traj_requests", 1000);  
  sub_traj = handle.subscribe("trajs", 1000, &Planner::trajCallback, &planner);
}


void send_paths() {
  
  std::vector<float> t;

  //For each path...
  for(unsigned int i=0;i<my_planner.paths_.size();i++) {
    
    //Hardcode some times
    //Eventually, the planner will determine times itself based on real-world constraints 
    for(unsigned int j=0;j<my_planner.paths_.at(i).all_.size();j++) {
      t.push_back(j+2);
    }

    ramp_msgs::TrajectoryRequest msg_request = my_planner.buildTrajectoryRequestMsg(i, t);  

    pub_traj_requests.publish(msg_request);

    t.clear();
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

  std::cout<<"\n\nPress Enter to publish initial Trajectory Request msgs!\n";
  std::cin.get();

  send_paths();
  

  std::cout<<"\nSpinning...\n";
  ros::spin();

  std::cout<<"\nExiting Normally\n";
  return 0;
}

