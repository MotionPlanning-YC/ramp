#include "ros/ros.h"
#include "trajectory.h"


int main(int argc, char** argv) {
  

  ros::init(argc, argv, "traj_gen_mobile_base");
  ros::NodeHandle handle;

  ros::Publisher pub_traj = handle.advertise<ramp_msgs::TrajectoryWithKnots>("traj", 1000);

  /** The following is just for testing purposes. */

  //Create some knot points
  geometry_msgs::Pose2D kp_start;
  kp_start.x = 0;
  kp_start.y = 0;
  kp_start.theta = 50;

  geometry_msgs::Pose2D kp_mid;
  kp_mid.x = 1;
  kp_mid.y = 0;
  kp_mid.theta = 50;

  geometry_msgs::Pose2D kp_end;
  kp_end.x = 0.5;
  kp_end.y = 0;
  kp_end.theta = 50;

  //Create a trajectory
  Trajectory traj;
  
  //set the resolution rate
  traj.resolutionRate_ = 5;
  
  //Set knot points
  traj.knot_points_.push_back(kp_start);;
  traj.knot_points_.push_back(kp_mid);
  traj.knot_points_.push_back(kp_end);

  //Set times
  traj.t_.push_back(2);
  traj.t_.push_back(1);
   
  //Build the segments
  traj.buildSegments(); 
  
  //Print info on segments
  //need some toString methods...
  for(unsigned int i=0;i<traj.segments_.size();i++) {
    std::cout<<"\nSegment "<<i<<":";
    std::cout<<"\nX slope:"<<traj.segments_.at(i).a1_.at(0);
    std::cout<<"\nY slope:"<<traj.segments_.at(i).a1_.at(1);
    std::cout<<"\nTheta slope:"<<traj.segments_.at(i).a1_.at(2)<<"\n";
  }

  //Generate the trajectory
  std::vector<MotionState> Ms = traj.generate();
  
  //Print info on the trajectory
  //need toString...
  for(unsigned int i=0;i<Ms.size();i++) {
    std::cout<<"\n\n\nMs["<<i<<"]:";
    for(unsigned int j=0;j<3;j++) {
      if (j==0)
        std::cout<<"\nP.x:";
      else if(j==1)
        std::cout<<"\nP.y:";
      else
        std::cout<<"\nP.theta:";
      std::cout<<Ms.at(i).p_.at(j);
    }
    
    
    for(unsigned int j=0;j<3;j++) {
      if (j==0)
        std::cout<<"\nV.x:";
      else if(j==1)
        std::cout<<"\nV.y:";
      else
        std::cout<<"\nV.theta:";
      std::cout<<Ms.at(i).v_.at(j);
    }
    
    
    for(unsigned int j=0;j<3;j++) {
      if (j==0)
        std::cout<<"\nA.x:";
      else if(j==1)
        std::cout<<"\nA.y:";
      else
        std::cout<<"\nA.theta:";
      std::cout<<Ms.at(i).v_.at(j);
    }
  }

  //Build a TrajectoryWithKnots msg
  ramp_msgs::TrajectoryWithKnots msg = traj.buildTrajectoryMsg();
  
  //Print the knot points
  for(unsigned int i=0;i<msg.index_knot_points.size();i++) {
    std::cout<<"\nmsg.index_knot_points["<<i<<"]:";
    for(unsigned int j=0;j<3;j++) {
      if (j==0)
        std::cout<<"\nP.x:";
      else if(j==1)
        std::cout<<"\nP.y:";
      else
        std::cout<<"\nP.theta:";
      std::cout<<Ms.at(msg.index_knot_points.at(i)).p_.at(j);
    }
    
    
    for(unsigned int j=0;j<3;j++) {
      if (j==0)
        std::cout<<"\nV.x:";
      else if(j==1)
        std::cout<<"\nV.y:";
      else
        std::cout<<"\nV.theta:";
      std::cout<<Ms.at(msg.index_knot_points.at(i)).v_.at(j);
    }
    
    
    for(unsigned int j=0;j<3;j++) {
      if (j==0)
        std::cout<<"\nA.x:";
      else if(j==1)
        std::cout<<"\nA.y:";
      else
        std::cout<<"\nA.theta:";
      std::cout<<Ms.at(msg.index_knot_points.at(i)).v_.at(j);
    }
  }

  //Publish the message and quit
  std::cout<<"\nPress Enter to publish!\n";
  std::cin.get();

  pub_traj.publish(msg);
  std::cout<<"\nMessage published!";

  std::cout<<"\nPress Enter to exit\n"; 
  std::cin.get();
  std::cout<<"\nExiting Normally\n";
  return 0;
}
