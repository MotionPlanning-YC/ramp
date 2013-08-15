#include "ros/ros.h"
#include "ramp_msgs/ModificationRequest.h"
#include "ramp_msgs/Range.h"
#include "utility.h"
#include "insert.h"
#include "delete.h"
#include "change.h"
#include "swap.h"
#include "crossover.h"

bool handleRequest(ramp_msgs::ModificationRequest::Request& req,
                   ramp_msgs::ModificationRequest::Response& res)
{

  Insert insert(req.paths.at(0));
  res.mod_paths.push_back(insert.perform());
  
  //Delete del(req.paths.at(0));
  //res.mod_paths.push_back(del.perform());
  
  //Change change(req.paths.at(0));
  //res.mod_paths.push_back(change.perform());

  //Swap swap(req.paths.at(0));
  //res.mod_paths.push_back(swap.perform());
  
  //Crossover cross(req.paths.at(0), req.paths.at(1));
  //res.mod_paths = cross.perform();
  
  //For now, just push back on the paths passed in
  /*for(unsigned int i=0;i<req.paths.size();i++) {
    res.mod_paths.push_back(req.paths.at(i));
  }*/

  return true;
}




int main(int argc, char** argv) {
  ros::init(argc, argv, "path_modification");
  srand(time(0)); 

  ros::NodeHandle handle;

  ros::ServiceServer service = handle.advertiseService("path_modification", handleRequest); 

  Utility u;

  ramp_msgs::Path p1;
  ramp_msgs::Range r;
  r.min=0;
  r.max=10;
  for(unsigned int i=0;i<10;i++) {
    ramp_msgs::Configuration c;
    c.K.push_back(i);
    c.ranges.push_back(r); 
    p1.configurations.push_back(c);
  }
  
  std::cout<<"\nPath p1:"<<u.toString(p1);

  ramp_msgs::Path p2;
  for(unsigned int i=10;i>0;i--) {
    ramp_msgs::Configuration c;
    c.K.push_back(i);
    c.ranges.push_back(r); 
    p2.configurations.push_back(c);
  }
  
  std::cout<<"\nPath p2:"<<u.toString(p2);
  //Insert insert(p);
  //Delete del(p);
  //Change cha(p);
  //Swap swap(p);
  Crossover cross(p1, p2);

  //do insertion
  //ramp_msgs::Path a = insert.perform();
  //ramp_msgs::Path a = del.perform();
  //ramp_msgs::Path a = cha.perform();
  //ramp_msgs::Path a = swap.perform();
  std::vector<ramp_msgs::Path> as = cross.perform();
  std::cout<<"\nnew path1:"<<u.toString(as.at(0));
  std::cout<<"\nnew path2:"<<u.toString(as.at(1));


  std::cout<<"\nOutside of perform\n";
  std::cout<<"\nSpinning...\n";
  ros::spin();
  std::cout<<"\nExiting Normally\n";
  return 0;
}
