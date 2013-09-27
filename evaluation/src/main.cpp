#include <iostream>
#include "ros/ros.h"
#include "evaluate.h"
#include "ramp_msgs/Update.h"
#include "ramp_msgs/ObjectList.h"


Evaluate ev;

bool handleRequest(ramp_msgs::EvaluationRequest::Request& req,
                   ramp_msgs::EvaluationRequest::Response& res) 
{
  std::cout<<"\nIn handling requests!\n";
  ev.setRequest(req);
  res.fitness = ev.performFitness();
  std::cout<<"\nfitness: "<<res.fitness<<"\n";

  
  //collision_.trajectory_ = req.trajectory;
  //res.feasible = !collision_.perform();
  if(ev.objList_.ir_object.x1 != 0.0 ||
     ev.objList_.ir_object.x2 != 0.0 ||
     ev.objList_.ir_object.y1 != 0.0 ||
     ev.objList_.ir_object.y2 != 0.0)
   // we check if we actually have an object, if so we check the collision
  {
    std::cout<<"checking if feasible\n";
    res.feasible  = !ev.performCollisionDetection();
  }
  else
  {
    res.feasible = true;
  }
  std::cout<<"\nfeasible: "<<(res.feasible ? "true" : "false")<<"\n";
  return true;
}

void objectListCallback(const ramp_msgs::ObjectList& ol) {
  ev.objList_ = ol;
}


int main(int argc, char** argv) {


  ros::init(argc, argv, "evaluation");
  ros::NodeHandle handle;

  ros::ServiceServer service = handle.advertiseService("evaluation", handleRequest);
  ros::Subscriber sub_obj_list = handle.subscribe("object_list", 1000, objectListCallback);

  std::cout<<"\nSpinning...\n";
  ros::spin();


  std::cout<<"\nExiting Normally\n";
  return 0;
}
