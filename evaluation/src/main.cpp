#include <iostream>
#include "ros/ros.h"
#include "evaluate.h"
#include "ramp_msgs/Update.h"

//Modified: add obstacle size constant 
const float OBSTACLE_SIZE = 0.3; // size of the obstacle in meters
ramp_msgs::Configuration current;

obstacle_struct obstacle; //Modified: add obstacle_struct variable. I know it is not the best place. But it is fast to do

CollisionDetection collision_;

bool handleRequest(ramp_msgs::EvaluationRequest::Request& req,
                   ramp_msgs::EvaluationRequest::Response& res) 
{
  std::cout<<"\nIn handling requests!\n";
  Evaluate ev(req);
  res.fitness = ev.perform(obstacle);
  std::cout<<"\nfitness: "<<res.fitness<<"\n";

  
  collision_.trajectory_ = req.trajectory;
  res.feasible = !collision_.perform();
  std::cout<<"\nfeasible: "<<(res.feasible ? "true" : "false")<<"\n";
  return true;
}

//When we get an update msg, set current
void updateCallback(const ramp_msgs::Update& msg) {
  current.K.clear();

  for(unsigned int i=0;i<msg.configuration.K.size();i++) {
    current.K.push_back(msg.configuration.K.at(i));
  }

  /*std::cout<<"\nGot update!";
  std::cout<<"\ncurrent: [";
  for(unsigned int i=0;i<current.K.size();i++) {
    std::cout<<", "<<current.K.at(i);
  }
  std::cout<<"]";*/

}

//Modified: Get the IR sensor value, calculate the box where the obstacle would be and evaluate
void sensorUpdate(const corobot_msgs::SensorMsg &msg)
// Get the sensor data, in this case the front infrared sensor, and reevaluate the trajectories to avoid the obstacles if detected
{
    // We care only about the sensor data coming from the front infrared sensor
    // Also, if the IR sensor reports a very high value (> 20m) that means that there is no obstacle detected within around 30cm in front
    if (msg.type == msg.INFRARED_FRONT) 
    {
        if (msg.value < 20)
        {
            std::cout<<"\nIR value: "<<msg.value<<"\n";
            // Figure out the square where the obstacle could be
            /*Modified:: I commented out because we don't know the robot position here at this point
              This is a very rough estimation of the obstacle as a big box that even includes part of the robot
            */
            obstacle.x1 = current.K.at(0) + msg.value;
            obstacle.x2 = obstacle.x1 + OBSTACLE_SIZE;
            obstacle.y1 = current.K.at(1) - (OBSTACLE_SIZE/2);
            obstacle.y2 = obstacle.y1 + OBSTACLE_SIZE;
            
            
            collision_.obstacle_list.push_back(obstacle);

            //Print out the list of obstacles
            for(unsigned int i=0;i<collision_.obstacle_list.size();i++) {
              std::cout<<"\nObstacle "<<i<<" - ";
              std::cout<<"x1: "<<collision_.obstacle_list.at(i).x1<<" ";
              std::cout<<"x2: "<<collision_.obstacle_list.at(i).x2<<" ";
              std::cout<<"y1: "<<collision_.obstacle_list.at(i).y1<<" ";
              std::cout<<"y2: "<<collision_.obstacle_list.at(i).y2;
            }

            /*//We need to make sure that x1 is lower than x2 and y1 is lower than y2
            if (obstacle.x1 > obstacle.x2)
            {
                float tmp = obstacle.x1;
                obstacle.x1 = obstacle.x2;
                obstacle.x2 = tmp;
            }
            if (obstacle.y1 > obstacle.y2)
            {
                float tmp = obstacle.y1;
                obstacle.y1 = obstacle.y2;
                obstacle.y2 = tmp;
            }*/
        }
    }
}

int main(int argc, char** argv) {


  ros::init(argc, argv, "evaluation");
  ros::NodeHandle handle;

  ros::ServiceServer service = handle.advertiseService("evaluation", handleRequest);
  ros::Subscriber sub_irSensor = handle.subscribe("infrared_data", 100, sensorUpdate); //Modified: subscribe to the ir sensor topic
  ros::Subscriber sub_current = handle.subscribe("update", 100, updateCallback); //Modified: subscribe to the ir sensor topic

  std::cout<<"\nSpinning...\n";
  ros::spin();


  std::cout<<"\nExiting Normally\n";
  return 0;
}
