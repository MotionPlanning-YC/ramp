#include <iostream>
#include "ros/ros.h"
#include "evaluate.h"

//Modified: add obstacle size constant 
const float OBSTACLE_SIZE = 0.5; // size of the obstacle in meters


obstacle_struct obstacle; //Modified: add obstacle_struct variable. I know it is not the best place. But it is fast to do


bool handleRequest(ramp_msgs::EvaluationRequest::Request& req,
                   ramp_msgs::EvaluationRequest::Response& res) 
{
  Evaluate ev(req);
  res.fitness = ev.perform(obstacle);
  res.feasible = true;
  return true;
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
            // Figure out the square where the obstacle could be
            /*Modified:: I commented out because we don't know the robot position here at this point
              This is a very rough estimation of the obstacle as a big box that even includes part of the robot
            */
        /*    obstacle.x1 = robot.posx - OBSTACLE_SIZE/2;
            obstacle.x2 = robot.posx + OBSTACLE_SIZE/2;
            obstacle.y1 = robot.posy - OBSTACLE_SIZE/2;
            obstacle.y2 = robot.posy + OBSTACLE_SIZE/2;
            */
            //We need to make sure that x1 is lower than x2 and y1 is lower than y2
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
            }
        }
        else
        {
            obstacle.x1 = 0;
            obstacle.x2 = 0;
            obstacle.y1 = 0;
            obstacle.y2 = 0;
        }
    }
}

int main(int argc, char** argv) {


  ros::init(argc, argv, "evaluation");
  ros::NodeHandle handle;

  ros::ServiceServer service = handle.advertiseService("evaluation", handleRequest);
  ros::Subscriber irSensor = handle.subscribe("infrared_data", 100, sensorUpdate); //Modified: subscribe to the ir sensor topic

  std::cout<<"\nSpinning...\n";
  ros::spin();


  std::cout<<"\nExiting Normally\n";
  return 0;
}
