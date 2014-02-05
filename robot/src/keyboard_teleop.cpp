#include <ros/ros.h>
#include <iostream>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "corobot_msgs/MotorCommand.h"


#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

int kfd = 0;
struct termios cooked, raw;
int l_wheel = 0, r_wheel = 0;
corobot_msgs::MotorCommand motorCommand;


int main(int argc, char** argv) {

  ros::init(argc, argv, "keyboard_teleop");
  ros::NodeHandle handle;
  ros::Publisher pub_motors = handle.advertise<corobot_msgs::MotorCommand>("PhidgetMotor", 1000);
  ros::Publisher pub_update = handle.advertise<corobot_msgs::MotorCommand>("update", 1000);

  char c;
  tcgetattr(kfd, &cooked); 
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");

  while(ros::ok()) {
    if(read(kfd, &c, 1) < 0) {
      perror("read(): ");
      exit(-1);
    }

    switch(c) {
      case KEYCODE_U:
        motorCommand.leftSpeed  = 25;
        motorCommand.rightSpeed = 25;
        
        break;

      case KEYCODE_D:
        motorCommand.leftSpeed = -25;
        motorCommand.rightSpeed = -25;

        break;

      case KEYCODE_L:
        motorCommand.leftSpeed = -25;
        motorCommand.rightSpeed = 25;

        break;

      case KEYCODE_R:
        motorCommand.leftSpeed = 25;
        motorCommand.rightSpeed = -25;

        break;
    }

    motorCommand.secondsDuration = 1;
    motorCommand.acceleration = 25;

    pub_motors.publish(motorCommand);

  }

  std::cout<<"\nExiting Normally\n";
  return 0;
}
