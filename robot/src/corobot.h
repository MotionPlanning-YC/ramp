
#ifndef COROBOT_H
#define COROBOT_H

#include "ros/ros.h"
#include "base_configuration.h"
#include "utility.h"

/** ROS msgs and srvs */
#include "nav_msgs/Odometry.h"
#include "corobot_msgs/MotorCommand.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "ramp_msgs/TrajectoryWithKnots.h"
#include "tf/transform_datatypes.h"

class Corobot {
  public:
    
    Corobot();
    ~Corobot();
    
    //Member functions 
    void drive(corobot_msgs::MotorCommand msg) const;
    void driveStraight(unsigned int speed) const;
    void turn(unsigned int speed, bool cwise) const;
    void turn(float speed, float angle) const; 
    void stop() const;
    void updateState(const nav_msgs::Odometry::ConstPtr& msg);
    
    void updateTrajectory(const ramp_msgs::TrajectoryWithKnots msg); 
    void moveOnTrajectory();

    //Data Members
    ros::Publisher                    pub_phidget_motor_;
    ros::Publisher                    pub_twist_;
    ros::Subscriber                   sub_odometry_;
    geometry_msgs::Pose2D             configuration_; 
    geometry_msgs::Twist              velocity_;
    ramp_msgs::TrajectoryWithKnots    trajectory_;

    
    //static const members
    static const std::string TOPIC_STR_PHIDGET_MOTOR;
    static const std::string TOPIC_STR_ODOMETRY;
    static const std::string TOPIC_STR_TWIST;
    static const int POSE_COUNT_THRESHOLD = 5;
    static const int ACCELERATION_CONSTANT = 50;

  private:
    float getSpeedToWaypoint(trajectory_msgs::JointTrajectoryPoint waypoint1, trajectory_msgs::JointTrajectoryPoint waypoint2);
    
    bool move; 
    std::vector<double> thetas_; //holds the last 5 thetas to average
};

#endif
