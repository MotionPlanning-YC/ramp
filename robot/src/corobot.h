
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
#include "ramp_msgs/Trajectory.h"
#include "tf/transform_datatypes.h"
#include <math.h>

class Corobot {
  public:
    
    Corobot();
    ~Corobot();
    
    //Member functions 
    void drive(corobot_msgs::MotorCommand msg) const;
    void driveStraight(const unsigned int speed) const;
    void turn(const unsigned int speed, const bool cwise) const;
    void turn(const float speed, const float angle) const; 
    void stop() const;
    void updateState(const nav_msgs::Odometry::ConstPtr& msg);
    
    void updateTrajectory(const ramp_msgs::Trajectory msg); 
    void moveOnTrajectory();

    //Data Members
    ros::Publisher                    pub_phidget_motor_;
    ros::Publisher                    pub_twist_;
    ros::Subscriber                   sub_odometry_;
    geometry_msgs::Pose2D             configuration_; 
    geometry_msgs::Twist              velocity_;
    ramp_msgs::Trajectory             trajectory_;

    
    //static const members
    static const std::string TOPIC_STR_PHIDGET_MOTOR;
    static const std::string TOPIC_STR_ODOMETRY;
    static const std::string TOPIC_STR_TWIST;
    static const int POSE_COUNT_THRESHOLD = 5;
    static const int ACCELERATION_CONSTANT = 50;

  private:
    const float getSpeedToWaypoint(const trajectory_msgs::JointTrajectoryPoint waypoint1, const trajectory_msgs::JointTrajectoryPoint waypoint2) const ;

    const float getAngularSpeed(const float direction1, const float direction2) const;
    
    float getTrajectoryOrientation(const trajectory_msgs::JointTrajectoryPoint waypoint1, const trajectory_msgs::JointTrajectoryPoint waypoint2) const;
    
    void calculateSpeedsAndTime ();
    
    bool move; 
    //holds the last 5 thetas to average
    std::vector<double> thetas_; 
    
    std::vector<ros::Time> end_times; // Save the ending time of each waypoint
    std::vector<float> speeds; // Linear speed for each trajectory
    std::vector<float> angular_speeds_knotpoints; //Angular Speed needed over 3s to get the correct orientation after each knot point reached.
    std::vector<float> angular_speeds_waypoints; // Angular Speed needed over 3s to get to the correct direction to be able to reach the next waypoint.
};

#endif
