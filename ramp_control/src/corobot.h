
#ifndef COROBOT_H
#define COROBOT_H

#include "ros/ros.h"
#include "utility.h"

/** ROS msgs and srvs */
#include "nav_msgs/Odometry.h"
#include "corobot_msgs/MotorCommand.h"
#include "geometry_msgs/Twist.h"
#include "ramp_msgs/Configuration.h"
#include "ramp_msgs/Trajectory.h"
#include "tf/transform_datatypes.h"
#include "ramp_msgs/Update.h"
#include <math.h>

class Corobot {
  public:
    
    Corobot();
    ~Corobot();
    
    /** Methods **/ 
    void drive(corobot_msgs::MotorCommand msg) const;
    void driveStraight(const unsigned int speed) const;
    void turn(const unsigned int speed, const bool cwise) const;
    void turn(const float speed, const float angle) const; 
    void stop() const;
    void updateState(const nav_msgs::Odometry& msg);
    
    void updateTrajectory(const ramp_msgs::Trajectory msg); 
    void moveOnTrajectory(bool simulation);

    void setConfiguration(float x, float y, float theta);
    
    void updatePublishTimer(const ros::TimerEvent&);
    void controlCycle(geometry_msgs::Twist twist, ros::Time end_time, ros::Rate r);
    
    void sendTwist(const geometry_msgs::Twist twist) const;



    /** Data Members **/
    ros::Publisher                    pub_phidget_motor_;
    ros::Publisher                    pub_twist_;
    ros::Publisher                    pub_cmd_vel_;
    ros::Publisher                    pub_update_;
    ros::Subscriber                   sub_odometry_;
    ramp_msgs::Configuration          configuration_; 
    geometry_msgs::Twist              velocity_;
    ramp_msgs::Trajectory             trajectory_;
    ros::Timer                        timer_;

    double initial_theta;

    
    // static const members
    static const std::string TOPIC_STR_PHIDGET_MOTOR;
    static const std::string TOPIC_STR_ODOMETRY;
    static const std::string TOPIC_STR_UPDATE;
    static const std::string TOPIC_STR_TWIST;
    static const int POSE_COUNT_THRESHOLD = 1;
    static const int ACCELERATION_CONSTANT = 50;

  private:

    /** Methods **/
    const float getSpeedToWaypoint(const trajectory_msgs::JointTrajectoryPoint waypoint1, const trajectory_msgs::JointTrajectoryPoint waypoint2) const;
    const float getAngularSpeed(const float direction1, const float direction2) const;
    float getTrajectoryOrientation(const trajectory_msgs::JointTrajectoryPoint waypoint1, const trajectory_msgs::JointTrajectoryPoint waypoint2) const;
    void calculateSpeedsAndTime ();
    void printVectors() const;
    
    
    /** Data Members **/
    Utility u;
    
    const unsigned int k_dof_;
    
    std::vector<ros::Time> end_times; // Save the ending time of each waypoint
    std::vector<float> speeds; //  Linear speed for each trajectory
    std::vector<float> angular_speeds; // Angular Speed needed over 3s to get the correct orientation after each knot point reached.
    std::vector<float> orientations; // The orientation needed to be at each knotpoint.

    geometry_msgs::Twist twist_;
    float angle_at_start; // the angle of the robot when the robot gets a trajectory.

    bool restart;
    bool mutex_;
    bool moving_;
    bool trajec_updated_;
    
    int num;
    int num_traveled;
    int i_knot_points;

    void lockMutex();
    void releaseMutex();
    void sendTwist() const;
    const bool checkImminentCollision() const;
};

#endif
