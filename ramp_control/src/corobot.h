
#ifndef COROBOT_H
#define COROBOT_H

#include "ros/ros.h"
#include "utility.h"
#include "nav_msgs/Odometry.h"
#include "corobot_msgs/MotorCommand.h"
#include "geometry_msgs/Twist.h"
#include "ramp_msgs/Configuration.h"
#include "ramp_msgs/Trajectory.h"
#include "tf/transform_datatypes.h"
#include "ramp_msgs/MotionState.h"
#include "trajectory_request_handler.h"
#include <math.h>

class Corobot {
  public:
    
    Corobot();
    ~Corobot();
    
    /** Methods **/ 
    void init(ros::NodeHandle& h);

    void drive(corobot_msgs::MotorCommand msg) const;
    void driveStraight(const unsigned int speed) const;
    void turn(const unsigned int speed, const bool cwise) const;
    void turn(const float speed, const float angle) const; 
    void stop() const;

    void moveOnTrajectory(bool simulation);
    void updateState(const nav_msgs::Odometry& msg);
    void updateTrajectory(const ramp_msgs::Trajectory msg); 
    void updatePublishTimer(const ros::TimerEvent&);
    void sendTwist(const geometry_msgs::Twist twist) const;
    void controlCycle(geometry_msgs::Twist twist, ros::Time end_time, ros::Rate r);



    /** Data Members **/
    ros::Publisher                    pub_phidget_motor_;
    ros::Publisher                    pub_twist_;
    ros::Publisher                    pub_cmd_vel_;
    ros::Publisher                    pub_update_;
    ros::Subscriber                   sub_odometry_;
    ramp_msgs::MotionState            configuration_; 
    geometry_msgs::Twist              velocity_;
    ramp_msgs::Trajectory             trajectory_;
    ros::Timer                        timer_;
    double                            initial_theta_;

    
    // static const members
    static const std::string  TOPIC_STR_PHIDGET_MOTOR;
    static const std::string  TOPIC_STR_ODOMETRY;
    static const std::string  TOPIC_STR_UPDATE;
    static const std::string  TOPIC_STR_TWIST;
    static const int          ACCELERATION_CONSTANT = 50;

  private:

    /** Methods **/

    void                        sendTwist() const;
    void                        calculateSpeedsAndTime();
    void                        printVectors() const;
    const bool                  checkImminentCollision() const;
    const bool                  checkOrientation(const int i) const;
    const ramp_msgs::Trajectory getRotationTrajectory() const;
    
    
    /** Data Members **/

    Utility                   utility_;
    bool                      restart_;
    int                       num;
    int                       num_traveled;
    const unsigned int        k_dof_;
    std::vector<ros::Time>    end_times; 
    std::vector<float>        speeds; 
    std::vector<float>        angular_speeds;
    std::vector<float>        orientations_;
    geometry_msgs::Twist      twist_;
    TrajectoryRequestHandler* h_traj_req_;
};

#endif
