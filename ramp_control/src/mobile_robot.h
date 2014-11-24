
#ifndef MOBILE_ROBOT_H
#define MOBILE_ROBOT_H

#include "ros/ros.h"
#include "utility.h"
#include "trajectory_request_handler.h"
#include "nav_msgs/Odometry.h"
#include "corobot_msgs/MotorCommand.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_datatypes.h"
#include "ramp_msgs/MotionState.h"
#include <math.h>

class MobileRobot {
  public:
    
    MobileRobot();
    ~MobileRobot();
    
    /** Methods **/ 
    void init(ros::NodeHandle& h);

    void drive(corobot_msgs::MotorCommand msg) const;
    void driveStraight(const unsigned int speed) const;
    void turn(const unsigned int speed, const bool cwise) const;
    void turn(const float speed, const float angle) const; 
    void stop() const;

    void moveOnTrajectory(bool simulation);
    void moveOnTrajectoryRot(const ramp_msgs::RampTrajectory traj, bool simulation);
    void updateState(const nav_msgs::Odometry& msg);
    void updateTrajectory(const ramp_msgs::RampTrajectory msg); 
    void updatePublishTimer(const ros::TimerEvent&);
    void sendTwist(const geometry_msgs::Twist twist) const;
    void controlCycle(geometry_msgs::Twist twist, ros::Time end_time, ros::Rate r);



    /** Data Members **/
    ros::Publisher                    pub_phidget_motor_;
    ros::Publisher                    pub_twist_;
    ros::Publisher                    pub_cmd_vel_;
    ros::Publisher                    pub_update_;
    ros::Subscriber                   sub_odometry_;
    ramp_msgs::MotionState            motion_state_; 
    geometry_msgs::Twist              velocity_;
    ramp_msgs::RampTrajectory         trajectory_;
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
    const std::vector<double>   computeAcceleration() const;
    
    
    /** Data Members **/

    Utility                   utility_;
    bool                      restart_;
    int                       num_;
    int                       num_prev_;
    int                       num_traveled_;
    const unsigned int        k_dof_;
    std::vector<ros::Time>    end_times; 
    std::vector<double>       speeds_linear_;
    std::vector<double>       speeds_angular_;

    geometry_msgs::Twist      twist_;
    TrajectoryRequestHandler* h_traj_req_;
    ramp_msgs::MotionState    prev_motion_state_; 
    ros::Time                 prev_t_;
    ros::Duration             t_immiColl_;
};

#endif
