#ifndef ROS_H
#define ROS_H

#include <QThread>

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "ramp_msgs/Population.h"

#include <QGraphicsScene>


class Ros : public QThread {

    Q_OBJECT

private:
    void subscribe();

    bool initialized;
    ros::Subscriber popSub_;


signals :
    void population(const ramp_msgs::Population&);

public slots:

public:
    ~Ros();
    Ros();

	void run();
    void init(int argc, char *argv[]);
    void populationCallback(const ramp_msgs::Population& msg);
};


#endif // ROS_H
