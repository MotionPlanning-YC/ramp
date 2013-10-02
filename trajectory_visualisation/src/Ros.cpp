#include "Ros.h"
#include <QImage>
#include <QGraphicsTextItem>
#include <QPainter>
#include <QGraphicsView>
#include <math.h>
#include <QImageReader>

#include <vector>

using namespace std;

Ros::Ros(){
    initialized = false;
}


Ros::~Ros(){

}

void Ros::subscribe()
//function that initialize every ros variables declared

{
    ros::NodeHandle n;
    ros::NodeHandle nh("~");


    //Advertise topics


    //Subscribe to topics
    popSub_= n.subscribe("population",1000, &Ros::populationCallback,this);

    initialized = true;
}

void Ros::init(int argc, char *argv[]){
    ros::init(argc, argv,"trajectory_view");

    qRegisterMetaType<ramp_msgs::Population>("ramp_msgs::Population");

    this->subscribe();
    this->start();
}

void Ros::run(){
    ros::spin();
}

void Ros::populationCallback(const ramp_msgs::Population& msg)
{
    emit population(msg);
}
