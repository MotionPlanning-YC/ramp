/****************************************************************************
**
** Copyright (C) 2010 Nokia Corporation and/or its subsidiary(-ies).
** All rights reserved.
** Contact: Nokia Corporation (qt-info@nokia.com)
**
** This file is part of the examples of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** You may use this file under the terms of the BSD license as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of Nokia Corporation and its Subsidiary(-ies) nor
**     the names of its contributors may be used to endorse or promote
**     products derived from this software without specific prior written
**     permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
** $QT_END_LICENSE$
**
****************************************************************************/

#include "TrajectoryView.h"

#include <math.h>
#include <QtGui>
#include <stdio.h>

TrajectoryView::TrajectoryView(QWidget *parent)
    : QGraphicsView(parent)
{
    width_ = 240;
    height_ = 240;
    maxWidthMeters_ = 4.0f;
    maxHeightMeters_ = 4.0f;

    // Setup the scene
    QGraphicsScene *scene = new QGraphicsScene(this);
    scene->setItemIndexMethod(QGraphicsScene::NoIndex);
    scene->setSceneRect(0, 0, width_, height_);

    setScene(scene);
    setCacheMode(CacheBackground);
    setViewportUpdateMode(BoundingRectViewportUpdate);
    setRenderHint(QPainter::Antialiasing);
    setTransformationAnchor(AnchorUnderMouse);
    setWindowTitle(tr("Trajectory View"));


}




void TrajectoryView::size_changed()
// Change the scene size to the updated one when the user resizes the window
{

    width_ = this->parentWidget()->frameSize().width();
    height_ = this->parentWidget()->frameSize().height();
    std::cout<<"\nwidth: "<<width_;
    std::cout<<"\nheight: "<<height_<<"\n";

    this->resize(width_,height_);
    this->scene()->setSceneRect(0, -height_, width_, height_);// We need to make the scene a little smaller than the frame
}

void TrajectoryView::population(const ramp_msgs::Population& msg)
// Update the population and called the drawing function
{

  populations_.clear();
  populations_.push_back(msg);

  /*if(populations_.size() < 2) {
    populations_.push_back(msg);
  }

  else if(populations_.size() == 2) {
    if((unsigned int)msg.robot_id == 1) {
      populations_.erase(populations_.begin());
      populations_.insert(populations_.begin(), msg);
    }
    else if((unsigned int)msg.robot_id == 2) {
      populations_.erase(populations_.begin()+1);
      populations_.insert(populations_.begin()+1, msg);
    }
  }*/

  drawPopulation();
}

void TrajectoryView::drawPopulation()
// Draw the trajectories on the scene
{
    this->scene()->clear();


    QPen pen = QPen( QColor(0,0,255,150) ); // Black pen for the normal trajectories
    this->scene()->addLine(0, metersToPixels(3.5, false), width_-20, metersToPixels(3.5, false), pen);
    this->scene()->addLine(metersToPixels(3.5, true), 0, metersToPixels(3.5, true), metersToPixels(3.5, false), pen);
    
    pen = QPen( QColor(0,255,0,150) ); // Black pen for the normal trajectories
    this->scene()->addLine(0, metersToPixels(3, false), width_-20, metersToPixels(3, false), pen);
    this->scene()->addLine(metersToPixels(3, true), 0, metersToPixels(3, true), metersToPixels(3.5, false), pen);
    
    pen = QPen( QColor(255,0,0,150) ); // Black pen for the normal trajectories
    this->scene()->addLine(0, metersToPixels(2, false), width_-20, metersToPixels(2, false), pen);
    this->scene()->addLine(metersToPixels(2, true), 0, metersToPixels(2, true), metersToPixels(3.5, false), pen);
    
    pen = QPen( QColor(0,0,0,150) ); // Black pen for the normal trajectories
    this->scene()->addLine(0, metersToPixels(1, false), width_-20, metersToPixels(1, false), pen);
    this->scene()->addLine(metersToPixels(1, true), 0, metersToPixels(1, true), metersToPixels(3.5, false), pen);
    pen = QPen( QColor(0,0,0,150) ); // Black pen for the normal trajectories
    
    for(unsigned int p=0;p<populations_.size();p++) {

      // Blue for robot 2
      if(populations_.at(p).robot_id == 2) {
        pen = QPen( QColor(0,0,255,150) );
      }



      //go through all the trajectories
      //We go through the first one at last to make sure it is displayed on top, to make sure we see the red color if some trajectories are the same
      //for(int i = populations_.at(p).population.size() -1 ; i >=0 ; i--)
      //{
      int i = populations_.at(p).best_id;
          std::vector<trajectory_msgs::JointTrajectoryPoint> points = populations_.at(p).population.at(i).trajectory.points;

          if (i==populations_.at(p).best_id && populations_.at(p).robot_id == 1) {
             pen = QPen( QColor(255,0,0,255) ); // red for the best trajectory
          }
          else if (i==populations_.at(p).best_id && populations_.at(p).robot_id == 2) {
             pen = QPen( QColor(0,255,0,255) ); // green for the best trajectory
          }


          //go through all the waypoints of the trajectory
          for(int j = 0 ; j < (points.size() -1 ) ; j++)
          {
              this->scene()->addLine(metersToPixels(points.at(j).positions.at(0), true),
                             metersToPixels(points.at(j).positions.at(1), false),
                             metersToPixels(points.at(j+1).positions.at(0), true),
                             metersToPixels(points.at(j+1).positions.at(1), false),
                             pen);
              if(j == 0) { 
                
                std::vector<float> p;
                p.push_back(points.at(j).positions.at(0));
                p.push_back(points.at(j).positions.at(1));

                std::vector<float> c = getCenter(p, points.at(j).positions.at(2));
                //std::cout<<"\nc: ("<<c.at(0)<<", "<<c.at(1)<<")";

                this->scene()->addEllipse(metersToPixels(p.at(0), true),
                                        metersToPixels(p.at(1), false),
                                        metersToPixels(0.33f, true), metersToPixels(0.33f, false), pen);
                
              }

        }

        //} //end inner for
    } //end outter for
}

/** 
 * Assume a 45 degree angle is formed between the robot's center and the reference point (left wheel)
 * */
const std::vector<float> TrajectoryView::getCenter(std::vector<float> p, float orientation) const {
  std::vector<float> result;
 
  // Get world coordinates of reference point 
  float x = p.at(0);
  float y = p.at(1);

  // Radius
  float r = 0.2155261;

  // Get world coodinates of center point
  if(orientation > 0) {
    x += r*cos( u.displaceAngle((-3*PI/4), orientation));
    y += r*sin( u.displaceAngle((-3*PI/4), orientation));
  }
  else {
    x += r*cos( u.displaceAngle((-3*PI/4), -orientation));
    y += r*sin( u.displaceAngle((-3*PI/4), -orientation));
  }
  
  //std::cout<<"\nx: "<<p.at(0)<<" y: "<<p.at(1)<<" orientation: "<<orientation<<" Returning ("<<x<<", "<<y<<")";
  result.push_back(x);
  result.push_back(y);

  return result;
} //End getCenter




int const TrajectoryView::metersToPixels(float value, bool isWidth)
//Calculate the pixel value of a distance. If width is true, treat the value has a x position, if false treat it as a y position.
{
    if (isWidth)
    {
        if (value > maxWidthMeters_)
            maxWidthMeters_ = value;

        //return value * width_ / (maxWidthMeters_ - 1);
        return value * width_ / (maxWidthMeters_);
    }

    if (!isWidth)
    {
        if (value > maxHeightMeters_)
            maxHeightMeters_ = value;
        value *= -1;

        //return value * height_ / (maxHeightMeters_ - 1);
        return value * height_ / (maxHeightMeters_);
    }
}


