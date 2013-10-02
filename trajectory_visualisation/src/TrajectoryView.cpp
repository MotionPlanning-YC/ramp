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
    width_ = 640;
    height_ = 480;
    maxWidthMeters_ = 0;
    maxHeightMeters_ = 0;

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

    this->resize(width_,height_);
    this->scene()->setSceneRect(0, 0, width_-10, height_-10);// We need to make the scene a little smaller than the frame
}

void TrajectoryView::population(const ramp_msgs::Population& msg)
// Update the population and called the drawing function
{
    population_ = msg;

    drawPopulation();
}

void TrajectoryView::drawPopulation()
// Draw the trajectories on the scene
{
    this->scene()->clear();

    QPen pen = QPen( QColor(0,0,0,150) ); // Black pen for the normal trajectories
    for(int i = population_.population.size() -1 ; i >=0 ; i--)
        //go through all the trajectories
    {
        std::vector<trajectory_msgs::JointTrajectoryPoint> points = population_.population.at(i).trajectory.points;

        if (i==0)
           pen = QPen( QColor(255,0,0,255) ); // red for the best trajectory
        for(int j = 0 ; j < (points.size() -1 ) ; j++)
            //go through all the waypoints of the trajectory
            // We go through the first one at last to make sure it is displayed on top, to make sure we see the red color if some trajectories are the same
        {
            this->scene()->addLine(metersToPixels(points.at(j).positions.at(0), true),
                           metersToPixels(points.at(j).positions.at(1), false),
                           metersToPixels(points.at(j+1).positions.at(0), true),
                           metersToPixels(points.at(j+1).positions.at(1), false),
                           pen);

        }

    }
}

int const TrajectoryView::metersToPixels(const float value, bool isWidth)
//Calculate the pixel value of a distance. If width is true, treat the value has a x position, if false treat it as a y position.
{
    if (isWidth)
    {
        if (value > maxWidthMeters_)
            maxWidthMeters_ = value;

        return value * width_ / (maxWidthMeters_ - 1);
    }

    if (!isWidth)
    {
        if (value > maxHeightMeters_)
            maxHeightMeters_ = value;

        return value * height_ / (maxHeightMeters_ - 1);
    }
}
