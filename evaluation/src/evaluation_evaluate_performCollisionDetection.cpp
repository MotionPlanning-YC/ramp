

const bool Evaluate::performCollisionDetection() {
  
  // Go through all the knot points in the trajectory
  for (int i = 0; i<= trajectory_.index_knot_points.size() -2 ; i++)
  {
    trajectory_msgs::JointTrajectoryPoint p1 = trajectory_.trajectory.points[trajectory_.index_knot_points.at(i)];  
    trajectory_msgs::JointTrajectoryPoint p2 = trajectory_.trajectory.points[trajectory_.index_knot_points.at(i+1) ];  

    //get slope
    float a = (p2.positions.at(0) - p1.positions.at(0)) / (p2.positions.at(1) - p1.positions.at(1));

    float c = p2.positions.at(0) - (a*p2.positions.at(1));

    float x3 = (a*objList_.ir_object.y1) + c;
    float x4 = (a*objList_.ir_object.y2) + c;

    // switch y3 and y4 if they are inverted
    if(x3 < x4) {
      float temp = x3;
      x3 = x4;
      x4 = temp;
    }
    

    std::cout << " a: " << a << " c: " << c << " y1: " << objList_.ir_object.y1 << " y2: " << objList_.ir_object.y2 << " x3: " << x3 << " x4: " << x4 << " x1: " << objList_.ir_object.x1 << " x2: " << objList_.ir_object.x2 << "\n";


    //Check that y1 or y2 are between y3 and y4. If so there is collision
    if( (objList_.ir_object.x1 <= x3 && objList_.ir_object.x1 >= x4) ||
        (objList_.ir_object.x2 <= x3 && objList_.ir_object.x2 >= x4) ) {
          return true; 
      }
  }
  return false;
}
