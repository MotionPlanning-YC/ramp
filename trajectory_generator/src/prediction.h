#ifndef PREDICTION_H
#define PREDICTION_H
#include "line.h"
#include "circle.h"
#include "utility.h"


#define CYCLE_TIME_IN_SECONDS 0.1


class Prediction {
public:
  Prediction();
  ~Prediction();

  // Service callback, the input is a path and the output a trajectory
  bool trajectoryRequest(ramp_msgs::TrajectoryRequest::Request& req,ramp_msgs::TrajectoryRequest::Response& res);
private:

  ramp_msgs::Path path_;

  void init(const ramp_msgs::TrajectoryRequest::Request req);

  Utility utility_;
};

#endif
