#include "modifier.h"


Modifier::Modifier(const ros::NodeHandle& h, const unsigned int n) : num_ops(n), i_changed1(-1), i_changed2(-1) {
  h_mod_req_ = new ModificationRequestHandler(h);
}

Modifier::Modifier(const ros::NodeHandle& h, const std::vector<Path> ps, const std::vector< std::vector<float> > vs, const unsigned int n) : num_ops(n), paths_(ps), velocities_(vs), i_changed1(-1), i_changed2(-1) {
  h_mod_req_ = new ModificationRequestHandler(h);
}

Modifier::~Modifier() {
  if(h_mod_req_ != 0) {
    delete h_mod_req_;
    h_mod_req_ = 0;
  }
}


void Modifier::updateAll(std::vector<Path> ps, std::vector< std::vector<float> > vs) {
  paths_.clear();
  velocities_.clear();
  
  for(unsigned int i=0;i<ps.size()-1;i++) {
    paths_.push_back(ps.at(i));
    velocities_.push_back(vs.at(i));
  }

  paths_.push_back(ps.at(ps.size()-1));

}

void Modifier::update(const Path p, const unsigned int i) {
  paths_.at(i) = p;
}


/** This method returns a random operator */
const std::string Modifier::getOperator() const {
  std::string result;  
  
  // First, randomly select an operator
  unsigned int op = rand() % num_ops;

  // Assign the correct name for the operator
  switch(op) {

    // Insert
    case 0:
      result = "insert";
      break;

    // Delete
    case 1:
      result = "delete";
      break;

    // Change
    case 2:
      result = "change";
      break;

    // Swap
    case 3:
      result = "swap";
      break;

    // Crossover
    case 4:
      result = "crossover";
      break;

    // Stop
    case 5:
      result = "stop";
      break;
  }

  return result;
} // End getOperator


/** This method generates the random paths to use for the modification operator passed in as op argument */
const std::vector<int> Modifier::getTargets(const std::string op) {
  std::vector<int> result;

  // Get random path(s) to modify
  unsigned int i_p1;
  i_p1 = rand() % paths_.size();

  // Set i_changed1
  i_changed1 = i_p1;
  
  // Push on i_p1
  result.push_back(i_p1);

  // If crossover, get a second path
  if(op == "crossover") {
    unsigned int i_p2;
    do { i_p2 = rand() % paths_.size(); } 
    while (i_p1 == i_p2);

    // Set i_changed2
    i_changed2 = i_p2;
  
    // Push on i_p1
    result.push_back(i_p2);
  } // end if crossover 
  // Else, no second path  
  else {i_changed2 = -1;}


  return result;
} // End getTargets


/** 
 * This method builds a ModificationRequest srv 
 * For stop operator, the path can be retreived from srv
 * */
const ramp_msgs::ModificationRequest Modifier::buildModificationRequest() {
  ramp_msgs::ModificationRequest result;

  result.request.op = getOperator();
  //std::cout<<"\nOperator: "<<result.request.op<<"\n";

  // Push the target paths onto the modification request
  std::vector<int> targets = getTargets(result.request.op);
  for(unsigned int i=0;i<targets.size();i++) {
    result.request.paths.push_back(paths_.at(targets.at(i)).buildPathMsg());
  }

  return result;
} // End buildModificationRequest



const Path Modifier::stop(Path p) {
  // p was the random path chosen
  
  // Randomly choose a point in p
  int i_point = rand() % (p.size()-1);

  // Maximum of 4 seconds to stop
  int time = (rand() % 4) + 1;

  p.all_.at(i_point).stop_time_ = time;

  // Push on the values to stop_points and stop_times
  //p.stop_points_.push_back(i_point);
  //p.stop_times_.push_back(time);

  return p;
}



/** This method performs all the tasks for path modification */
const std::vector<Path> Modifier::perform() {
  std::vector<Path> result;
 
  // Build a modification request srv 
  ramp_msgs::ModificationRequest mr = buildModificationRequest(); 

  // Check if the operation changes the path
  if(mr.request.op == "stop") {
    // Call stop with the path chosen by buildModificationRequest
    Path temp = stop(mr.request.paths.at(0));
    result.push_back(temp);
  }

  else {

    // If the request was successful
    if(h_mod_req_->request(mr)) {

      // Push on the modified paths
      for(unsigned int i=0;i<mr.response.mod_paths.size();i++) {
        Path temp(mr.response.mod_paths.at(i));
        result.push_back(temp);
      }
    }
  } // end if operator != stop


  return result;
}
