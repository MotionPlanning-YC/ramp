#include "modifier.h"


Modifier::Modifier(const ros::NodeHandle& h) : num_ops(5), i_changed1(-1), i_changed2(-1) {
  h_mod_req_ = new ModificationRequestHandler(h);
}

Modifier::Modifier(const ros::NodeHandle& h, const std::vector<Path> ps, const std::vector< std::vector<float> > vs) : num_ops(5), paths_(ps), velocities_(vs), i_changed1(-1), i_changed2(-1) {
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

/** This method builds a ModificationRequest srv */
const ramp_msgs::ModificationRequest Modifier::buildModificationRequest() {
  ramp_msgs::ModificationRequest result;

  // First, randomly select an operator
  unsigned int op = rand() % num_ops;

  // Assign the correct name for the operator
  switch(op) {

    // Insert
    case 0:
      result.request.op = "insert";
      break;

    // Delete
    case 1:
      result.request.op = "delete";
      break;

    // Change
    case 2:
      result.request.op = "change";
      break;

    // Swap
    case 3:
      result.request.op = "swap";
      break;

    // Crossover
    case 4:
      result.request.op = "crossover";
      break;
  }

  // Get random path(s) to modify
  unsigned int i_p1;
  i_p1 = rand() % paths_.size();
  // std::cout<<"\ni_p1: "<<i_p1;

  // Push the path to change onto the result
  result.request.paths.push_back(paths_.at(i_p1).buildPathMsg());

  // Set i_changed1
  i_changed1 = i_p1;

  // If crossover, get a second path
  if(op == 4) {
    unsigned int i_p2;
    do { i_p2 = rand() % paths_.size(); } 
    while (i_p1 == i_p2);

    // push the path to change onto the result
    result.request.paths.push_back(paths_.at(i_p2).buildPathMsg());

    // Set i_changed2
    i_changed2 = i_p2;

    // std::cout<<"\ni_p2: "<<i_p2;
  }
  else {
    i_changed2 = -1;
  }

  return result;
}






/** This method performs all the tasks for path modification */
const std::vector<Path> Modifier::perform() {
  std::vector<Path> result;
 
  // Build a modification request srv 
  ramp_msgs::ModificationRequest mr = buildModificationRequest(); 

  // If the request was successful
  if(h_mod_req_->request(mr)) {

    // Push on the modified paths
    for(unsigned int i=0;i<mr.response.mod_paths.size();i++) {
      Path temp(mr.response.mod_paths.at(i));
      result.push_back(temp);
    }
  }
  else {
    // some error handling
  }

  return result;
}
