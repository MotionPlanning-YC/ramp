#include "modifier.h"


Modifier::Modifier(const ros::NodeHandle& h, const unsigned int n) : num_ops(n) {
  h_mod_req_ = new ModificationRequestHandler(h);
}

Modifier::~Modifier() {
  if(h_mod_req_ != 0) {
    delete h_mod_req_;
    h_mod_req_ = 0;
  }
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
const std::vector<int> Modifier::getTargets(const std::string op, const Population pop) {
  //std::cout<<"\nIn targets\n";
  std::vector<int> result;

  // Get random path(s) to modify
  unsigned int i_p1 = rand() % pop.paths_.size();
  
  // Push on i_p1
  result.push_back(i_p1);


  // If crossover, get a second path
  if(op == "crossover") {
    unsigned int i_p2;
    do { i_p2 = rand() % pop.paths_.size(); } 
    while (i_p1 == i_p2);
  
    // Push on i_p1
    result.push_back(i_p2);
  } // end if crossover 


  //std::cout<<"\nLeaving targets\n";
  return result;
} // End getTargets


/** 
 * This method builds a ModificationRequest srv 
 * For stop operator, the path can be retreived from srv
 * */
const ramp_msgs::ModificationRequest Modifier::buildModificationRequest(const Population pop) {
  ramp_msgs::ModificationRequest result;

  result.request.op = getOperator();

  // Push the target paths onto the modification request
  std::vector<int> targets = getTargets(result.request.op, pop);
  for(unsigned int i=0;i<targets.size();i++) {
    //std::cout<<"\nTarget index: "<<targets.at(i);
    //std::cout<<"\nPath "<<targets.at(i)<<" size: "<<pop.paths_.at(targets.at(i)).size()<<"\n";
    result.request.paths.push_back(
        pop.paths_.at(targets.at(i)).buildPathMsg());
  }

  return result;
} // End buildModificationRequest




/** This method performs all the tasks for path modification */
const std::vector<Path> Modifier::perform(const Population pop) {
  std::vector<Path> result;
 
  // Build a modification request srv 
  ramp_msgs::ModificationRequest mr = buildModificationRequest(pop); 
  

  // Check if the operation changes the path
  if(mr.request.op == "stop") {
    // Call stop with the path chosen by buildModificationRequest
    //Path temp = stop(mr.request.paths.at(0));
    //result.push_back(temp);
  }

  else {
    std::cout<<"\nIn else\n";

    // If the request was successful
    if(h_mod_req_->request(mr)) {

      // Push on the modified paths
      for(unsigned int i=0;i<mr.response.mod_paths.size();i++) {
        Path temp(mr.response.mod_paths.at(i));
        result.push_back(temp);
      }
    } // end inner if 
  } // end if operator != stop

  std::cout<<"\nLeaving perform\n";
  return result;
}
