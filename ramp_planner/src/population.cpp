#include "population.h"

Population::Population() : maxSize_(5), i_best_(-1) {}

Population::Population(const unsigned int size) : maxSize_(size), i_best_(-1) {}


/** Return the size of the population */
const unsigned int Population::size() const { return population_.size(); }

void Population::clear() { population_.clear(); }

const bool Population::replaceAll(const std::vector<RampTrajectory> new_pop) {
  if(new_pop.size() == population_.size()) {
    population_ = new_pop;
    return true;
  }
  
  return false;
}

/** This method adds a trajectory to the population. 
 *  If the population is full, a random trajectory (that isn't the best one) is replaced.
 *  Returns the index that the trajectory is added at */
const unsigned int Population::add(const RampTrajectory rt) {
 
  //If not full, simply push back
  if(population_.size() < maxSize_) {
    population_.push_back(rt);  
    return population_.size()-1;
  } 

  //If full, replace a trajectory
  else {

    //Generate a random index for a random trajectory to remove
    //Don't pick the fittest trajectory!!
    unsigned int i;
    do {i = rand() % maxSize_;}
    while(i == i_best_);
  
    //Remove the random trajectory
    population_.erase(population_.begin()+i);

    //Push back the new trajectory
    population_.insert(population_.begin()+i, rt);

    return i;
  }
} //End add



const bool Population::checkIfChange() const {
  return i_best_ != i_best_prev_;
}



/** Returns the fittest trajectory and sets i_best_ */
const RampTrajectory Population::findBest() {
  //std::cout<<"\nIn findBest\n";
  
  // Find the index of the trajectory with the highest fitness value
  unsigned int i_max = 0;
  for(unsigned int i=1;i<population_.size();i++) {
    if(population_.at(i).fitness_ > population_.at(i_max).fitness_) {
      i_max = i;
    }
  } //end for


  // Set i_best_
  i_best_prev_ = i_best_;
  i_best_ = i_max;

  return population_.at(i_best_); 
} //End getBest 



/** fitness and feasible toString */
const std::string Population::fitnessFeasibleToString() const {
  std::ostringstream result;

  result<<"\n****************************************************";
  result<<"\nPopulation's fitness and feasibility:";
  for(unsigned int i=0;i<population_.size();i++) {
    result<<"\n"<<population_.at(i).fitnessFeasibleToString();
    if(i == i_best_) {
      result<<" - Best!";
    }
  }
  result<<"\n****************************************************";

  return result.str();
} //End toString


/** toString */
const std::string Population::toString() const {
  std::ostringstream result;

  for(unsigned int i=0;i<population_.size();i++) {
    result<<"\nTrajectory "<<i<<": "<<population_.at(i).toString();
  }

  return result.str();
} //End toString

//Return a message of type ramp_msgs::Population to be sent to the trajectory viewer 
ramp_msgs::Population Population::populationMsg()
{
    ramp_msgs::Population msg;
    
    for( int i =0; i<population_.size(); i++) {
      msg.population.push_back(population_.at(i).msg_trajec_);
    }
    
    msg.best_id = i_best_;
    return msg;
}
