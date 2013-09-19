#include "population.h"


Population::Population() : max_size(7), i_best(0) {}

Population::Population(const unsigned int size) : max_size(size), i_best(0) {}


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
  if(population_.size() < max_size) {
    population_.push_back(rt);  
    return population_.size()-1;
  } 

  //If full, replace a trajectory
  else {

    //Pick a random id to remove
    //Don't pick the id of the fittest trajectory!!
    unsigned int i;
    do {i = rand() % max_size;}
    while(i == i_best);
  
    //Remove the random trajectory
    population_.erase(population_.begin()+i);

    //Push back the new trajectory
    population_.insert(population_.begin()+i, rt);

    return i;
  }
} //End add



/** Return the fittest trajectory */
//TODO: If all trajectories are infeasible, pick the fittest infeasible
const RampTrajectory Population::findBest() {
  
  //Find first feasible trajectory
  int i_max = 0;
  while(i_max < population_.size() && !population_.at(i_max).feasible_) {
    i_max++;
  }
  
  if(i_max == population_.size()) {
    std::cout<<"\nNo feasible trajectories!\n";
  }
  //Find the maximum fitness of all feasible trajectories
  for(unsigned int i=i_max;i<population_.size();i++) {
    if( population_.at(i).feasible_ &&
        population_.at(i).fitness_ > population_.at(i_max).fitness_) 
    {
      i_max = i; 
    }
  }

  //Set i_best
  i_best = i_max;
  
  return population_.at(i_best);
} //End getBest



/** fitness and feasible toString */
const std::string Population::fitnessFeasibleToString() const {
  std::ostringstream result;

  for(unsigned int i=0;i<population_.size();i++) {
    result<<"\nTrajectory "<<i<<": "<<population_.at(i).fitnessFeasibleToString();
  }

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
