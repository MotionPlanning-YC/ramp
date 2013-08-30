#include "population.h"


Population::Population() : max_size(7), i_best(0) {}

Population::Population(const unsigned int size) : max_size(size), i_best(0) {}


/** This method adds a trajectory to the population. 
 *  If the population is full, a random trajectory (that isn't the best one) is replaced */
void Population::add(const RampTrajectory rt) {
 
  //If not full, simply push back
  if(population_.size() < max_size) {
    population_.push_back(rt);  
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
    population_.push_back(rt);
  }
} //End add



/** Return the fittest trajectory */
const RampTrajectory Population::getBest() const {
  
  //Find first feasible trajectory
  int i_max = 0;
  while(i_max < population_.size() && !population_.at(i_max).feasible_) {
    i_max++;
  }
  
  //Find the maximum fitness of all feasible trajectories
  for(unsigned int i=i_max;i<population_.size();i++) {
    if( population_.at(i).feasible_ &&
        population_.at(i).fitness_ > population_.at(i_max).fitness_) 
    {
      i_max = i; 
    }
  }

  return population_.at(i_max);
} //End getBest


/** toString */
const std::string Population::toString() const {
  std::ostringstream result;

  for(unsigned int i=0;i<population_.size();i++) {
    result<<"\nTrajectory "<<i<<": "<<population_.at(i).toString();
  }

  return result.str();
} //End toString
