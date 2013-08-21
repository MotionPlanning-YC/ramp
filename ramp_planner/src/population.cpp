#include "population.h"


Population::Population() : max_size(7), i_best(0) {}

Population::Population(const unsigned int size) : max_size(size), i_best(0) {}

const RampTrajectory Population::getBest() const {return population_.at(i_best);}


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
}

/** This method should call the evaluate procedure for all of the trajectories
 *  It also finds the best trajectory and sets i_best */
const RampTrajectory Population::evaluateAndObtainBest() {
  i_best = 0;
}


/** toString */
const std::string Population::toString() const {
  std::ostringstream result;
  for(unsigned int i=0;i<population_.size();i++) {
    result<<"\n"<<population_.at(i).toString();
  }
  return result.str();
}


