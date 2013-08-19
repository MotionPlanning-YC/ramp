#include "population.h"


Population::Population() : max_size(7), i_best(0) {}

Population::Population(const unsigned int size) : max_size(size), i_best(0) {}

const RampTrajectory Population::getBest() const {return population_.at(i_best);}

void Population::add(const RampTrajectory rt) {
 
  if(population_.size() < max_size) {
    population_.push_back(rt);  
  } 
  else {

    //Pick a random id to remove
    //Don't pick the id of the fittest trajectory!!
    unsigned int i;
    do {i = rand() % max_size;}
    while(i == i_best);
  
    population_.erase(population_.begin()+i);

    //Push back the new trajectory
    population_.push_back(rt);
  }
}


const std::string Population::toString() const {
  std::ostringstream result;
  for(unsigned int i=0;i<population_.size();i++) {
    result<<"\n"<<population_.at(i).toString();
  }
  return result.str();
}


const RampTrajectory Population::evaluateAndObtainBest() {
  i_best = 0;
}
