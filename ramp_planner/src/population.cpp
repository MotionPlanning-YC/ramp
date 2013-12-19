#include "population.h"


Population::Population() : max_size(4), i_best(-1) {}

Population::Population(const unsigned int size) : max_size(size), i_best(-1) {}


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

    //Generate a random index for a random trajectory to remove
    //Don't pick the fittest trajectory!!
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


/**  
 * This method returns the index of the most fit feasible/infeasible trajectory 
 * feasible = true if looking for feasible trajectory, false for infeasible
 * Returns -1 if no trajectories of that type exist in the population
 */
const int Population::findBestFeasible(bool feasible, int generation) const {
  int result = -1;
  int result_second = result;

  //Find the first feasible(or infeasible) trajectory
  int i_start    = 0;
  while(i_start < population_.size() && population_.at(i_start).feasible_ != feasible) {i_start++;}

  //If one of the trajectories was desired feasibility
  if(i_start < population_.size()) {
    
    //Set result
    result = i_start;
    result_second = result;

    //Go through each trajectory and see it is highest fitness value
    for(unsigned int i=i_start;i<population_.size();i++) {
      
      //If the trajectory feasibility == feasible, and it has the highest fitness so far
      if(population_.at(i).feasible_ == feasible && population_.at(i).fitness_ > population_.at(result).fitness_) {
        result_second = result;
        result = i;
      }

      else if(population_.at(i).feasible_ == feasible && population_.at(i).fitness_ > population_.at(result_second).fitness_) {
        result_second = i;
      }
    }
  }

  //if(generation > 75) {
    //std::cout<<"\nReturning "<<result_second<<" for feasible: "<<feasible<<"\n";
    //return result_second;
  //}
  
  return result;
} //End findBestFeasible



/** Returns the fittest trajectory and sets i_best */
const RampTrajectory Population::findBest(int generation) {
  //std::cout<<"\nIn findBest\n";

  //Find both the best feasible and infeasible trajectories
  int i_bestF   = findBestFeasible(true, generation);
  int i_bestInf = findBestFeasible(false, generation);
  //std::cout<<"\ni_bestF: "<<i_bestF<<"\n";
  //std::cout<<"\ni_bestInf: "<<i_bestInf<<"\n";
  

  //Set i_best
  if(i_bestF < 0)
    i_best = i_bestInf;
  else
    i_best = i_bestF;

  //std::cout<<"\ni_best: "<<i_best<<"\n";
  return population_.at(i_best); 
} //End getBest 



/** fitness and feasible toString */
const std::string Population::fitnessFeasibleToString() const {
  std::ostringstream result;

  for(unsigned int i=0;i<population_.size();i++) {
    result<<"\n"<<population_.at(i).fitnessFeasibleToString();
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

//Return a message of type ramp_msgs::Population to be sent to the trajectory viewer 
ramp_msgs::Population Population::populationMsg()
{
    ramp_msgs::Population msg;
    
    msg.population.push_back(population_.at(i_best).msg_trajec_);
    for( int i =0; i<population_.size(); i++)
    {
        if (i != i_best)
            msg.population.push_back(population_.at(i).msg_trajec_);
    }
    return msg;
}
