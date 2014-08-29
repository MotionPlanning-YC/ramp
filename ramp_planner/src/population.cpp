#include "population.h"

Population::Population() : maxSize_(3), i_best_(-1), changed_(true), isSubPopulation_(false) {}

Population::Population(const unsigned int size, const bool isSubPop) : maxSize_(size), i_best_(-1), changed_(true), isSubPopulation_(isSubPop) {}


/** Return the size of the population */
const unsigned int Population::size() const { return trajectories_.size(); }

void Population::clear() { 
  trajectories_.clear(); 
  paths_.clear();
  changed_ = true;
}


/** This method returns the trajectory at index i */
const RampTrajectory Population::get(const unsigned int i) {
  return trajectories_.at(i);
} // End get





void Population::replace(const uint8_t i, const RampTrajectory trajec) {
  if(i < trajectories_.size()) {
    trajectories_.at(i) = trajec;
    paths_.at(i) = trajec.path_;
  }
  else {
    ROS_WARN("Replacing trajectory at index %i, but population size = %lu\n", (int)i, trajectories_.size());
    trajectories_.push_back(trajec);
    paths_.push_back(trajec.path_);  
  }
  
  changed_ = true;
}



void Population::replaceAll(const std::vector<RampTrajectory> new_pop) {
  
  // Set trajectories vector
  trajectories_ = new_pop;

  // Set paths vector
  paths_.clear();
  for(uint8_t i=0;i<new_pop.size();i++) {
    paths_.push_back(new_pop.at(i).path_);
  }

  changed_ = true;
} // End replaceAll





const int Population::getNumSubPops() const {
  return subPopulations_.size();
}





/** This method returns the minimum fitness of the population */
const int Population::getMinFitness() const {
  int result = trajectories_.at(0).msg_.fitness;

  for(uint8_t i=1;i<trajectories_.size();i++) {
    if(trajectories_.at(i).msg_.fitness < result) {
      result = trajectories_.at(i).msg_.fitness;
    }
  }
  
  return result;
} // End getMinFitness








/** This method returns true if there is at least one feasible trajectory in the population */
const bool Population::feasibleExists() const {
  for(uint8_t i=0;i<trajectories_.size();i++) {
    if(trajectories_.at(i).msg_.feasible) {
      return true;
    }
  }

  return false;
} // End feasibleExists



/** This method returns true if there is at least one infeasible trajectory in the population */
const bool Population::infeasibleExists() const {
  for(uint8_t i=0;i<trajectories_.size();i++) {
    if(!trajectories_.at(i).msg_.feasible) {
      return true;
    }
  }

  return false;
} // End infeasibleExists








/** This method checks if a trajectory can replace an existing trajectory in the population */
const bool Population::replacementPossible(const RampTrajectory rt) const {
  //std::cout<<"\n=========In replacementPossible==========\n";

  // If the fitness is not higher than the minimum fitness
  if(rt.msg_.fitness <= getMinFitness()) {
    //std::cout<<"\nfitness < min fitness, not possible\n";
    return false;
  }
  
  // If the trajectory is infeasible and
  // no infeasible trajectories exist, no
  // trajectories can be replaced
  if(!rt.msg_.feasible && !infeasibleExists()) {
    //std::cout<<"\nrt infeasible, no other infeasible, not possible\n";
    return false;
  }

  /** IF subpopulations are being used */
  if(subPopulations_.size() > 0) {
    //std::cout<<"\nIn sub-pops\n";
    
    // If each subpopulation has <= 1 trajectory,
    // no trajectories can be replaced
    std::vector<uint8_t> i_validSubpops;
    for(uint8_t i=0;i<subPopulations_.size();i++) {
      if(subPopulations_.at(i).size() > 1) {
        i_validSubpops.push_back(i);
      }
    }
    if(i_validSubpops.size() == 0) {
      //std::cout<<"\nAll sub-pops size < 2, not possible\n";
      return false;
    }
    else {
      //std::cout<<"\ni_validSubPops.size(): "<<i_validSubpops.size()<<"\n";
    }
  
    // If the valid sub-populations have only feasible trajectories
    // no trajectories can be replaced
    // or if rt's fitness is lower than the min fitness of sub-population
    if(!rt.msg_.feasible) {
      //std::cout<<"\nIn infeasible\n";
      bool valid=false;
      for(uint8_t i=0;i<i_validSubpops.size();i++) {
        if(subPopulations_.at(i_validSubpops.at(i)).infeasibleExists() &&
            rt.msg_.fitness > subPopulations_.at(i_validSubpops.at(i)).getMinFitness())
        {
          valid = true;
        }
      }

      if(!valid) {
        //std::cout<<"\nnot valid, not possible\n";
        return false;
      }
    } // end if rt is infeasible
  } // end if sub-populations are used

  //std::cout<<"\nreplacementPossible returning true\n";
  return true;
} // End replacementPossible






/** This method returns true if rt can replace the trajectory at index i */
const bool Population::canReplace(const RampTrajectory rt, const int i) const {
  //std::cout<<"\n"<<toString();
  //std::cout<<"\n----------In canReplace, i: "<<i<<", trajectory: "<<trajectories_.at(i).id_<<"----------\n";
  
  if(i == i_best_) {
    //std::cout<<"\ni == i_best, returning false\n";
    return false;
  }

  if(!rt.msg_.feasible && trajectories_.at(i).msg_.feasible) {
    //std::cout<<"\nrt infeasible, i feasible, returning false\n";
    return false;
  }


  // If sub-populations are used,
  if(subPopulations_.size() > 0) {
    
    //std::cout<<"\ntrajectories.size(): "<<trajectories_.size();
    RampTrajectory temp = trajectories_.at(i);

    //std::cout<<"\nsubPopulations.size(): "<<subPopulations_.size();
    //std::cout<<"\ntemp.i_subPopulation: "<<temp.msg_.i_subPopulation<<"\n";
    Population p = subPopulations_.at(temp.msg_.i_subPopulation);

    if(p.trajectories_.size() < 2) {
      //std::cout<<"\nSub-Population size < 2, returning false\n";
      return false;
    }

    //std::cout<<"\np.trajectories.size(): "<<p.trajectories_.size();
    //std::cout<<"\np.i_best_: "<<p.i_best_<<"\n";

    // if i is the best in trajectory i's sub-population
    if(temp.equal( p.trajectories_.at(p.i_best_) )) {
      ///std::cout<<"\ntemp == best in sub-population, returning false\n";
      return false;
    }
  }
  
  //std::cout<<"\nreturning true\n";
  return true;
} // End canReplace







/** This method determines which trajectory (if any) in the
* population will be replaced if the population is full when
* adding a trajectory to it
* A result of -1 means no trajectories could be removed
* - Will happen if rt is infeasible and the rest are feasible
* - or if each sub-population has <= 1 trajectory */
const int Population::getReplacementID(const RampTrajectory rt) const {
  //std::cout<<"\nIn getReplacementID\n";
  //std::cout<<"\n"<<toString();
  
  // If the trajectory is infeasible and
  // no infeasible trajectories exist, no
  // trajectories can be replaced
  int result;
  
  // Generate a random index for a random trajectory to remove
  do {result = rand() % trajectories_.size();}
  
  // Keep getting a random index until it
  // is an index that rt can replace
  while(!canReplace(rt, result));

  
  return result;
} // End getReplacementID






/** This method adds a trajectory to the population. 
 *  If the population is full, a random trajectory (that isn't the best one) is replaced
 *  Returns the index that the trajectory is added at */
const int Population::add(const RampTrajectory rt) {
  changed_ = true;


  if(subPopulations_.size() > 0) {
    // Go through each sub-population and find best
    for(uint8_t i=0;i<subPopulations_.size();i++) {
      subPopulations_.at(i).getBestID();
    }
  }
 
  // If it's a sub-population or
  // If it's not full, simply push back
  if(isSubPopulation_ || trajectories_.size() < maxSize_) {
    trajectories_.push_back (rt);  
    paths_.push_back        (rt.path_);
    return trajectories_.size()-1;
  } 

  // If full, replace a trajectory
  else if(replacementPossible(rt)) {


    int i = getReplacementID(rt);

    replace(i, rt);
    return i;
  }


  //std::cout<<"\nAdd not possible, returning -1\n";
  changed_ = false;
  return -1;
} //End add



/** Returns the fittest trajectory and sets i_best_ */
const int Population::getBestID() {
  //std::cout<<"\nIn findBest\n";
  
  // If population has not changed since last
  // findBest call, return without searching
  if(!changed_) {
    return i_best_;
  }
  
  // Find the index of the trajectory with the highest fitness value
  unsigned int i_max = 0;
  for(unsigned int i=1;i<trajectories_.size();i++) {
    if(trajectories_.at(i).msg_.fitness > trajectories_.at(i_max).msg_.fitness) {
      i_max = i;
    }
  } //end for


  // Set i_best_
  i_best_ = i_max;

  // Set changed 
  changed_ = false;

  return i_best_; 
} //End getBestID



const RampTrajectory Population::getBest() {
  return trajectories_.at(getBestID());
}





/** This method will return the best trajectory from each sub-population */
const std::vector<RampTrajectory> Population::getBestFromSubPops() {
  std::vector<RampTrajectory> result;

  if(subPopulations_.size() == 0) {
    ROS_ERROR("Calling Population::getBestFromSubPops, but Population has no sub-populations!");
  }

  else {
    for(uint8_t i=0;i<subPopulations_.size();i++) {
      if(subPopulations_.at(i).size() > 0) {
        int i_best = subPopulations_.at(i).getBestID(); 
        result.push_back(subPopulations_.at(i).get(i_best));
      }
    }
  }

  return result;
}



/** This method creates sub-populations based on delta_theta.
* It will set the subPopulations_ member and also return the sub populations */
const std::vector<Population> Population::createSubPopulations(const double delta_theta) {
  subPopulations_.clear();


  // Get the number of sub-pops for delta_theta
  int num = ceil((2*PI) / delta_theta);
 
  // Create the sub-populations
  for(uint8_t i=0;i<num;i++) {
    Population sub(maxSize_, true);
    subPopulations_.push_back(sub);
  }

  // Go through each trajectory
  for(uint8_t i=0;i<trajectories_.size();i++) {

    // Get direction and Convert to [0,2PI]
    double departure_direction = trajectories_.at(i).getDirection();
    if(departure_direction < 0)
      departure_direction += (2*PI);
    

    // Find the sub-pop it belongs to
    // and add it to that sub-pop
    for(uint8_t sp=0;sp<num;sp++) {
      if(departure_direction < delta_theta*(sp+1)) {
        subPopulations_.at(sp).add(trajectories_.at(i));
        trajectories_.at(i).msg_.i_subPopulation = sp;
        sp = num;
      }
    } // end inner loop
  } // end outer loop


  // Go through each sub-population and find best
  for(uint8_t i=0;i<subPopulations_.size();i++) {
    subPopulations_.at(i).getBestID();
  }

  //std::cout<<"\n***********Leaving createSubPopulations***********\n";
  return subPopulations_;
}





/** fitness and feasible toString */
const std::string Population::fitnessFeasibleToString() const {
  std::ostringstream result;

  result<<"\n****************************************************";
  result<<"\nPopulation's fitness and feasibility:";
  for(unsigned int i=0;i<trajectories_.size();i++) {
    result<<"\n"<<trajectories_.at(i).fitnessFeasibleToString();
    if(i == i_best_) {
      result<<" - Best!";
    }
  }
  result<<"\n****************************************************";

  return result.str();
} // End fitnessFeasibleToString




/** toString */
const std::string Population::toString() const {
  std::ostringstream result;

  // If sub-populations exist, print those
  if(subPopulations_.size() > 0) {
    for(unsigned int i=0;i<subPopulations_.size();i++) {
      result<<"\n\nSub-Population "<<i<<":";

      for(unsigned int j=0;j<subPopulations_.at(i).trajectories_.size();j++) {
        result<<"\n\nTrajectory "<<subPopulations_.at(i).trajectories_.at(j).msg_.id<<": "<<subPopulations_.at(i).trajectories_.at(j).path_.toString();
      }
    }
  }

  // Otherwise, print population as a whole
  else {
    for(unsigned int i=0;i<trajectories_.size();i++) {
      result<<"\nTrajectory "<<i<<": "<<paths_.at(i).toString();
      //result<<"\nTrajectory "<<i<<": "<<trajectories_.at(i).toString();
    }
  }

  return result.str();
} //End toString




//Return a message of type ramp_msgs::Population to be sent to the trajectory viewer 
ramp_msgs::Population Population::populationMsg()
{
  ramp_msgs::Population msg;
  
  for(int i=0; i<trajectories_.size(); i++) {
    msg.population.push_back(trajectories_.at(i).msg_);
  }
  
  msg.best_id = i_best_;
  return msg;
}
