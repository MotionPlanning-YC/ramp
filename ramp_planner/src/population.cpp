#include "population.h"

/*************************************
 * The paths_ vector must be changed * 
 * any time trajectories_ is changed *
 *************************************/

Population::Population() : i_best_(-1), changed_(true) {}

Population::Population(const uint8_t maxSize) : maxSize_(maxSize), i_best_(-1), changed_(true) {}


/** Return the size of the population */
const unsigned int Population::size() const { return trajectories_.size(); }


void Population::clear() { 
  trajectories_.clear(); 
  paths_.clear();
  changed_ = true;
}


/** This method replaces the trajectory at index i with the trajectory passed in as trajec. It also updates the paths_ vector */
void Population::replace(uint8_t i, const RampTrajectory trajec) {
  changed_ = true;
  trajectories_.at(i) = trajec;
  paths_.at(i) = trajec.path_;
} // End replace



/** This method replaces the whole population with a new set of trajectories. The paths_ vector is updated with the paths of the new trajectories. */
const bool Population::replaceAll(const std::vector<RampTrajectory> new_pop) {
  if(new_pop.size() == trajectories_.size()) {
    changed_ = true;

    // Set trajectories and update paths
    trajectories_ = new_pop;
    for(uint8_t i=0;i<new_pop.size();i++) {
      paths_.at(i) = new_pop.at(i).path_;
    }
    
    return true;
  }
  
  return false;
} // End replaceAll



/** This method returns the minimum fitness of the population */
const int Population::getMinFitness() const {
  int result = trajectories_.at(0).fitness_;

  for(uint8_t i=1;i<trajectories_.size();i++) {
    if(trajectories_.at(i).fitness_ < result) {
      result = trajectories_.at(i).fitness_;
    }
  }
  
  return result;
} // End getMinFitness




/** This method returns true if there is at least one feasible trajectory in the population */
const bool Population::feasibleExists() const {
  for(uint8_t i=0;i<trajectories_.size();i++) {
    if(trajectories_.at(i).feasible_) {
      return true;
    }
  }

  return false;
} // End feasibleExists



/** This method returns true if there is at least one infeasible trajectory in the population */
const bool Population::infeasibleExists() const {
  for(uint8_t i=0;i<trajectories_.size();i++) {
    if(!trajectories_.at(i).feasible_) {
      return true;
    }
  }

  return false;
} // End infeasibleExists



/** This method checks if a trajectory can replace an existing trajectory in the population */
const bool Population::replacementPossible(const RampTrajectory rt) const {
  //std::cout<<"\nIn replacementPossible\n";

  // If the fitness is not higher than the minimum fitness
  if(rt.fitness_ <= getMinFitness()) {
    std::cout<<"\nfitness < min fitness, not possible\n";
    return false;
  }
  
  // If the trajectory is infeasible and
  // no infeasible trajectories exist, no
  // trajectories can be replaced
  if(!rt.feasible_ && !infeasibleExists()) {
    std::cout<<"\nrt infeasible, no other infeasible, not possible\n";
    return false;
  } 

  /** IF subpopulations are being used */
  if(subPopulations_.size() > 0) {
    std::cout<<"\nIn sub-pops\n";
    
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
    if(!rt.feasible_) {
      bool valid=false;
      for(uint8_t i=0;i<i_validSubpops.size();i++) {
        if(subPopulations_.at(i_validSubpops.at(i)).infeasibleExists() &&
            rt.fitness_ > subPopulations_.at(i_validSubpops.at(i)).getMinFitness()) 
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

  return true;
} // End replacementPossible




/** This method returns true if rt can replace the trajectory at index i */
const bool Population::canReplace(const RampTrajectory rt, const int i) const {
  //std::cout<<"\n"<<toString();
  //std::cout<<"\nIn canReplace, i: "<<i<<", trajectory: "<<trajectories_.at(i).id_<<"\n";
  
  if(i == i_best_) {
    //std::cout<<"\ni == i_best, returning false\n";
    return false;
  }

  if(!rt.feasible_ && trajectories_.at(i).feasible_) {
    //std::cout<<"\nrt infeasible, i feasible, returning false\n";
    return false;
  }


  // If sub-populations are used,
  if(subPopulations_.size() > 0) {
    
    //std::cout<<"\ntrajectories.size(): "<<trajectories_.size();
    RampTrajectory temp = trajectories_.at(i);

    //std::cout<<"\nsubPopulations.size(): "<<subPopulations_.size();
    //std::cout<<"\ntemp.subPopulation: "<<temp.subPopulation_<<"\n";
    Population p = subPopulations_.at(temp.subPopulation_);

    if(p.trajectories_.size() < 2) {
      //std::cout<<"\nSub-Population size < 2, returning false\n";
      return false;
    }

    //std::cout<<"\np.trajectories.size(): "<<p.trajectories_.size();
    //std::cout<<"\np.i_best_: "<<p.i_best_<<"\n";

    // if i is the best in trajectory i's sub-population
    if(temp.equal( p.trajectories_.at(p.i_best_) ))  {
      //std::cout<<"\ntemp == best in sub-population, returning false\n";
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
 *   - Will happen if rt is infeasible and the rest are feasible
 *   - or if each sub-population has <= 1 trajectory */
const int Population::getReplacementID(const RampTrajectory rt) const {
  //std::cout<<"\nIn getReplacementID\n";
  
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
 *  Returns the index that the trajectory is added at 
 *  or -1 if the trajectory is not fit enough to be added */
const int Population::add(const RampTrajectory rt) {
  changed_ = true;
  //std::cout<<"\nIn add\n";

  if(subPopulations_.size() > 0) {
    // Go through each sub-population and find best
    for(uint8_t i=0;i<subPopulations_.size();i++) {
      subPopulations_.at(i).findBest();
    }
  }
 
  // If not full, simply push back
  if(trajectories_.size() < maxSize_) {
    //std::cout<<"\nIn pop not full\n";
    //std::cout<<"\nmaxSize: "<<maxSize_;
    //std::cout<<"\ntrajectories.size(): "<<trajectories_.size()<<"\n";
    trajectories_.push_back(rt);  
    paths_.push_back(rt.getPath());
    return trajectories_.size()-1;
  } 

  // If full, replace a trajectory
  else if(replacementPossible(rt)) {

    // Generate an index for the trajectory to remove
    int i = getReplacementID(rt);
    //std::cout<<"\nFound replacement ID: "<<i<<"\n";

    // Do the replacement
    replace(i, rt);

    //std::cout<<"\nAfter replacement\n";

    return i;
  }


  std::cout<<"\nCould not add trajectory\n";
  // If the trajectory could not be added, 
  // set changed_ to false and return an error code
  changed_ = false;
  return -1;
} //End add





/** Returns the fittest trajectory and sets i_best_ */
const unsigned int Population::findBest() {
  //std::cout<<"\nIn findBest\n";
  
  // If population has not changed since last
  // findBest call, return without searching
  if(!changed_) {
    return i_best_;
  }
  
  // Find the index of the trajectory with the highest fitness value
  unsigned int i_max = 0;
  for(unsigned int i=1;i<trajectories_.size();i++) {
    if(trajectories_.at(i).fitness_ > trajectories_.at(i_max).fitness_) {
      i_max = i;
    }
  } //end for


  // Set i_best_
  i_best_ = i_max;

  // Set changed 
  changed_ = false;

  return i_best_; 
} //End findBest 





/** This method returns the trajectory at index i */
const RampTrajectory Population::get(const uint8_t i) {
  return trajectories_.at(i);
} // End get





/** This method creates sub-populations based on delta_theta. 
 *  It will set the subPopulations_ member and also return the sub populations */
const std::vector<Population> Population::createSubPopulations(const double delta_theta) {
  subPopulations_.clear();


  // Get the number of sub-pops for delta_theta
  int num = ceil((2*PI) / delta_theta);
  
  // Create the sub-populations
  // TODO: How to find sub-pop size?
  for(uint8_t i=0;i<num;i++) {
    Population sub(3);
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
        trajectories_.at(i).subPopulation_ = sp;
        sp = num;
      }
    } // end inner loop
  } // end outer loop


  // Go through each sub-population and find best
  for(uint8_t i=0;i<subPopulations_.size();i++) {
    subPopulations_.at(i).findBest();
  }

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
} //End toString


/** toString */
const std::string Population::toString() const {
  std::ostringstream result;

  // If sub-populations exist, print those
  if(subPopulations_.size() > 0) {
    for(unsigned int i=0;i<subPopulations_.size();i++) {
      result<<"\n\nSub-Population "<<i<<":";

      for(unsigned int j=0;j<subPopulations_.at(i).trajectories_.size();j++) {
        result<<"\n\nTrajectory "<<subPopulations_.at(i).trajectories_.at(j).id_<<": "<<subPopulations_.at(i).trajectories_.at(j).path_.toString();
      }
    }
  }

  // Otherwise, print population as a whole
  else {
    for(unsigned int i=0;i<trajectories_.size();i++) {
      //result<<"\nTrajectory "<<i<<": "<<trajectories_.at(i).path_.toString();
      result<<"\nTrajectory "<<i<<": "<<paths_.at(i).toString();
    }
  }

  return result.str();
} //End toString

//Return a message of type ramp_msgs::Population to be sent to the trajectory viewer 
ramp_msgs::Population Population::populationMsg()
{
  ramp_msgs::Population msg;
  
  for(int i=0; i<trajectories_.size(); i++) {
    msg.population.push_back(trajectories_.at(i).msg_trajec_);
  }
  
  msg.best_id = i_best_;
  return msg;
}
