#include "population.h"

Population::Population() : i_best_(-1), maxSize_(6), changed_(false) {}

Population::Population(const uint8_t maxSize) : i_best_(-1), maxSize_(maxSize), changed_(false) {}


/** Return the size of the population */
const unsigned int Population::size() const { return trajectories_.size(); }

void Population::clear() { 
  trajectories_.clear(); 
  changed_ = true;
}


void Population::replace(uint8_t i, const RampTrajectory trajec) {
  changed_ = true;
  trajectories_.at(i) = trajec;
}

const bool Population::replaceAll(const std::vector<RampTrajectory> new_pop) {
  if(new_pop.size() == trajectories_.size()) {
    trajectories_ = new_pop;
    changed_ = true;
    return true;
  }
  
  return false;
}




/** This method adds a trajectory to the population. 
 *  If the population is full, a random trajectory (that isn't the best one) is replaced
 *  Returns the index that the trajectory is added at */
const unsigned int Population::add(const RampTrajectory rt) {
  changed_ = true;
 
  //If not full, simply push back
  if(trajectories_.size() < maxSize_) {
    trajectories_.push_back(rt);  
    paths_.push_back(rt.getPath());
    return trajectories_.size()-1;
  } 

  //If full, replace a trajectory
  else {

    //Generate a random index for a random trajectory to remove
    //Don't pick the fittest trajectory!!
    unsigned int i;
    do {i = rand() % maxSize_;}
    while(i == i_best_);
  
    //Remove the random trajectory
    trajectories_.erase(trajectories_.begin()+i);
    paths_.erase(paths_.begin()+i);

    //Push back the new trajectory
    trajectories_.insert(trajectories_.begin()+i, rt);
    paths_.insert(paths_.begin()+i, rt.getPath());

    return i;
  }
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





const std::vector<Population> Population::createSubPopulations(const double delta_theta) {
  std::vector<Population> result;
  subPopulations_.clear();


  // Get the number of sub-pops for delta_theta
  int num = ceil((2*PI) / delta_theta);
  std::cout<<"\ndelta_theta: "<<delta_theta;
  std::cout<<"\nnum: "<<num;
  
  // Create the sub-populations
  // TODO: How to find sub-pop size?
  for(uint8_t i=0;i<num;i++) {
    Population sub(3);
    subPopulations_.push_back(sub);
  }

  // Go through each trajectory 
  for(uint8_t i=0;i<trajectories_.size();i++) {
    std::cout<<"\nTrajectory "<<i<<": "<<trajectories_.at(i).getPath().toString();

    // Get direction and Convert to [0,2PI]
    double departure_direction = trajectories_.at(i).getDirection();
    if(departure_direction < 0) 
      departure_direction += (2*PI);
    

    // Find the sub-pop it belongs to
    // and add it to that sub-pop
    for(uint8_t sp=0;sp<num;sp++) {
      if(departure_direction < delta_theta*(sp+1)) { 
        subPopulations_.at(sp).add(trajectories_.at(i));
        sp = num;
      }
    } // end inner loop
  } // end outer loop


  // Push sub-pops onto result
  for(uint8_t i=0;i<subPopulations_.size();i++) {
    result.push_back(subPopulations_.at(i));
  }

  return result;
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
        result<<"\n\nTrajectory "<<j<<": "<<subPopulations_.at(i).trajectories_.at(j).path_.toString();
      }
    }
  }

  // Otherwise, print population as a whole
  else {
    for(unsigned int i=0;i<trajectories_.size();i++) {
      result<<"\nTrajectory "<<i<<": "<<trajectories_.at(i).toString();
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
