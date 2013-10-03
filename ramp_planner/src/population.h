#ifndef POPULATION_H
#define POPULATION_H

#include "configuration.h"
#include "ramp_trajectory.h"
#include "utility.h"
#include "ramp_msgs/Population.h"


class Population {
  public:

    Population();
    Population(const unsigned int size);

    /* Methods */
    const unsigned int size() const;
    const unsigned int  add(const RampTrajectory rt);
    const RampTrajectory findBest();
    void clear();
    const bool replaceAll(const std::vector<RampTrajectory> new_pop);
    const std::string fitnessFeasibleToString() const;
    const std::string toString() const;
    ramp_msgs::Population populationMsg();

    /** Data Members */
    std::vector<RampTrajectory> population_;
    
  private:
    const int findBestFeasible(bool) const;
    const unsigned int max_size;
    int i_best;
};

#endif
