#ifndef POPULATION_H
#define POPULATION_H

#include "ramp_trajectory.h"
#include "utility.h"
#include "ramp_msgs/Population.h"


class Population {
  public:

    Population();
    Population(const unsigned int size);

    /* Methods */
    const unsigned int    size() const;
    const unsigned int    add(const RampTrajectory rt);
    const RampTrajectory  findBest();
    void                  clear();
    const bool            replaceAll(const std::vector<RampTrajectory> new_pop);
    const bool            checkIfChange() const;
    const std::string     fitnessFeasibleToString() const;
    const std::string     toString() const;
    ramp_msgs::Population populationMsg();

    /** Data Members */
    std::vector<RampTrajectory> population_;
    
  private:
    unsigned int        maxSize_;
    int                 i_best_;
    int                 i_best_prev_;
};

#endif
