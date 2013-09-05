#ifndef POPULATION_H
#define POPULATION_H

#include "configuration.h"
#include "ramp_trajectory.h"
#include "utility.h"


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
    

    /** Data Members */
    std::vector<RampTrajectory> population_;
    
  private:

    const unsigned int max_size;
    unsigned int i_best;
};

#endif
