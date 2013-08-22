#ifndef POPULATION_H
#define POPULATION_H

#include <queue>
#include "ramp_trajectory.h"
#include "utility.h"


class Population {
  public:

    Population();
    Population(const unsigned int size);

    const RampTrajectory evaluateAndObtainBest();
    void add(const RampTrajectory rt);
    const RampTrajectory getBest() const;
    const std::string toString() const;
    
    std::vector<RampTrajectory> population_;
    
  private:
    const unsigned int max_size;
    unsigned int i_best;
};

#endif
