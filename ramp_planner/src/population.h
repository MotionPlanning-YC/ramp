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
    const unsigned int    findBest();
    void                  clear();
    const bool            replaceAll(const std::vector<RampTrajectory> new_pop);
    RampTrajectory&       get(const unsigned int i);
    const std::string     fitnessFeasibleToString() const;
    const std::string     toString() const;
    ramp_msgs::Population populationMsg();

    /** Data Members */
    
  private:
    int                         i_best_;
    std::vector<RampTrajectory> population_;
    unsigned int                maxSize_;
    bool                        changed_;
};

#endif
