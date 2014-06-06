#ifndef POPULATION_H
#define POPULATION_H

#include "ramp_trajectory.h"
#include "utility.h"
#include "ramp_msgs/Population.h"


class Population {
  public:

    Population();
    Population(const uint8_t maxSize);

    /* Methods */
    const unsigned int      size() const;
    const unsigned int      add(const RampTrajectory rt);
    const unsigned int      findBest();
    void                    clear();
    void                    replace(const uint8_t i, const RampTrajectory trajec);
    const bool              replaceAll(const std::vector<RampTrajectory> new_pop);
    const RampTrajectory    get(const uint8_t i);
    const std::string       fitnessFeasibleToString() const;
    const std::string       toString() const;
    ramp_msgs::Population   populationMsg();
    
    const std::vector<Population> createSubPopulations
                              (const double delta_theta=PI/3.);

    /** Data Members */
    std::vector<Path> paths_;
    
  private:
    int                         i_best_;
    std::vector<RampTrajectory> trajectories_;
    unsigned int                maxSize_;
    bool                        changed_;
    std::vector<Population>     subPopulations_;
};

#endif
