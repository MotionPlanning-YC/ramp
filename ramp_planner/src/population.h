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
    const int               add(const RampTrajectory rt);
    const unsigned int      findBest();
    void                    clear();

    void                    replace(const uint8_t i, 
                          const RampTrajectory trajec);

    const bool              replaceAll(const 
                          std::vector<RampTrajectory> new_pop);

    const RampTrajectory    get(const uint8_t i);
    const int               getMinFitness() const;

    const bool              feasibleExists() const;
    const bool              infeasibleExists() const;

    const std::vector<Population> createSubPopulations
                              (const double delta_theta=PI/3.);


    const std::string       fitnessFeasibleToString() const;
    const std::string       toString() const;
    ramp_msgs::Population   populationMsg();

    /** Data Members */
    std::vector<Path> paths_;
    unsigned int      maxSize_;
    
  private:

    const int  getReplacementID(const RampTrajectory rt) const;
    const bool replacementPossible(const RampTrajectory rt) const;
    const bool canReplace(const RampTrajectory rt, const int i) const;

    int                         i_best_;
    std::vector<RampTrajectory> trajectories_;
    bool                        changed_;
    std::vector<Population>     subPopulations_;
};

#endif
