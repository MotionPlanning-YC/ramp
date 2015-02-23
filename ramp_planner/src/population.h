#ifndef POPULATION_H
#define POPULATION_H

#include "ramp_trajectory.h"
#include "utility.h"
#include "ramp_msgs/Population.h"


class Population {
  public:

    Population();
    Population(const unsigned int size, const bool isSubPop=false);

    /* Methods */
    const unsigned int    size() const;
    const int             add(const RampTrajectory rt);
    const int             getBestIndex() const;
    const int             findBestIndex();
    void                  clear();
    void                  replace(const uint8_t i, const RampTrajectory trajec);
    void                  replaceAll(const std::vector<RampTrajectory> new_pop);
    const RampTrajectory  get(const unsigned int i) const;
    const int             getIndexFromId(const uint16_t id) const;
    const std::vector<RampTrajectory> getTrajectories() const;
    const double          getMinFitness() const;
    const bool            contains(const RampTrajectory rt) const;
    const bool            feasibleExists() const;
    const bool            infeasibleExists() const;
    const bool            replacementPossible(const RampTrajectory rt) const;
    const bool            canReplace(const RampTrajectory rt, const int i) const;
    const int             getReplacementID(const RampTrajectory rt) const;
    const int             getNumSubPops() const;
    const RampTrajectory  getBest();
   
    const std::vector<RampTrajectory> getBestFromSubPops();
    const std::vector<Population> createSubPopulations(const double delta_theta=PI/3);


    const std::string     fitnessFeasibleToString() const;
    const std::string     toString() const;
    ramp_msgs::Population populationMsg();

    /** Data Members */
    std::vector<Path>           paths_;
    unsigned int                maxSize_;
    int                         i_best_;
    
  private:
    std::vector<RampTrajectory> trajectories_;
    bool                        changed_;
    std::vector<Population>     subPopulations_;
    bool                        isSubPopulation_;
    Utility                     utility_;
};

#endif
