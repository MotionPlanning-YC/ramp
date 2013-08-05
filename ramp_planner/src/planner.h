#ifndef PLANNER_H
#define PLANNER
#include "path.h"

class Planner {
  public:
    Planner();
    Planner(const int p);
    ~Planner();


    std::vector<Path> paths_;
    std::vector<Range> ranges_;
    Configuration start_;
    Configuration goal_;
    

    void initialization();

  private:
    const int populationSize_;
};

#endif
