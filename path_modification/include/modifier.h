#ifndef MODIFIER_H
#define MODIFIER_H
#include "ramp_msgs/ModificationRequest.h"
#include "insert.h"
#include "delete.h"
#include "change.h"
#include "crossover.h"
#include "swap.h"
#include "move.h"


class Modifier {
  public:

    Modifier(ramp_msgs::ModificationRequest::Request& req);
    ~Modifier() {}
    
    Insert in;
    Delete del;
    Change chg;
    Crossover cross;
    Swap swap; 

    ramp_msgs::ModificationRequest::Request mod_req;
    const std::vector<ramp_msgs::Path> perform();

    Utility u;
};

#endif
