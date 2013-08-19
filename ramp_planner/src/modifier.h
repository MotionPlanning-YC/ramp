#ifndef MODIFIER_H
#define MODIFIER_H
#include "utility.h"
#include "path.h"
#include "ramp_msgs/ModificationRequest.h"
#include "modification_request_handler.h"

class Modifier {
  public:
    Modifier(const ros::NodeHandle& h);
    Modifier(const ros::NodeHandle& h, const std::vector<Path> ps);    
    ~Modifier();

    //Methods
    std::vector<Path> perform();
    ramp_msgs::ModificationRequest buildModificationRequest();


    //Data members
    std::vector<Path> paths_;
  private:
    ModificationRequestHandler* h_mod_req_;
    unsigned int num_ops;
};

#endif
