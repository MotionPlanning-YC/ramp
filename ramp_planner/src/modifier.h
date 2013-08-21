#ifndef MODIFIER_H
#define MODIFIER_H
#include "utility.h"
#include "path.h"
#include "ramp_msgs/ModificationRequest.h"
#include "modification_request_handler.h"

class Modifier {
  public:
    Modifier(const ros::NodeHandle& h);
    Modifier(const ros::NodeHandle& h, const std::vector<Path> ps, const std::vector< std::vector<float> > vs);    
    ~Modifier();

    //Methods
    const std::vector<Path> perform();
    const ramp_msgs::ModificationRequest buildModificationRequest();


    //Data members
    std::vector<Path> paths_;
    std::vector< std::vector<float> > velocities_;
    std::vector< std::vector<float> > new_velocities_;
    int i_changed1;
    int i_changed2;
  private:
    ModificationRequestHandler* h_mod_req_;
    unsigned int num_ops;

    unsigned int mod_op;
    Utility u;


    
    void setNewVelocities(const unsigned int op, const std::vector<Path> nps);
};

#endif
