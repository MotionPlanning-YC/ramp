#ifndef MODIFIER_H
#define MODIFIER_H
#include "utility.h"
#include "path.h"
#include "ramp_msgs/ModificationRequest.h"
#include "modification_request_handler.h"

class Modifier {
  public:
    Modifier(const ros::NodeHandle& h, const unsigned int n);
    Modifier(const ros::NodeHandle& h, const std::vector<Path> ps, const unsigned int n);    
    ~Modifier();

    // Methods
    const std::vector<Path> perform();
    void updateAll(const std::vector<Path> ps);
    void update(const Path p, const unsigned int i);
    const ramp_msgs::ModificationRequest buildModificationRequest();
    const Path stop(Path p);


    // Data members
    unsigned int num_ops;
    std::vector<Path> paths_;
    int i_changed1;
    int i_changed2;

  private:
    const std::string getOperator() const;
    const std::vector<int> getTargets(const std::string op);

    ModificationRequestHandler* h_mod_req_;
    Utility u;
};

#endif
