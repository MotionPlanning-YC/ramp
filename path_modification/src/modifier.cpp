#include "modifier.h"

Modifier::Modifier(ramp_msgs::ModificationRequest::Request& req) : mod_req(req) {}

std::vector<ramp_msgs::Path> Modifier::perform() {
  std::vector<ramp_msgs::Path> result;

  if(mod_req.op == "insert") {
    in.path_ = mod_req.paths.at(0); 
    result.push_back(in.perform());
  }

  else if(mod_req.op == "delete") {
    del.path_ = mod_req.paths.at(0);
    result.push_back(del.perform());
  }
  
  else if(mod_req.op == "change") {
    chg.path_ = mod_req.paths.at(0);
    result.push_back(chg.perform());
  }

  else if(mod_req.op == "swap") {
    swap.path_ = mod_req.paths.at(0);
    result.push_back(swap.perform());
  }

  else if(mod_req.op == "crossover") {
    cross.path1_ = mod_req.paths.at(0);
    cross.path2_ = mod_req.paths.at(1);
    result = cross.perform();
  }

  return result;
}
