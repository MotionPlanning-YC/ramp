#include "configuration.h"


Configuration::Configuration() {}

Configuration::~Configuration() {}


/** Set the configuration to be of random values and return the value of this configuration */
Configuration Configuration::random() {

  for(unsigned int i=0;i<ranges_.size();i++) {
    K_.push_back(ranges_.at(i).random());
  }

  return *this;
}

const std::string Configuration::toString() const {
  std::ostringstream result;
  
  result<<"("<<K_.at(0);
  for(unsigned int i=1;i<K_.size();i++) {
    result<<", "<<K_.at(i);
  }
  result<<")";

  return result.str(); 
}
