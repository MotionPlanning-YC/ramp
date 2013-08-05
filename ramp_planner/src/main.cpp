#include "planner.h"
#include "range.h"
 
void init_pub_sub() {
}


int main(int argc, char** argv) {
  srand( time(NULL));
  Range range0(5.2, 911.7);
  Range range1(0, 180);
  Range range2(30, 150);


  Planner my_planner; 
  my_planner.ranges_.push_back(range0);
  my_planner.ranges_.push_back(range1);
  my_planner.ranges_.push_back(range2);
  
  Configuration s;
  Configuration g;
  s.ranges_ = my_planner.ranges_;
  g.ranges_ = my_planner.ranges_;
  s.random();
  g.random();
  std::cout<<"\nStart:"<<s.toString();
  std::cout<<"\nGoal:"<<g.toString();

  my_planner.start_ = s;
  my_planner.goal_ = g;

  my_planner.initialization();

  std::cout<<"\nAfter initialization!\n";
  std::cout<<"\nmy_planner.paths_.size():"<<my_planner.paths_.size();
  std::cout<<"\nmy_planner.paths_.at(0).size():"<<my_planner.paths_.at(0).all_.size()<<"\n";
  std::cout<<"\n"<<my_planner.paths_.at(0).toString();

  std::cout<<"\nExiting Normally\n";
  return 0;
}

