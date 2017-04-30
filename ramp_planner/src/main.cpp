#include "ros/ros.h"
#include "planner.h"
#include <visualization_msgs/MarkerArray.h>
 

Utility utility;


Planner             my_planner; 
int                 id;
MotionState         start, goal;
std::vector<Range>  ranges;
int                 population_size;
int                 gensBeforeCC;
bool                sub_populations;
bool                modifications;
bool                evaluations;
bool                seedPopulation;
bool                errorReduction;
bool                only_sensing;
double              t_cc_rate;
double              t_pc_rate;
int                 pop_type;
TrajectoryType      pt;
std::vector<std::string> ob_topics;

int costmap_width, costmap_height, costmap_origin_x, costmap_origin_y;
ros::Publisher pub_rviz;


// Initializes a vector of Ranges that the Planner is initialized with
void initDOF(const std::vector<double> dof_min, const std::vector<double> dof_max) 
{
  
  for(unsigned int i=0;i<dof_min.size();i++) 
  {
    Range temp(dof_min.at(i), dof_max.at(i));
    ranges.push_back(temp); 
  }

} // End initDOF



// Initializes global start and goal variables
void initStartGoal(const std::vector<float> s, const std::vector<float> g) 
{
  for(unsigned int i=0;i<s.size();i++) {
    start.msg_.positions.push_back(s.at(i));
    goal.msg_.positions.push_back(g.at(i));

    start.msg_.velocities.push_back(0);
    goal.msg_.velocities.push_back(0);

    start.msg_.accelerations.push_back(0);
    goal.msg_.accelerations.push_back(0);

    start.msg_.jerks.push_back(0);
    goal.msg_.jerks.push_back(0);
  }
} // End initStartGoal




/** Loads all of the ros parameters from .yaml 
 *  Calls initDOF, initStartGoal */
void loadParameters(const ros::NodeHandle handle) 
{
  std::cout<<"\nLoading parameters\n";
  std::cout<<"\nHandle NS: "<<handle.getNamespace();

  std::string key;
  std::vector<double> dof_min;
  std::vector<double> dof_max;


  // Get the id of the robot
  if(handle.hasParam("robot_info/id")) 
  {
    handle.getParam("robot_info/id", id);
  }
  else 
  {
    ROS_ERROR("Did not find parameter robot_info/id");
  }


  // Get the dofs
  if(handle.hasParam("robot_info/DOF_min") && 
      handle.hasParam("robot_info/DOF_max")) 
  {

    handle.getParam("robot_info/DOF_min", dof_min); 
    handle.getParam("robot_info/DOF_max", dof_max); 

    //initDOF(dof_min, dof_max);
  }
  else 
  {
    ROS_ERROR("Did not find parameters robot_info/DOF_min, robot_info/DOF_max");
  }

  /*
   * Check for all costmap parameters!
   */
  if( handle.hasParam("costmap_node/costmap/width")     &&
      handle.hasParam("costmap_node/costmap/height")    &&
      handle.hasParam("costmap_node/costmap/origin_x")  &&
      handle.hasParam("costmap_node/costmap/origin_y") )
  {
    handle.getParam("costmap_node/costmap/width", costmap_width);
    handle.getParam("costmap_node/costmap/height", costmap_height);
    handle.getParam("costmap_node/costmap/origin_x", costmap_origin_x);
    handle.getParam("costmap_node/costmap/origin_y", costmap_origin_y);

    ROS_INFO("Got costmap parameters. w: %i h: %i x: %i y: %i", costmap_width, costmap_height, costmap_origin_x, costmap_origin_y);

    int x_max = costmap_width + costmap_origin_x;
    int x_min = costmap_origin_x;
    int y_max = costmap_height + costmap_origin_y;
    int y_min = costmap_origin_y;
    
    std::vector<double> dof_min, dof_max;
    dof_min.push_back(x_min);
    dof_min.push_back(y_min);
    dof_max.push_back(x_max);
    dof_max.push_back(y_max);

    dof_min.push_back(-PI);
    dof_max.push_back(PI);

    initDOF(dof_min, dof_max); 
  }


  // Get the start and goal vectors
  if(handle.hasParam("robot_info/start") &&
      handle.hasParam("robot_info/goal"))
  {
    std::vector<float> p_start;
    std::vector<float> p_goal;
    handle.getParam("robot_info/start", p_start);
    handle.getParam("robot_info/goal",  p_goal );
    initStartGoal(p_start, p_goal);
  }
  else 
  {
    ROS_ERROR("Did not find parameters robot_info/start, robot_info/goal");
  }



  if(handle.hasParam("ramp/population_size")) 
  {
    handle.getParam("ramp/population_size", population_size);
    std::cout<<"\npopulation_size: "<<population_size;
  }

  
  if(handle.hasParam("ramp/sub_populations")) 
  {
    handle.getParam("ramp/sub_populations", sub_populations);
    std::cout<<"\nsub_populations: "<<sub_populations;
  }
  
  if(handle.hasParam("ramp/modifications")) 
  {
    handle.getParam("ramp/modifications", modifications);
    std::cout<<"\nmodifications: "<<modifications;
  }
  
  if(handle.hasParam("ramp/evaluations")) 
  {
    handle.getParam("ramp/evaluations", evaluations);
    std::cout<<"\nevaluations: "<<evaluations;
  }
  
  if(handle.hasParam("ramp/seed_population")) 
  {
    handle.getParam("ramp/seed_population", seedPopulation);
    std::cout<<"\nseed_population: "<<seedPopulation;
  }
  
  if(handle.hasParam("ramp/only_sensing"))
  {
    handle.getParam("ramp/only_sensing", only_sensing);
    std::cout<<"\nonly_sensing: "<<only_sensing;
  }

  if(handle.hasParam("ramp/gens_before_control_cycle")) 
  {
    handle.getParam("ramp/gens_before_control_cycle", gensBeforeCC);
    std::cout<<"\ngens_before_control_cycle: "<<gensBeforeCC;
  }
  
  if(handle.hasParam("ramp/fixed_control_cycle_rate")) 
  {
    handle.getParam("ramp/fixed_control_cycle_rate", t_cc_rate);
    ROS_INFO("t_cc_rate: %f", t_cc_rate);
  }
  
  if(handle.hasParam("ramp/pop_traj_type")) 
  {
    handle.getParam("ramp/pop_traj_type", pop_type);
    ROS_INFO("pop_type: %s", pop_type ? "Partial Bezier" : "All Straight");
    switch (pop_type) 
    {
      case 0:
        pt = HOLONOMIC;
        break;
      case 1:
        pt = HYBRID;
        break;
    }
  }
  
  if(handle.hasParam("ramp/error_reduction")) 
  {
    handle.getParam("ramp/error_reduction", errorReduction);
    ROS_INFO("errorReduction: %s", errorReduction ? "True" : "False");
  }



  std::cout<<"\n------- Done loading parameters -------\n";
    std::cout<<"\n  ID: "<<id;
    std::cout<<"\n  Start: "<<start.toString();
    std::cout<<"\n  Goal: "<<goal.toString();
    std::cout<<"\n  Ranges: ";
    for(uint8_t i=0;i<ranges.size();i++) 
    {
      std::cout<<"\n  "<<i<<": "<<ranges.at(i).toString();
    }
  std::cout<<"\n---------------------------------------";
}



void pubStartGoalMarkers()
{
  ROS_INFO("In pubStartGoalMarkers");
  visualization_msgs::MarkerArray result;

  // Make Markers for both positions
  visualization_msgs::Marker start_marker, goal_marker;

  start_marker.header.stamp = ros::Time::now();
  goal_marker.header.stamp = ros::Time::now();
  start_marker.id = 10000;
  goal_marker.id = 10001;

  start_marker.header.frame_id = "/map_rot";
  goal_marker.header.frame_id = "/map_rot";

  start_marker.ns = "basic_shapes";
  goal_marker.ns = "basic_shapes";

  start_marker.type = visualization_msgs::Marker::SPHERE;
  goal_marker.type = visualization_msgs::Marker::SPHERE;

  start_marker.action = visualization_msgs::Marker::ADD;
  goal_marker.action = visualization_msgs::Marker::ADD;
  
  // Set positions
  start_marker.pose.position.x = start.msg_.positions[0];
  start_marker.pose.position.y = start.msg_.positions[1];
  start_marker.pose.position.z = start.msg_.positions[2];
  
  goal_marker.pose.position.x = goal.msg_.positions[0];
  goal_marker.pose.position.y = goal.msg_.positions[1];
  goal_marker.pose.position.z = goal.msg_.positions[2];

  // Set orientations
  start_marker.pose.orientation.x = 0.0;
  start_marker.pose.orientation.y = 0.0;
  start_marker.pose.orientation.z = 0.0;
  start_marker.pose.orientation.w = 1.0;
 
  goal_marker.pose.orientation.x = 0.0;
  goal_marker.pose.orientation.y = 0.0;
  goal_marker.pose.orientation.z = 0.0;
  goal_marker.pose.orientation.w = 1.0;

  // Set radii
  start_marker.scale.x = 0.5;
  start_marker.scale.y = 0.5;
  start_marker.scale.z = 0.1;
 
  goal_marker.scale.x = 0.5;
  goal_marker.scale.y = 0.5;
  goal_marker.scale.z = 0.1;

  // Set colors
  start_marker.color.r = 1;
  start_marker.color.g = 0;
  start_marker.color.b = 0;
  start_marker.color.a = 1;
 
  goal_marker.color.r = 0;
  goal_marker.color.g = 1;
  goal_marker.color.b = 0;
  goal_marker.color.a = 1;

  // Set lifetimes
  start_marker.lifetime = ros::Duration(10.0);
  goal_marker.lifetime = ros::Duration(10.0);

  // Create marker array and publish
  result.markers.push_back(start_marker);
  result.markers.push_back(goal_marker);

  while(pub_rviz.getNumSubscribers() == 0) {}
  ROS_INFO("# of subscribers: %i", (int)pub_rviz.getNumSubscribers());

  pub_rviz.publish(result);
  pub_rviz.publish(result);
  
  ROS_INFO("Exiting pubStartGoalMarkers");
}



int main(int argc, char** argv) {
  srand( time(0));

  ros::init(argc, argv, "ramp_planner");
  ros::NodeHandle handle;

  ros::param::set("ramp/cc_started", false);
  std::cout<<"\nHandle namespace: "<<handle.getNamespace();
  
  ros::Subscriber sub_update_ = handle.subscribe("update", 1, &Planner::updateCallback, &my_planner);
  ros::Subscriber sub_sc_ = handle.subscribe("obstacles", 1, &Planner::sensingCycleCallback, &my_planner);

  pub_rviz = handle.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);



  // Load ros parameters
  loadParameters(handle);

  ROS_INFO("Parameters loaded. Please review them and press Enter to continue");
  //std::cin.get();
  
  for(int i=0;i<ranges.size();i++)
  {
    if( start.msg_.positions[i] < ranges[i].msg_.min || start.msg_.positions[i] > ranges[i].msg_.max ||
        goal.msg_.positions[i] < ranges[i].msg_.min || goal.msg_.positions[i] > ranges[i].msg_.max )
    {
      ROS_ERROR("Either the Start or goal position is not within DOF ranges, exiting ramp_planner");
      exit(1);
    }
  }
 
  /** Initialize the Planner's handlers */ 
  ROS_INFO("Initializing Planner object");
  my_planner.init(id, handle, start, goal, ranges, population_size, sub_populations, pt, gensBeforeCC, t_pc_rate, t_cc_rate, errorReduction); 
  my_planner.modifications_   = modifications;
  my_planner.evaluations_     = evaluations;
  my_planner.seedPopulation_  = seedPopulation;

  std::cout<<"\nStart: "<<my_planner.start_.toString();
  std::cout<<"\nGoal: "<<my_planner.goal_.toString();

  pubStartGoalMarkers();

  
  //testSwitch();
  //exit(1);
 
 
  /******* Start the planner *******/
  std::cout<<"\nPress Enter to start the planner\n";
  std::cin.get(); 
  
  my_planner.go();


 
  //****MotionState exp_results = my_planner.findAverageDiff();
  //****std::cout<<"\n\nAverage Difference: "<<exp_results.toString();
  
  
  std::cout<<"\n\nExiting Normally\n";
  ros::shutdown();
  return 0;
}

