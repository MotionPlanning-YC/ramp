#include "planner.h"
#include <ros/ros.h>
#include <std_srvs/Empty.h>
 

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
double              t_cc_rate;
double              t_pc_rate;
int                 num_obs;
int                 pop_type;
TrajectoryType      pt;
std::vector<std::string> ob_topics;
std::vector<tf::Transform> ob_tfs;
ros::Publisher pub_obs;


// Initializes a vector of Ranges that the Planner is initialized with
void initDOF(const std::vector<double> dof_min, const std::vector<double> dof_max) 
{
  
  for(unsigned int i=0;i<dof_min.size();i++) {
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



void loadObstacleTF()
{
  std::ifstream ifile("/home/sterlingm/ros_workspace/src/ramp/ramp_planner/obstacle_tf.txt", std::ios::in);

  if(!ifile.is_open())
  {
    ROS_ERROR("Cannot open obstacle_tf.txt file!");
  }
  else
  {
    std::string line;
    std::string delimiter = ",";
    while( getline(ifile, line) )
    {
      ROS_INFO("Got line: %s", line.c_str());
      std::vector<double> conf;
      size_t pos = 0;
      std::string token;
      while((pos = line.find(delimiter)) != std::string::npos)
      {
        token = line.substr(0, pos);
        ROS_INFO("Got token: %s", token.c_str());
        conf.push_back(stod(token));
        line.erase(0, pos+1);
      } // end inner while
    
      ROS_INFO("Last token: %s", line.c_str());

      conf.push_back(stod(line));

      tf::Transform temp;
      temp.setOrigin( tf::Vector3(conf.at(0), conf.at(1), 0));
      temp.setRotation(tf::createQuaternionFromYaw(conf.at(2)));

      ob_tfs.push_back(temp);
      
    } // end outter while
  } // end else


  ifile.close();
}


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

    initDOF(dof_min, dof_max);
  }
  else 
  {
    ROS_ERROR("Did not find parameters robot_info/DOF_min, robot_info/DOF_max");
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

  if(handle.hasParam("ramp/num_of_obstacles"))
  {
    handle.getParam("ramp/num_of_obstacles", num_obs);
    ROS_INFO("num_of_obstacles: %i", num_obs);
  }


  if(handle.hasParam("ramp/obstacle_topics"))
  {
    handle.getParam("ramp/obstacle_topics", ob_topics);
    ROS_INFO("ob_topics.size(): %i", (int)ob_topics.size());
    for(int i=0;i<ob_topics.size();i++)
    {
      ROS_INFO("ob_topics[%i]: %s", i, ob_topics.at(i).c_str());
    }
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





enum Group 
{
  EXTERIOR = 0,
  INTERIOR = 1,
  CRITICAL = 2,
  MIX      = 3
};

struct ABTC
{
  bool moving[9];
  double times[9];
  
};


struct ObInfo
{
  double x;
  double y;
  double v;
  double w;
  double relative_direction;
  double d;
  ramp_msgs::Obstacle msg;
  
  bool faster;
};

struct TestCase {
  ros::Time t_begin;
  std::vector<ObInfo> obs;
  ramp_msgs::ObstacleList ob_list;
  std::vector<RampTrajectory> ob_trjs;
  Group group;
  int history;

  bool success;
};


struct TestCaseTwo {
  ABTC abtc;
  std::vector<ObInfo> obs;
  ramp_msgs::ObstacleList ob_list;
  std::vector<ramp_msgs::RampTrajectory> ob_trjs;
  ros::Time t_begin;
};

ramp_msgs::Obstacle getStaticOb(ramp_msgs::Obstacle ob)
{
  ramp_msgs::Obstacle result = ob; 

  result.odom_t.twist.twist.linear.x = 0;
  result.odom_t.twist.twist.linear.y = 0;
  result.odom_t.twist.twist.linear.z = 0;

  return result;
}

ObInfo generateObInfoSimple(const MotionState robot_state)
{
  ObInfo result;

  Range v(0,  0.33);
  Range w(0,  PI/2.f);

  result.v = v.random();
  result.w = w.random();
  result.relative_direction = 0;

  result.x = robot_state.msg_.positions[0];
  result.y = robot_state.msg_.positions[1] + 1;

  result.d = 1;
  
  return result;
}


ObInfo generateObInfoGrid(const MotionState robot_state)
{
  ObInfo result;

  Range x(0.5, 2.);
  Range y(0.5, 2.);

  double ob_x = x.random();
  
  // Round to 1 decimal place for grid of .1m x .1m
  ob_x *= 10;
  ob_x = round(ob_x);
  ob_x /= 10;

  double ob_y = y.random();
  
  // Round to 1 decimal place for grid of .1m x .1m
  ob_y *= 10;
  ob_y = round(ob_y);
  ob_y /= 10;


  Range v(0, 0.5);
  Range w(0, PI/2.f);

  result.x = ob_x;
  result.y = ob_y;
  result.v = v.random();
  result.w = w.random();
  
  result.relative_direction = utility.displaceAngle(atan( ob_y / ob_x ), PI);
  
  return result;
}

ObInfo generateObInfo(const MotionState robot_state)
{
  ObInfo result;

  Range dist(0, 3.5);

  Range v(0, 0.5);
  Range w       (0      ,  PI/2.f);
  Range rela_dir(PI/6.f ,  5.f*PI/6.f);
  
  
  result.v                  = v.random();
  result.w                  = w.random();
  result.relative_direction = rela_dir.random();
  
  
  result.d    = dist.random();
  double ob_x = robot_state.msg_.positions[0] + result.d*cos(result.relative_direction);
  double ob_y = robot_state.msg_.positions[1] + result.d*sin(result.relative_direction);

  if(ob_x > 3.5)
    ob_x = 3.5;
  else if(ob_x < 0)
    ob_x = 0.;
  if(ob_y > 3.5)
    ob_y = 3.5;
  else if(ob_y < 0)
    ob_y = 0.;

  result.x = ob_x;
  result.y = ob_y;

  result.faster = result.v > sqrt(pow(robot_state.msg_.velocities[0],2) + pow(robot_state.msg_.velocities[1],2));

  return result;
}



/*
 * p_x and p_y are the position values,                 range: [0, 3.5]
 * v_mag is the magnitude of the linear velocity,       range: [0, 0.25]
 * v_direction is the direction of the linear velocity, range: [0, pi]
 * w is the angular velocity (not a vector),            range: [-pi/4, pi/4]
 */
const ramp_msgs::Obstacle buildObstacleMsg(const double& p_x, const double& p_y, const double& v_mag, const double& v_direction, const double& w)
{
  ROS_INFO("p_x: %f p_y: %f, v_mag: %f v_direction: %f w: %f", p_x, p_y, v_mag, v_direction, w);

  ramp_msgs::Obstacle result;

  // odom_msg describes the obstacle's position and velocity
  nav_msgs::Odometry odom_msg;

  // Set the x,y position
  odom_msg.pose.pose.position.x = p_x;
  odom_msg.pose.pose.position.y = p_y; 
  odom_msg.pose.pose.position.z = 0;

  // Set orientation
  odom_msg.pose.pose.orientation  = tf::createQuaternionMsgFromYaw(v_direction);
  
  // For linear velocity, calculate x and y components
  double v_x = v_mag*cos(v_direction);
  double v_y = v_mag*sin(v_direction);

  // Set velocities
  odom_msg.twist.twist.linear.x   = v_x;
  odom_msg.twist.twist.linear.y   = v_y;
  odom_msg.twist.twist.angular.z  = w;

  // Set odom_msg and return result
  result.odom_t = odom_msg;
  return result;
}


TestCase generateTestCase(const MotionState robot_state, double inner_r, double outter_r, int num_obs)
{
  TestCase result;

  Range history(0,20);

  // Generate all obstacles and push them onto test case
  for(int i=0;i<num_obs;i++)
  {
    //ObInfo temp = generateObInfo(robot_state);
    ObInfo temp = generateObInfoSimple(robot_state);

    temp.msg = buildObstacleMsg(temp.x, temp.y, temp.v, temp.relative_direction, temp.w);
    
    result.obs.push_back(temp);
    result.ob_list.obstacles.push_back(temp.msg);

    result.history = history.random();
  }

  return result;
}

/*
 * Does NOT generate an ABTC
 * Put that in later on once single test case is working well
 */
TestCaseTwo generateTestCase(const MotionState robot_state, int num_obs)
{
  TestCaseTwo result;

  ROS_INFO("In generateTestCase");
  ROS_INFO("num_obs: %i", num_obs);

  // Generate all obstacles and push them onto test case
  for(int i=0;i<num_obs;i++)
  {
    //ObInfo temp = generateObInfo(robot_state);
    ObInfo temp = generateObInfoGrid(robot_state);

    temp.msg = buildObstacleMsg(temp.x, temp.y, temp.v, temp.relative_direction, temp.w);
    
    result.obs.push_back(temp);
    result.ob_list.obstacles.push_back(temp.msg);
    ROS_INFO("result.obs.size(): %i", (int)result.obs.size());
    ROS_INFO("result.ob_list.obstacles.size(): %i", (int)result.ob_list.obstacles.size());
  }

  return result;
}


MotionState getGoal(const MotionState init, const double dim)
{
  ROS_INFO("getGoal init: %s", init.toString().c_str());

  double r = sqrt( pow(dim,2) * 2 );
  double x = init.msg_.positions[0] + r*cos(PI/4.f);
  double y = init.msg_.positions[1] + r*sin(PI/4.f);

  MotionState result;
  result.msg_.positions.push_back(x);
  result.msg_.positions.push_back(y);
  result.msg_.positions.push_back(init.msg_.positions[2]);

  ROS_INFO("getGoal result: %s", result.toString().c_str());
  return result;
}



/*
 * Publish obstacle information at 20Hz to simulate sensing cycles
 */
void pubObTrj(const ros::TimerEvent e, TestCaseTwo& tc)
{
  ROS_INFO("In pubObTrj");

  ros::Duration d_elapsed = ros::Time::now() - tc.t_begin;
  
  int index = d_elapsed.toSec() * 10;

  for(int i=0;i<tc.ob_trjs.size();i++)
  {
    trajectory_msgs::JointTrajectoryPoint p = tc.ob_trjs[i].trajectory.points[index]; 

    ROS_INFO("New point: %s", utility.toString(p).c_str());
    
    
    // Build new obstacle msg
    ramp_msgs::Obstacle ob = buildObstacleMsg(p.positions[0], p.positions[1], tc.obs[i].v, p.positions[2], tc.obs[i].w);

   
    tc.obs[i].msg = ob;

    tc.ob_list.obstacles[i] = ob;
  }


  pub_obs.publish(tc.ob_list);
}


int main(int argc, char** argv) {
  srand( time(0));

  ros::init(argc, argv, "ramp_planner");
  ros::NodeHandle handle;
  
  // Load ros parameters and obstacle transforms
  //loadParameters(handle);
  //loadObstacleTF();

  num_obs = 3;

  ros::Rate r(100);

  //my_planner.ranges_ = ranges;
  
  ros::Timer ob_trj_timer;
  
  int num_tests = 1;
  int num_successful_tests = 0;
  std::vector<int> num_generations;
  std::vector<TestCase> test_cases;


  // Make an ObstacleList Publisher
  pub_obs = handle.advertise<ramp_msgs::ObstacleList>("obstacles", 1);

  ros::ServiceClient trj_gen = handle.serviceClient<ramp_msgs::TrajectorySrv>("trajectory_generator");
  
  // Set flag signifying that the next test case is not ready
  ros::param::set("/ramp/tc_generated", false);

  ros::Duration d_history(1);
  ros::Duration d_test_case_thresh(20);
  for(int i=0;i<num_tests;i++)
  {
    MotionState initial_state;
    //my_planner.randomMS(initial_state);

    /*
     *
     * Generate a test case
     *
     */
   
    /*
     * Generate the test case
     */

    ABTC abtc;

    /*
     * Create test case where all obstacles stop, move, stop for 1 second each
     */
    for(int i_ob=0;i_ob<3;i_ob++)
    {
      abtc.moving[i_ob]   = 0;
      abtc.moving[i_ob+3] = 1;
      abtc.moving[i_ob+6] = 0;
      abtc.times[i_ob] = 1;
      abtc.times[i_ob+3] = 1;
      abtc.times[i_ob+6] = 1;
    }

    generateObInfoGrid(initial_state);
    
    /*
     * Get test data for the abtc
     */
    TestCaseTwo tc = generateTestCase(initial_state, num_obs);
    tc.abtc = abtc; 

    /*
     * Get trajectories for each obstacle
     */
    ramp_msgs::TrajectorySrv tr_srv;
    for(int i=0;i<tc.obs.size();i++)
    {
      ramp_msgs::TrajectoryRequest tr;

      tf::Transform tf;
      tf.setOrigin( tf::Vector3(0,0,0) );
      tf.setRotation( tf::createQuaternionFromYaw(0) );
      MotionType mt = my_planner.findMotionType(tc.obs[i].msg);

      ramp_msgs::Path p = my_planner.getObstaclePath(tc.obs[i].msg, tf, mt);

      tr.path = p;
      tr.type = PREDICTION;
    
      tr_srv.request.reqs.push_back(tr);
   
    }

    // Call trajectory generator
    if(trj_gen.call(tr_srv))
    {
      for(int i=0;i<tr_srv.response.resps.size();i++)
      {
        ROS_INFO("Traj: %s", utility.toString(tr_srv.response.resps[i].trajectory).c_str());
        tc.ob_trjs.push_back(tr_srv.response.resps[i].trajectory);
      }
    }
    else
    {
      ROS_ERROR("Error in getting obstacle trajectories");
    }
 
    /*
     * Get static version of obstacles
     */
    ramp_msgs::ObstacleList obs_stat;
    for(int i=0;i<tc.obs.size();i++)
    {
      obs_stat.obstacles.push_back(getStaticOb(tc.obs[i].msg));
    }
    ROS_INFO("Generate: obs_stat.size(): %i", (int)obs_stat.obstacles.size());


    // Set flag signifying that the next test case is ready
    ros::param::set("/ramp/tc_generated", true);

    ROS_INFO("Generate: Waiting for planner to prepare");
    /*
     * Wait for planner to be ready to start test case
     */
    bool start_tc = false;
    while(!start_tc)
    {
      handle.getParam("/ramp/ready_tc", start_tc);
      r.sleep();
      ros::spinOnce();
    }

    ROS_INFO("Generate: Planner ready, publishing static obstacles");

    // Publish static obstacles
    pub_obs.publish(obs_stat);

    // Wait for 1 second
    d_history.sleep();

    ROS_INFO("Generate: Done sleeping for 1 second");

    // Publish dynamic obstacles
    pub_obs.publish(tc.ob_list);

    // Wait for planner to generate predicted trajectories
    //while(my_planner.ob_trajectory_[0].trajectory.points.size() < 1) {}
    

    tc.t_begin = ros::Time::now();
    
    // Create timer to continuously publish obstacle information
    ob_trj_timer = handle.createTimer(ros::Duration(1./20.), boost::bind(pubObTrj, _1, tc));



    /*
     * Wait for planner to run for time threshold
     */
    while(start_tc)
    {
      handle.getParam("ramp/ready_tc", start_tc);
      ROS_INFO("generate_test_case: start_tc: %s", start_tc ? "True" : "False");
      r.sleep();
      ros::spinOnce();
    }

    ROS_INFO("generate_test_case: Test case completed");

    // Set flag signifying that the next test case is not ready
    ros::param::set("/ramp/tc_generated", false);
    
  } // end for








  /*ROS_INFO("Num tests: %d Num success: %d Percent: %d", num_tests, num_successful_tests, (num_successful_tests / 
        num_tests*10));
  
  int count = num_generations[0];
  for(int i=1;i<num_generations.size();i++)
  {
    count+=num_generations[i];
  }
  ROS_INFO("Average number of planning cycles: %f", (float)count / num_generations.size());*/


  std::cout<<"\n\nExiting Normally\n";
  ros::shutdown();
  return 0;
}
