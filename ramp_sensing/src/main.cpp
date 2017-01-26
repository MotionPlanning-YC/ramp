#include <iostream>
#include <fstream>
#include <string>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "ramp_msgs/ObstacleList.h"
#include "obstacle.h"
#include "nav_msgs/OccupancyGrid.h"
#include "map_msgs/OccupancyGridUpdate.h"
#include "circle_packer.h"
#include <visualization_msgs/MarkerArray.h>
#include "utility.h"

Utility util;
double rate;
ros::Publisher pub_obj, pub_rviz;
std::vector< Obstacle> obs;
ramp_msgs::ObstacleList list;
std::vector< std::string > ob_odoms;
std::map< std::string, uint8_t > topic_index_map;
nav_msgs::OccupancyGrid global_grid;

std::vector<tf::Transform> ob_tfs;

std::vector<Circle> prev_cirs;

size_t prev_size;

ros::Time t_last_costmap;

int count;

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
        conf.push_back(std::stod(token));
        line.erase(0, pos+1);
      } // end inner while
    
      ROS_INFO("Last token: %s", line.c_str());

      conf.push_back(std::stod(line));

      tf::Transform temp;
      temp.setOrigin( tf::Vector3(conf.at(0), conf.at(1), 0));
      temp.setRotation(tf::createQuaternionFromYaw(conf.at(2)));

      ob_tfs.push_back(temp);
      
    } // end outter while
  } // end else


  ifile.close();
}




/** Get the other robot's current odometry information and update the obstacle info */
void updateOtherRobotCb(const nav_msgs::Odometry::ConstPtr& o, const std::string& topic) 
{
  //ROS_INFO("In updateOtherRobotCb");
  //ROS_INFO("topic: %s", topic.c_str());
  int index = topic_index_map[topic];
  //ROS_INFO("index: %i", index);
  
  if(obs.size() < index+1)
  {
    //ROS_INFO("In if obs.size() < index");
    Obstacle temp(*o);
    obs.push_back(temp);
    list.obstacles.push_back(temp.msg_);
  }
  else
  {
    //ROS_INFO("In else");
    obs.at(index).update(*o);
    list.obstacles.at(index) = obs.at(index).msg_;
  }
} //End updateOtherRobotCb




/** Publish the list of objects */
void publishList(const ros::TimerEvent& e) 
{
  //pub_obj.publish(obstacle.buildObstacleMsg());
  pub_obj.publish(list);
} //End sendList


std::vector<visualization_msgs::Marker> convertObsToMarkers()
{
  std::vector<visualization_msgs::Marker> result;
  
  double x_origin = global_grid.info.origin.position.x;
  double y_origin = global_grid.info.origin.position.y;
  
  //ROS_INFO("Before translate: x_origin: %f y_origin: %f", x_origin, y_origin);

  x_origin /= global_grid.info.resolution;
  y_origin /= global_grid.info.resolution;
  
  //ROS_INFO("After translate: x_origin: %f y_origin: %f", x_origin, y_origin);

  for(int i=0;i<obs.size();i++)
  {
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "/map";
    marker.ns = "basic_shapes";
    marker.id = i;
    
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    // This needs to be generalized to the costmap resolution
    double x = (obs[i].cir_.center.x + x_origin) * global_grid.info.resolution;
    double y = (obs[i].cir_.center.y + y_origin) * global_grid.info.resolution;
    
    //ROS_INFO("Before translation: (%f, %f) After translation: (%f, %f)", obs[i].cir_.center.x, obs[i].cir_.center.y, x, y);
    

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    double radius = obs[i].cir_.radius * global_grid.info.resolution;
    ROS_INFO("x: %f y: %f radius: %f", x, y, radius);
    
    obs[i].cir_.center.x = x;
    obs[i].cir_.center.y = y;
    obs[i].cir_.radius = radius;
    
    marker.scale.x = radius;
    marker.scale.y = radius;
    marker.scale.z = 0.1;
    marker.color.r = 0;
    marker.color.g = 1;
    marker.color.b = 0;
    marker.color.a = 1;
    marker.lifetime = ros::Duration(0.1);

    result.push_back(marker);
  }

  return result;
}

void publishMarkers(const ros::TimerEvent& e)
{
  
  // Publish a single Marker
  visualization_msgs::MarkerArray result;

  
  std::vector<visualization_msgs::Marker> markers = convertObsToMarkers();

  //ROS_INFO("Publishing %i markers", (int)markers.size());

  result.markers = markers;

  pub_rviz.publish(result);



  //ROS_INFO("markers.size(): %i", (int)markers.size());
  /*for(int i=0;i<result.markers.size();i++)
  {
    if(i==0 && result.markers.size() > 0)
    {
      ROS_INFO("result.markers.size(): %i", (int)result.markers.size());
      for(int j=0;j<result.markers.size();j++)
      {
        ROS_INFO("Circle %i scale: %f, %f", j, result.markers[j].scale.x, result.markers[j].scale.y);
      }
    }
  }*/
}


int getClosestPrev(Circle m, std::vector<Circle> N)
{
  int min_index = 0;

  std::vector<double> center;
  center.push_back(m.center.x);
  center.push_back(m.center.y);

  //ROS_INFO("center: [%f, %f]", center[0], center[1]);

  if(prev_cirs.size() > 0)
  {

    std::vector<double> prev_center;
    prev_center.push_back(N.at(0).center.x);
    prev_center.push_back(N.at(0).center.y);

    double dist = util.positionDistance(center, prev_center);
    
    //ROS_INFO("Prev center: [%f, %f]", prev_center[0], prev_center[1]);
    //ROS_INFO("dist: %f", dist);

    for(int i=1;i<N.size();i++)
    {
      prev_center.at(0) =  N[i].center.x;
      prev_center.at(1) =  N[i].center.y;

      if( util.positionDistance(center, prev_center) < dist )
      {
        dist =  util.positionDistance(center, prev_center);
        min_index = i;
      }
    }

    //ROS_INFO("min_index: %i dist: %f", min_index, dist);
  }

  else
  {
    return -1;
  }

  return min_index;
}

void costmapCb(const nav_msgs::OccupancyGridConstPtr grid)
{
  ros::Duration d_elapsed = ros::Time::now() - t_last_costmap;
  t_last_costmap = ros::Time::now();

  double grid_resolution = grid->info.resolution; 

  global_grid = *grid;
  //ROS_INFO("Got a new costmap!");
  CirclePacker c(grid);
  std::vector<Circle> cirs = c.go();
  
  std::vector<Circle> over = c.combineOverlappingCircles(cirs);
  ROS_INFO("over.size(): %i", (int)over.size());
  for(int i=0;i<over.size();i++)
  {
    ROS_INFO("Overlapping Circle %i - Center: (%f, %f) Radius: %f", i, over[i].center.x, over[i].center.y, over[i].radius);
  }


  //ROS_INFO("cirs.size(): %i obs.size(): %i", (int)cirs.size(), (int)obs.size());

  std::vector<int> cir_prev_cen_index;
  std::vector<double> linear_vs;
  std::vector<double> angular_vs;
  if(cirs.size() == prev_cirs.size() || cirs.size() < prev_cirs.size())
  {
    // Find closest previous circle for each new circle
    for(int i=0;i<cirs.size();i++)
    {
      int index = getClosestPrev(cirs[i], prev_cirs);
      ROS_INFO("Circle %i center: (%f,%f)", i, cirs[i].center.x, cirs[i].center.y);
      ROS_INFO("Closest prev: Circle %i center: (%f, %f)", i, prev_cirs[index].center.x, prev_cirs[index].center.y);
      cir_prev_cen_index.push_back(index);

      // Find velocities
      std::vector<double> pc;
      pc.push_back(prev_cirs[index].center.x);
      pc.push_back(prev_cirs[index].center.y);
      std::vector<double> cc;
      cc.push_back(cirs[i].center.x);
      cc.push_back(cirs[i].center.y);
      double theta = util.findAngleFromAToB(pc, cc);
      double linear_v = (util.positionDistance(pc, cc) / d_elapsed.toSec()) * grid_resolution;
      ROS_INFO("dist: %f time: %f linear_v: %f converted: %f", util.positionDistance(pc, cc), d_elapsed.toSec(), util.positionDistance(pc, cc) / d_elapsed.toSec(), linear_v);
    }
  }
  else
  {
    ROS_INFO("********************************************************************************");
    ROS_INFO("More new circles than previous circles!");
    // Find closest new circle for each prev center
    // Do this because the other way could match 2 new circles to the same prev center
    for(int i=0;i<prev_cirs.size();i++)
    {
      int index = getClosestPrev(prev_cirs[i], cirs);
      ROS_INFO("Circle %i center: (%f,%f)", i, cirs[i].center.x, cirs[i].center.y);
      ROS_INFO("Closest prev: Circle %i center: (%f, %f)", i, prev_cirs[i].center.x, prev_cirs[i].center.y);
      cir_prev_cen_index.push_back(index);
    }
    ROS_INFO("********************************************************************************");
  }
  
  // Set prev_cirs variable!
  prev_cirs = cirs;

  /*
   *  Compute Circle velocity
   */
  // Create Obstacle for each circle?

  // After finding velocities, populate Obstacle list
  obs.clear();

  for(int i=0;i<cirs.size();i++)
  {
    Obstacle o; 
    o.cir_ = cirs[i];
    obs.push_back(o);
  }

  for(int i=0;i<over.size();i++)
  {
    Obstacle o;
    o.cir_ = over[i];
    obs.push_back(o);
  }
 
  //ROS_INFO("obs.size(): %i", (int)obs.size());
  //ROS_INFO("Leaving Cb");
}


int main(int argc, char** argv) 
{
  ros::init(argc, argv, "ramp_sensing");
  ros::NodeHandle handle;
  
  //count = 0;

  //Get parameters
  
  /*std::string other_robot_odom;
  handle.getParam("ramp_sensing/other_robot_odom", other_robot_odom);
  std::cout<<"\nother_robot_odom:"<<other_robot_odom;*/

  /*if(handle.hasParam("/ramp/obstacle_odoms"))
  {
    ROS_INFO("Found rosparam obstacle_odoms");
    handle.getParam("/ramp/obstacle_odoms", ob_odoms);
    ROS_INFO("ob_odoms.size(): %i", (int)ob_odoms.size());
    for(int i=0;i<ob_odoms.size();i++)
    {
      ROS_INFO("ob_odoms[%i]: %s", i, ob_odoms.at(i).c_str());
      topic_index_map[ob_odoms.at(i)] = i;
    }
  }
  else
  {
    ROS_ERROR("ramp_sensing: Could not find obstacle_topics rosparam!");
  }

  if(handle.hasParam("/ramp/sensing_cycle_rate"))
  {
    handle.getParam("/ramp/sensing_cycle_rate", rate);
    ROS_INFO("Sensing cycle rate: %f", rate);
  }
  else
  {
    ROS_ERROR("ramp_sensing: Could not find sensing_cycle_rate rosparam!");
  }

  loadObstacleTF();

  // Create subscribers
  std::vector< ros::Subscriber > subs_obs;
  for(uint8_t i=0;i<ob_odoms.size();i++)
  {
    Obstacle temp;
    temp.T_w_init_ = ob_tfs[i];
    obs.push_back(temp);
    list.obstacles.push_back(temp.msg_);

    ros::Subscriber sub_ob = handle.subscribe<nav_msgs::Odometry>(ob_odoms.at(i), 1, boost::bind(updateOtherRobotCb, _1, ob_odoms.at(i)));
    subs_obs.push_back(sub_ob);
  } // end for*/

  ros::Subscriber sub_costmap = handle.subscribe<nav_msgs::OccupancyGrid>("/costmap_node/costmap/costmap", 1, &costmapCb);

  //Publishers
  pub_obj = handle.advertise<ramp_msgs::ObstacleList>("obstacles", 1);
  pub_rviz = handle.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

  //Timers
  //ros::Timer timer = handle.createTimer(ros::Duration(1.f / rate), publishList);
  ros::Timer timer_markers = handle.createTimer(ros::Duration(1.f/10.f), publishMarkers);
   

  std::cout<<"\nSpinning\n";
  ros::spin();
  std::cout<<"\nExiting Normally\n";
  return 0;
}
