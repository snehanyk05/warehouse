#include "ros/ros.h"
#include <iostream>
#include <string>
#include "nav_msgs/Odometry.h"
#include "death_star/ds_node.h"


int main(int argv, char **argc) {
  ros::init(argv, argc, "global_planner");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);
  std::string ns = ros::this_node::getNamespace();
  DeathStar dstar(nh, ns);
  float path_change_thresh = 4.0;
  ros::spinOnce();
  while (ros::ok())
  {
    std_msgs::Bool msg;
    
    dstar.findPathCost(false);

    if((dstar.path_cost - dstar.path_cost_init) > path_change_thresh)
    {
      
      msg.data = true;
      dstar.path_cost_init = dstar.path_cost;
    }
    else
    {
      msg.data = false;
    }
    dstar.path_cost_pub.publish(msg);
    ros::spinOnce();
  }
  return 0; 
}

 