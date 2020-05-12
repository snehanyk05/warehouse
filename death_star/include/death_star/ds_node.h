#ifndef INCLUDE_DS_NODE_H_
#define INCLUDE_DS_NODE_H_

#include <ros/ros.h>
#include <vector>
#include <queue>
#include <string>
#include <map>
#include "dynamic_global_planner/graph_node.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "death_star/smartPlan.h"
#include "dynamic_global_planner/Graph.h"
#include "dynamic_global_planner/Node.h"
#include "dynamic_global_planner/Neighbour.h"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "std_msgs/Bool.h"


class DeathStar
{
private:

public:

    /**
     * @brief Construct a new Death Star object
     * 
     */
	DeathStar();

    /**
     * @brief Construct a new Death Star object
     * 
     * @param n Node handle of that particular robot
     * @param nss Name space used by that robot
     */
	DeathStar(ros::NodeHandle& n, std::string nss)
	{
		ROS_INFO("DS Created");
		nh = n;
		ns = nss;
		path_service = nh.advertiseService("gen_path", &DeathStar::PathGenerator, this);

		graph_sub = nh.subscribe < dynamic_global_planner::Graph> ("/graph_topic", 10, &DeathStar::graph_callback, this);
		path_cost_pub = nh.advertise<std_msgs::Bool>("path_cost", 1);
		ROS_INFO("DS Created");
	}
	std::string ns;
	ros::ServiceServer path_service;

    /**
     * @brief Service server which generates path using Death* method
     * 
     * @param req Start and Goal position 
     * @param resp Path generated in the form of nav_msgs/Path
     * @return true If the service is completed successfully
     * @return false If the service is not completed successfully
     */
	bool PathGenerator(death_star::smartPlan::Request &req, death_star::smartPlan::Response &resp);
 
    /**
     * @brief Get the Eucledian Distance
     * 
     * @param x1 x co-ordinate of point1
     * @param y1 y co-ordinate of point1
     * @param x2 x co-ordinate of point2
     * @param y2 y co-ordinate of point2
     * @return float 
     */
	float getEucledianDistance(float x1, float y1, float x2, float y2);

    /**
     * @brief Method that computes the path
     * 
     * @param x_start x co-ordinate of start
     * @param y_start y co-ordinate of start
     * @param x_goal x co-ordinate of goal
     * @param y_goal y co-ordinate of goal
     */
	void findShortestPath(float x_start, float y_start, float x_goal, float y_goal);

    /**
     * @brief Find the nearest node in the mesh to the given point 
     * 
     * @param x x co-ordinate of the point
     * @param y y co-ordinate of the point
     * @return Node* Nearest node in the mesh 
     */
	Node* findNearestNode(float x, float y);

	/**
	 * @brief Debug method to draw path/mesh on the warehouse image
	 * 
	 */
	void drawGraphonImage();

	/**
	 * @brief Method to find the cost of the current path
	 * 
	 * @param called_by_service This boolean determines if we are finding the path length for the first time or
	 * 							calling this method to compare the path cost with the previously found path cost
	 */
	void findPathCost(bool called_by_service);

	std::vector<Node*> graph; // Local copy of the graph
	std::vector<Node*> graph_s; // Local copy of the graph
	std::map<std::tuple<float, float, float>, std::vector<std::tuple<float, float, float>>> graph_dict; 
	std::map<std::tuple<float, float, float>, std::vector<std::tuple<float, float, float>>> graph_dict_s;
	std::vector<std::tuple<float, float, float>> curr_path;
	std::map<std::tuple<float, float>, float> weight_storage;
	std::map<std::tuple<float, float>, float> weight_storage_s;
	float path_cost = 0;
	float path_cost_init = 0;
	ros::NodeHandle nh;

	ros::Subscriber graph_sub;
	ros::Publisher path_cost_pub;
	bool subscriber_callback_executing;

	/**
	 * @brief Subscriber callback to update the mesh used for global planner
	 * 
	 * @param msg Information of mesh nodes
	 */
	void graph_callback(const dynamic_global_planner::Graph::ConstPtr& msg);
};

#endif // INCLUDE_DS_NODE_H_
