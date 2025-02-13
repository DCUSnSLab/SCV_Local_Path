#include <pluginlib/class_list_macros.h>
#include "../include/graph_planner/graph_planner.h"
#include <iostream>

 // #include "graph_planner.h"

 //register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(graph_planner::GraphPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

 //Default Constructor
namespace graph_planner {
    GraphPlanner::GraphPlanner (){
        ROS_DEBUG("This is test msg for GraphPlanner");
        std::cout << "This is test output(GraphPlanner)" << std::endl;
    }

    GraphPlanner::GraphPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
        ROS_DEBUG("This is test msg for GraphPlanner");
        initialize(name, costmap_ros);
    }


    void GraphPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
        ROS_DEBUG("This is test msg for GraphPlanner");
        std::cout << "This is test output(init)" << std::endl;
    }

    bool GraphPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan) {
        ROS_DEBUG("This is test msg for GraphPlanner");
        std::cout << "This is test output(makeplan)" << std::endl;
    }
};