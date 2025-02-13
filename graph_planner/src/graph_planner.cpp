#include <pluginlib/class_list_macros.h>
#include "../include/graph_planner/graph_planner.h"
#include "path_planning/MapGraph.h"
#include <iostream>

#include <global_planner/astar.h>

 //register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(graph_planner::GraphPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

 //Default Constructor
namespace graph_planner {
    GraphPlanner::GraphPlanner (){
        ROS_DEBUG("This is test msg for GraphPlanner(no attr)");
        std::cout << "This is test output(GraphPlanner(no attr))" << std::endl;
    }

    GraphPlanner::GraphPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
        ROS_DEBUG("This is test msg for GraphPlanner");
        std::cout << "This is test output(GraphPlanner)" << std::endl;
        initialize(name, costmap_ros);
    }


    void GraphPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
        if(!initialized_){
            std::cout << "This is test output(init)" << std::endl;
            ROS_DEBUG("This is test output(init)");
            ros::NodeHandle private_nh("~/" + name);

            ros::ServiceClient client = private_nh.serviceClient<path_planning::MapGraph>("/map_server");

            path_planning::MapGraph map_srv;

            map_srv.request.file_path = "/home/ros/SCV2/src/scv_system/global_path/ROS_PathPlanning_pkg/data/graph(map)/20250115_k-city.json";

            if (client.call(map_srv)) {
                ROS_INFO("success");
                std::cout << map_srv.response.map_graph.node_array.nodes[0].ID << endl;
            } else {
                ROS_ERROR("error");
            }

            initialized_ = true;
        }
    }

    bool GraphPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan) {
        ROS_DEBUG("This is test output(makeplan)");
    }
};