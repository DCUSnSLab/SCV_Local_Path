/** include the libraries you need in your planner here */
/** for global path planner interface */

#ifndef GLOBAL_PLANNER_CPP
#define GLOBAL_PLANNER_CPP

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <string>
#include <vector>
#include <tuple>

#include "ros/ros.h"
#include "graph_planner/Graph.h"
#include "graph_planner/Node.h"
#include "graph_planner/NodeArray.h"
#include "graph_planner/Link.h"
#include "graph_planner/LinkArray.h"

using std::string;

namespace graph_planner {
    class GraphPlanner : public nav_core::BaseGlobalPlanner {
        public:
            GraphPlanner();
            GraphPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

            void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
            bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan);
        private:
            bool initialized_;

            std::pair<double, double> origin_utm_;      // UTM 좌표 원점 (x, y)
            std::pair<double, double> waypoint_relative_utm_;  // 상대 UTM 좌표 (x, y)

            std::tuple<double, double, double> map_gps_coordinates_;  // (위도, 경도, 고도)
            std::pair<double, double> map_utm_;  // (x, y)

            std::vector<Node> nodes_;
            std::vector<Link> links_;

            int utm_zone_;
    };
};
#endif