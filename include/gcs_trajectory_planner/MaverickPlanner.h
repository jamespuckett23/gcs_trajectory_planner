#ifndef MAVERICKPLANNER_H
#define MAVERICKPLANNER_H

#include <iostream>
#include <vector>
#include <cmath>
#include <optional>
#include <chrono>
#include <functional>
#include <memory>
#include <filesystem>
#include <fstream>
#include <algorithm> 

#include "rclcpp/rclcpp.hpp"
// #include "UTM.h"
// #include "read_data.h"
// #include "Planner.h"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <boost/geometry.hpp>

#include <../include/gcs_trajectory_planner/Common.h>

class MaverickPlanner {
public:
    MaverickPlanner() = default;

    void set_map(const nav_msgs::msg::OccupancyGrid &occupancy_grid_msg, const SemMap& SWM_input);

    void set_start_end(const geometry_msgs::msg::Pose &start_pose, const geometry_msgs::msg::Pose &end_pose);

    bool is_point_traversable(const cv::Point2f &point);

    void generate_waypoints();

    std::optional<cv::Point2f> get_next_traversable_point(const cv::Point2f &current_point);
    double calculate_cost(const cv::Point2f &point);

    nav_msgs::msg::Path refine_path_with_rrt();

    std::vector<cv::Point2f> rrt(const cv::Point2f &start, const cv::Point2f &end, int max_iterations = 100, double step_size = 1.0);

    nav_msgs::msg::Path run_planner();

private:
    cv::Mat occupancy_grid;
    nav_msgs::msg::OccupancyGrid cost_map;
    SemMap SWM;
    double grid_resolution;
    double grid_origin_x, grid_origin_y;
    int grid_width, grid_height;
    cv::Point2f start_location, end_location;
    std::vector<cv::Point2f> waypoints;
};


struct EdgeMav {
    int to;
    double weight;
    EdgeMav(int target, double edgeWeight) : to(target), weight(edgeWeight) {}
};

struct GraphMav {
    std::unordered_map<int, geometry_msgs::msg::PoseStamped> nodes; // Node ID to PoseStamped
    std::unordered_map<int, std::vector<EdgeMav>> adjacencyList;      // Adjacency list

    void addNode(int id, const geometry_msgs::msg::PoseStamped& pose) {
        nodes[id] = pose;
    }

    void addEdge(int from, int to) {
        double weight = calculateDistance(nodes[from], nodes[to]);
        adjacencyList[from].emplace_back(to, weight);
    }

    static double calculateDistance(const geometry_msgs::msg::PoseStamped& a,
                                    const geometry_msgs::msg::PoseStamped& b) {
        double dx = a.pose.position.x - b.pose.position.x;
        double dy = a.pose.position.y - b.pose.position.y;
        return std::sqrt(dx * dx + dy * dy);
    }
};

struct NodeInfoMav {
    int id;
    double cost; // f(n) = g(n) + h(n)
    NodeInfoMav(int nodeId, double totalCost) : id(nodeId), cost(totalCost) {}
    bool operator>(const NodeInfoMav& other) const {
        return cost > other.cost;
    }
};


#endif // MAVERICKPLANNER_H