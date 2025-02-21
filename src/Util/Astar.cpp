#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <queue>
#include <vector>
#include <cmath>
#include <unordered_map>
#include <optional>

#include "../../include/gcs_trajectory_planner/Astar.h"

struct Node {
    int x, y;
    float gCost;  // Cost from start to this node
    float fCost;  // gCost + heuristic
    bool operator>(const Node& other) const { return fCost > other.fCost; }
};

// Helper to get index from x, y coordinates with proper rounding
int getIndex(int x, int y, int width) {
    return y * width + x;
}

// Calculate heuristic, scaling by the resolution
float heuristic(int x, int y, int goal_x, int goal_y, float resolution) {
    float dist = std::sqrt(std::pow(x - goal_x, 2) + std::pow(y - goal_y, 2));
    return dist * resolution;
}

// A* pathfinding function
std::optional<nav_msgs::msg::Path> aStarPathfinder(const nav_msgs::msg::OccupancyGrid& costmap, double start_x, double start_y, double goal_x, double goal_y) {

    int width = costmap.info.width;
    int height = costmap.info.height;
    float resolution = costmap.info.resolution;

    std::vector<std::pair<int, int>> directions = {{1, 0}, {0, 1}, {-1, 0}, {0, -1},
                                                   {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openList;
    std::unordered_map<int, Node> allNodes;
    std::unordered_map<int, std::pair<int, int>> cameFrom;

    // Scale start and goal positions to grid coordinates
    int startX = static_cast<int>(std::round(start_x / resolution));
    int startY = static_cast<int>(std::round(start_y / resolution));
    int goalX = static_cast<int>(std::round(goal_x / resolution));
    int goalY = static_cast<int>(std::round(goal_y / resolution));

    int startIdx = getIndex(startX, startY, width);
    int goalIdx = getIndex(goalX, goalY, width);

    openList.push({startX, startY, 0, heuristic(startX, startY, goalX, goalY, resolution)});
    allNodes[startIdx] = {startX, startY, 0, heuristic(startX, startY, goalX, goalY, resolution)};

    while (!openList.empty()) {
        Node current = openList.top();
        openList.pop();
        int currentIdx = getIndex(current.x, current.y, width);

        if (current.x == goalX && current.y == goalY) {
            std::vector<geometry_msgs::msg::PoseStamped> path;
            while (currentIdx != startIdx) {
                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position.x = allNodes[currentIdx].x * resolution + costmap.info.origin.position.x;
                pose.pose.position.y = allNodes[currentIdx].y * resolution + costmap.info.origin.position.y;
                pose.pose.position.z = 0.0;
                pose.pose.orientation.w = 1.0; 

                path.push_back(pose);
                currentIdx = getIndex(cameFrom[currentIdx].first, cameFrom[currentIdx].second, width);
            }

            geometry_msgs::msg::PoseStamped startPose;
            startPose.pose.position.x = startX * resolution + costmap.info.origin.position.x;
            startPose.pose.position.y = startY * resolution + costmap.info.origin.position.y;
            startPose.pose.position.z = 0.0;
            startPose.pose.orientation.w = 1.0; 
            path.push_back(startPose);

            std::reverse(path.begin(), path.end());

            nav_msgs::msg::Path nav_path;
            nav_path.poses = path;
            return nav_path;
        }

        for (const auto& [dx, dy] : directions) {
            int neighborX = current.x + dx;
            int neighborY = current.y + dy;

            if (neighborX < 0 || neighborX >= width || neighborY < 0 || neighborY >= height) continue;
            int neighborIdx = getIndex(neighborX, neighborY, width);

            if (costmap.data[neighborIdx] < 1) continue;  // Skip untraversable cells

            float travel_cost = ((dx == 0 || dy == 0) ? 1.0f : std::sqrt(2.0f)) * resolution * (100 - costmap.data[neighborIdx]);
            float cost = current.gCost + travel_cost;

            if (!allNodes.count(neighborIdx) || cost < allNodes[neighborIdx].gCost) {
                allNodes[neighborIdx] = {neighborX, neighborY, cost, cost + heuristic(neighborX, neighborY, goalX, goalY, resolution)};
                openList.push(allNodes[neighborIdx]);
                cameFrom[neighborIdx] = {current.x, current.y};
            }
        }
    }

    return std::nullopt;  // Return empty if no path found
}
