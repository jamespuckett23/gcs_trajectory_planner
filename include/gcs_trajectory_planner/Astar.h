// a_star_pathfinder.h
#ifndef A_STAR_PATHFINDER_H
#define A_STAR_PATHFINDER_H

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <optional>
#include <vector>
#include <utility>

std::optional<nav_msgs::msg::Path> aStarPathfinder(
    const nav_msgs::msg::OccupancyGrid& costmap, double start_x, double start_y, double goal_x, double goal_y);

#endif // A_STAR_PATHFINDER_HPP
