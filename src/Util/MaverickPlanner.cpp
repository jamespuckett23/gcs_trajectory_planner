#include "../include/gcs_trajectory_planner/MaverickPlanner.h"


class AStarSolver {
public:
    std::vector<geometry_msgs::msg::PoseStamped> solve(GraphMav& graph_input, int startId, int goalId) {
        graph = graph_input;
        std::priority_queue<NodeInfoMav, std::vector<NodeInfoMav>, std::greater<>> openSet;
        std::unordered_map<int, double> gCost; // Cost from start to each node
        std::unordered_map<int, int> cameFrom; // For reconstructing path

        // Initialize costs
        for (const auto& [id, _] : graph.nodes) {
            gCost[id] = std::numeric_limits<double>::infinity();
        }
        gCost[startId] = 0.0;

        openSet.emplace(startId, heuristic(graph.nodes[startId], graph.nodes[goalId]));

        while (!openSet.empty()) {
            int current = openSet.top().id;
            openSet.pop();

            if (current == goalId) {
                return reconstructPath(cameFrom, startId, goalId);
            }

            for (const auto& edge : graph.adjacencyList[current]) {
                double tentativeG = gCost[current] + edge.weight;
                if (tentativeG < gCost[edge.to]) {
                    cameFrom[edge.to] = current;
                    gCost[edge.to] = tentativeG;
                    double fCost = tentativeG + heuristic(graph.nodes[edge.to], graph.nodes[goalId]);
                    openSet.emplace(edge.to, fCost);
                }
            }
        }

        return {}; // Return empty path if no solution
    }

private:
    GraphMav graph;
    double heuristic(const geometry_msgs::msg::PoseStamped& a,
                            const geometry_msgs::msg::PoseStamped& b) {
        return GraphMav::calculateDistance(a, b);
    }

    std::vector<geometry_msgs::msg::PoseStamped> reconstructPath(const std::unordered_map<int, int>& cameFrom,
                                            int start, int goal) {
        std::vector<int> path;
        for (int current = goal; current != start; current = cameFrom.at(current)) {
            path.push_back(current);
        }
        path.push_back(start);
        std::reverse(path.begin(), path.end());

        std::vector<geometry_msgs::msg::PoseStamped> trajectory;
        for (auto pt : path) {
            trajectory.push_back(graph.nodes[pt]);
            std::cout << "mav waypoints: " << graph.nodes[pt].pose.position.x << ", " << graph.nodes[pt].pose.position.y << std::endl;
        }

        return trajectory;
    }
};


struct Node {
    double position_x;
    double position_y;
    int parent; // Index of the parent node in the tree
    double cost; // Cost to reach this node
};

class RRTStar {
private:
    std::vector<Node> tree; // Tree of nodes
    double step_size;       // Maximum step size
    int width, height;      // Dimensions of the cost map
    double resolution;
    nav_msgs::msg::OccupancyGrid grid; // ROS2 OccupancyGrid
    std::default_random_engine rng;
    cv::Point2d start_, goal_;

public:
    RRTStar(const nav_msgs::msg::OccupancyGrid& grid, double step_size)
        : grid(grid), step_size(step_size) {
        width = grid.info.width;
        height = grid.info.height;
        resolution = grid.info.resolution;
        rng.seed(std::random_device()());
    }

    std::vector<cv::Point2d> plan(const cv::Point2d& start, const cv::Point2d& goal, int max_iterations) {
        start_ = start;
        goal_ = goal;
        std::cout << "size of tree: " << tree.size() << std::endl;
        std::cout << "start: " << start << std::endl;
        std::cout << "goal: " << goal << std::endl;

        tree.reserve(100000);
        tree.push_back(Node());
        tree[0].parent = -1;
        tree[0].cost = 0.0;
        tree[0].position_x = start_.x;
        tree[0].position_y = start_.y;

        std::cout << "start: [" << tree[0].position_x << ", " << tree[0].position_y
            << "], parent: " << tree[0].parent << ", cost: " << tree[0].cost << std::endl;

        for (int i = 0; i < max_iterations; ++i) {
            // 1. Sample a random point
            cv::Point2d random_point = sample();

            // 2. Find the nearest node in the tree
            int nearest_index = findNearest(random_point);
            cv::Point2d nearest_point = cv::Point2d(tree[nearest_index].position_x, tree[nearest_index].position_y);

            // 3. Steer towards the random point
            cv::Point2d new_point = steer(nearest_point, random_point);

            // 4. Check if the new point is traversable
            if (!isTraversable(new_point)) continue;

            // 5. Add the new point to the tree
            double cost_to_new = tree[nearest_index].cost + cv::norm(new_point - nearest_point);
            if (new_point.x == 0.0 || new_point.y == 0.0) {
                continue;
            }
            int new_index = addNode(new_point, nearest_index, cost_to_new);

            // 6. Rewire the tree
            rewire(new_index);

            // Check if goal is reached
            if (cv::norm(new_point - goal_) <= 0.2) {
                addNode(goal_, new_index, tree[new_index].cost + cv::norm(new_point - goal_));
                break;
            }
        }

        // Construct the path
        return constructPath(goal);
    }

private:
    cv::Point2d sample() {
        double buffer = sqrt(pow(start_.x - goal_.x, 2) + pow(start_.y - goal_.y, 2))/4.0;
        
        std::uniform_real_distribution<double> x_dist(0.0, width);
        std::uniform_real_distribution<double> y_dist(0.0, height);

        return cv::Point2d(round(x_dist(rng)/0.2) * 0.2, round(y_dist(rng)/0.2) * 0.2);
    }

    int findNearest(const cv::Point2d& point) {
        int nearest_index = -1;
        double min_distance = std::numeric_limits<double>::max();
        for (size_t i = 0; i < tree.size(); ++i) {
            double distance = cv::norm(cv::Point2d(tree[i].position_x, tree[i].position_y) - point);
            if (distance < min_distance) {
                min_distance = distance;
                nearest_index = i;
            }
        }
        return nearest_index;
    }

    cv::Point2d steer(const cv::Point2d& from, const cv::Point2d& to) {
        cv::Point2d direction = to - from;
        double length = cv::norm(direction);
        if (length > 0.2) {
            direction = direction * (0.2 / length);
        }
        // std::cout << "steer direction: " << direction << ", should always be less than 0.2" << std::endl;
        return from + direction;
    }

    bool isTraversable(const cv::Point2d point) {
        // switch this cast to correctly switching to grid coordinates
        int x_idx = static_cast<int>(point.x);
        int y_idx = static_cast<int>(point.y);
        if (x_idx < 0 || x_idx >= width || y_idx < 0 || y_idx >= height) return false;

        int index = y_idx * width + x_idx;
        // std::cout << "data index: " << grid.data[index] << std::endl;
        return grid.data[index] > 0; // 0 means obstacle in the OccupancyGrid
    }

    bool isClear(const cv::Point2d origin, const cv::Point2d goal) {
        double prev_x = origin.x;
        double prev_y = origin.y;
        double next_x = goal.x;
        double next_y = goal.y;

        // Calculate differences
        double dx = next_x - prev_x;
        double dy = next_y - prev_y;

        // Find the number of steps required
        double steps = std::max(std::abs(dx)/0.2, std::abs(dy)/0.2);

        // Calculate the increment for each step
        double x_inc = dx / steps;
        double y_inc = dy / steps;

        // Generate points along the line
        double x = prev_x;
        double y = prev_y;
        for (int i = 0; i <= steps; ++i) {
            if (!isTraversable(cv::Point2d(x,y))) return false;
            x += x_inc;
            y += y_inc;
        }

        return true;
    }

    int addNode(const cv::Point2d& position, int parent, double cost) {
        tree.push_back(Node());
        int index = tree.size() - 1;

        tree[index].position_x = position.x;
        tree[index].position_y = position.y;
        tree[index].cost = cost;
        tree[index].parent = parent;

        tree[0].parent = -1;
        tree[0].cost = 0.0;

        return index;
    }

    void rewire(int new_index) {
        Node& new_node = tree[new_index];
        for (size_t i = 1; i < tree.size(); ++i) {
            if (static_cast<int>(i) == new_index) continue;

            Node& other_node = tree[i];
            double distance = cv::norm(cv::Point2d(new_node.position_x, new_node.position_y) - cv::Point2d(other_node.position_x, other_node.position_y));
            double neighborhood_distance = 10.0;
            double new_cost = new_node.cost + distance;
            if (new_cost < other_node.cost && isClear(cv::Point2d(new_node.position_x, new_node.position_y), cv::Point2d(other_node.position_x, other_node.position_y)) && distance <= neighborhood_distance) {
                other_node.parent = new_index;
                other_node.cost = new_cost;
            }
        }
    }

    std::vector<cv::Point2d> constructPath(const cv::Point2d& goal) {
        std::vector<cv::Point2d> path;

        // Find the closest node to the goal
        int goal_index = -1;
        double min_distance = std::numeric_limits<double>::max();
        for (size_t i = 0; i < tree.size(); ++i) {
            double distance = cv::norm(cv::Point2d(tree[i].position_x, tree[i].position_y) - goal);
            if (distance < min_distance) {
                min_distance = distance;
                goal_index = i;
            }
        }

        // Trace back to the start
        path.push_back(cv::Point2d(goal_.x, goal_.y));
        while (goal_index != -1) {
            if (tree[goal_index].position_x == 0.0 && tree[goal_index].position_y == 0.0) {
                if (goal_index == -1) {
                    std::cout << "location of 0,0 and goal_index of -1" << std::endl;
                }
                std::cout << "location of 0,0 at: " << goal_index << std::endl;
                std::cout << "position: " << tree[goal_index].position_x << ", " << tree[goal_index].position_y << std::endl;
                std::cout << "parents: " << tree[goal_index].parent << ", cost: " << tree[goal_index].cost << std::endl;
                // goal_index = tree[goal_index].parent;
                // continue;
            }
            path.push_back(cv::Point2d(tree[goal_index].position_x, tree[goal_index].position_y));
            std::cout << "constructPath-position: " << cv::Point2d(tree[goal_index].position_x, tree[goal_index].position_y) << ", parent: " << tree[goal_index].parent << ", cost: " << tree[goal_index].cost << std::endl;
            goal_index = tree[goal_index].parent;
            // if (goal_index == 0) {
            //     path.push_back(cv::Point2d(tree[goal_index].position_x, tree[goal_index].position_y));
            //     // std::cout << "constructPath-position: " << cv::Point2d(tree[goal_index].position_x, tree[goal_index].position_y) << ", parent: " << tree[goal_index].parent << ", cost: " << tree[goal_index].cost << std::endl;
            //     goal_index = -1;
            // }
        }
        path.push_back(cv::Point2d(start_.x, start_.y));

        auto location = std::find(path.begin(), path.end(), cv::Point2d(0, 0));

        if (location != path.end()) {
            path.erase(location);
            std::cout << "removed 0.0, 0.0" << std::endl;
        }

        std::reverse(path.begin(), path.end());
        return path;
    }
};


void MaverickPlanner::set_map(const nav_msgs::msg::OccupancyGrid &occupancy_grid_msg, const SemMap& SWM_input) {
    cost_map = occupancy_grid_msg;
    grid_width = occupancy_grid_msg.info.width;
    grid_height = occupancy_grid_msg.info.height;
    grid_resolution = 0.2;
    grid_origin_x = occupancy_grid_msg.info.origin.position.x;
    grid_origin_y = occupancy_grid_msg.info.origin.position.y;

    occupancy_grid = cv::Mat(grid_height, grid_width, CV_8UC1);
    for (int y = 0; y < grid_height; ++y) {
        for (int x = 0; x < grid_width; ++x) {
            occupancy_grid.at<uchar>(y, x) = occupancy_grid_msg.data[y * grid_width + x];
        }
    }

    SWM = SWM_input;
}

void MaverickPlanner::set_start_end(const geometry_msgs::msg::Pose &start_pose, const geometry_msgs::msg::Pose &end_pose) {
    start_location = cv::Point2f(start_pose.position.x, start_pose.position.y);
    end_location = cv::Point2f(end_pose.position.x, end_pose.position.y);
}

bool MaverickPlanner::is_point_traversable(const cv::Point2f &point) {
    int x_idx = static_cast<int>((point.x - grid_origin_x) / grid_resolution);
    int y_idx = static_cast<int>((point.y - grid_origin_y) / grid_resolution);

    if (x_idx >= 0 && x_idx < grid_width && y_idx >= 0 && y_idx < grid_height) {
        return occupancy_grid.at<uchar>(y_idx, x_idx) > 0;
    }
    return false;
}

void MaverickPlanner::generate_waypoints() {
    waypoints.clear();
    std::vector<geometry_msgs::msg::PoseStamped> map;
    
    // generate node-based map
    point_type start_point;
    start_point.set<0>(start_location.x);
    start_point.set<1>(start_location.y);
    point_type goal_point;
    goal_point.set<0>(end_location.x);
    goal_point.set<1>(end_location.y);

    GraphMav graph;
    int start_node;
    int goal_node;
    int index = 0;
    for (auto element : SWM.fm) {
        point_type centroid;
        boost::geometry::centroid(element.second.Polygon, centroid);

        geometry_msgs::msg::PoseStamped pt;
        pt.pose.position.x = centroid.get<0>();
        pt.pose.position.y = centroid.get<1>();

        if (element.second.nodeType->currentFeature.traversability.gv > 30) {
            graph.addNode(index, pt);
        }

        if (boost::geometry::covered_by(start_point, element.second.Polygon)) {
            start_node = index;
        }
        if (boost::geometry::covered_by(goal_point, element.second.Polygon)) {
            goal_node = index;
        }

        index += 1;
    }

    auto element_list = SWM.fm;
    int index_first = 0;
    for (auto element_first : element_list) {
        int index_second = 0;
        for (auto element_second : element_list) {
            if (element_first.first == element_second.first) {
                continue;
            }
            if (element_first.second.nodeType->currentFeature.traversability.gv > 0 && element_second.second.nodeType->currentFeature.traversability.gv > 0) {
                if (boost::geometry::intersects(element_first.second.Polygon, element_second.second.Polygon)) {
                    graph.addEdge(index_first, index_second);
                    graph.addEdge(index_second, index_first);
                }
            }
            index_second += 1;
        }
        index_first += 1;
    }

    auto solver = AStarSolver();
    std::vector<geometry_msgs::msg::PoseStamped> path = solver.solve(graph, start_node, goal_node);
    std::cout << "size of A* path: " << path.size() << std::endl;

    waypoints.push_back(start_location);

    for (auto pt : path) {
        cv::Point2f point;
        point.x = pt.pose.position.x;
        point.y = pt.pose.position.y;
        if (point.x == 0.0 && point.y == 0.0) continue;
        waypoints.push_back(point);
    }

    // waypoints.insert(waypoints.begin(), start_location);

    waypoints.erase(waypoints.begin()+1);
    waypoints.erase(waypoints.end());
    waypoints.push_back(end_location);
    
    for (size_t i = 0; i < waypoints.size(); i++)
    {
        waypoints[i].x = round(waypoints[i].x / 0.2) * 0.2;
        waypoints[i].y = round(waypoints[i].y / 0.2) * 0.2;
    }
}

std::optional<cv::Point2f> MaverickPlanner::get_next_traversable_point(const cv::Point2f &current_point) {
    std::vector<cv::Point2f> candidates = {
        {current_point.x + grid_resolution, current_point.y},
        {current_point.x - grid_resolution, current_point.y},
        {current_point.x, current_point.y + grid_resolution},
        {current_point.x, current_point.y - grid_resolution},
        {current_point.x + grid_resolution, current_point.y + grid_resolution},
        {current_point.x - grid_resolution, current_point.y - grid_resolution},
        {current_point.x + grid_resolution, current_point.y - grid_resolution},
        {current_point.x - grid_resolution, current_point.y + grid_resolution},
    };

    cv::Point2f best_point;
    double min_cost = std::numeric_limits<double>::infinity();
    bool found = false;

    for (const auto &candidate : candidates) {
        double cost = calculate_cost(candidate);
        if (cost < min_cost) {
            min_cost = cost;
            best_point = candidate;
            found = true;
        }
    }
    return found ? std::make_optional(best_point) : std::nullopt;
}

double MaverickPlanner::calculate_cost(const cv::Point2f &point) {
    int x_idx = static_cast<int>((point.x - grid_origin_x) / grid_resolution);
    int y_idx = static_cast<int>((point.y - grid_origin_y) / grid_resolution);

    uchar traversability = occupancy_grid.at<uchar>(y_idx, x_idx);
    double distance_to_goal = cv::norm(point - end_location);
    return (100 - traversability) * distance_to_goal;
}

nav_msgs::msg::Path MaverickPlanner::refine_path_with_rrt() {
    std::vector<cv::Point2f> refined_path;
    refined_path.push_back(start_location);

    for (size_t i = 0; i < waypoints.size(); ++i) {
        std::cout << "waypoint[" << i << "]: " << waypoints[i] << std::endl;
    }

    std::cout << "waypoints.size(): " << waypoints.size() << std::endl;
    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        std::vector<cv::Point2f> refined_segment = rrt(waypoints[i], waypoints[i + 1]);
        refined_path.insert(refined_path.end(), refined_segment.begin(), refined_segment.end());
    }

    nav_msgs::msg::Path maverick_path;
    for (auto pt : refined_path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = pt.x;
        pose.pose.position.y = pt.y;
        maverick_path.poses.push_back(pose);
    }

    return maverick_path;
}

std::vector<cv::Point2f> MaverickPlanner::rrt(const cv::Point2f &start, const cv::Point2f &end, int max_iterations, double step_size) {

    RRTStar planner(cost_map, 0.2); // step size
    std::vector<cv::Point2d> path = planner.plan(start, end, 20000); // max iterations
    std::cout << "planning between waypoints: " << start << ", and " << end << " resulted in: " << path.size() << " points" << std::endl;

    auto bresenham = [](cv::Point2f previous_point, cv::Point2f next_point) -> std::vector<cv::Point2f> {
        std::vector<cv::Point2f> path;
        double step_size = 0.2;

        double prev_x = previous_point.x;
        double prev_y = previous_point.y;
        double next_x = next_point.x;
        double next_y = next_point.y;

        double m_new = 2 * (next_y - prev_y);
        double slope_error_new = m_new - (next_x - prev_x);
        for (double x = prev_x, y = prev_y; x <= next_x; x += step_size) { 
            path.push_back(cv::Point2f(x, y));

            // Add slope to increment angle formed 
            slope_error_new += m_new; 
    
            // Slope error reached limit, time to 
            // increment y and update slope error. 
            if (slope_error_new >= 0) { 
                y += step_size;
                slope_error_new -= 2 * (next_x - prev_x);
            }
        }

        return path;
    };

    auto dda = [](cv::Point2f previous_point, cv::Point2f next_point) -> std::vector<cv::Point2f> {
        std::vector<cv::Point2f> path;

        double prev_x = previous_point.x;
        double prev_y = previous_point.y;
        double next_x = next_point.x;
        double next_y = next_point.y;

        // Calculate differences
        double dx = next_x - prev_x;
        double dy = next_y - prev_y;

        // Find the number of steps required
        double steps = std::max(std::abs(dx)/0.2, std::abs(dy)/0.2);

        // Calculate the increment for each step
        double x_inc = dx / steps;
        double y_inc = dy / steps;

        // Generate points along the line
        double x = prev_x;
        double y = prev_y;
        for (int i = 0; i <= steps; ++i) {
            path.emplace_back(cv::Point2f(x, y));
            x += x_inc;
            y += y_inc;
        }

        return path;
    };


    std::vector<cv::Point2f> path_float;
    path_float.push_back(cv::Point2f(path[0].x, path[0].y));
    for (int i = 1; i < path.size(); i++) {
        auto new_point = cv::Point2f(path[i].x, path[i].y);
        auto previous_point = cv::Point2f(path[i-1].x, path[i-1].y);

        auto next_path_segment = dda(previous_point, new_point);
        // auto next_path_segment = bresenham(previous_point, new_point);

        for (int j=0; j<next_path_segment.size(); j++) {
            path_float.push_back(next_path_segment[j]);
            // std::cout << "next_path_segment[j]: " << next_path_segment[j] << std::endl;
        }
        // path_float.push_back(new_point);
    }

    return path_float;
}

nav_msgs::msg::Path MaverickPlanner::run_planner() {
    generate_waypoints();
    return refine_path_with_rrt();
}
