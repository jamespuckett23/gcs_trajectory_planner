#include <../include/gcs_trajectory_planner/GCSPlanner.h>
#include <../include/gcs_trajectory_planner/GenerateSWM.h>
#include <../include/gcs_trajectory_planner/A*.h>
#include <../include/gcs_trajectory_planner/MaverickPlanner.h>
#include <chrono>
#include <thread>
#include <nav2_msgs/msg/costmap.h>
#include <nav_msgs/msg/occupancy_grid.h>
#include <nav_msgs/msg/path.h>


class PlannerNode : public rclcpp::Node
{
private:
    // path information
    std::vector<double> source_location;
    std::vector<double> goal_location;

    // global maps
    SemMap SWM;
    nav_msgs::msg::OccupancyGrid occupancy_grid;
    nav2_msgs::msg::Costmap cost_map;

    // ROS2 specific information
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_; // publishes the polygons (features) to RVIZ
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr graph_viz_publisher_; // publishes the graph viz string
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_; // publishes route for RVIZ
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr graph_based_path_publisher; // publishes route for RVIZ
    // rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr a_path_publisher_; // publishes route for RVIZ
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_publisher_; // publishes route for RVIZ
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr source_and_target_publisher_; // publishes starting and ending locations to RVIZ
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub;
    rclcpp::CallbackGroup::SharedPtr callback_group1_;
    rclcpp::CallbackGroup::SharedPtr callback_group2_;
    rclcpp::Subscription<sa_msgs::msg::SWM>::SharedPtr swm_sub;
    rclcpp::Service<sa_msgs::srv::SemanticMapInARegionServer>::SharedPtr visualize_swm_srv; // responds to a service request with the loaded SM
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<geometry_msgs::msg::PoseStamped> path;
    nav_msgs::msg::Path a_star_path;
    std::vector<geometry_msgs::msg::PoseStamped> graph_based_path;
    point_type origin;

    // rclcpp::Client<sa_msgs::srv::SemanticMapInARegionServer>::SharedPtr client;

    // publishers and subscribers
    // rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr cost_map_publisher_; // publishes cost map
    // rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr start_publisher_; // publishes start location
    // rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr goal_publisher_; // publishes goal location

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr maverick_path_subscriber_; // subscribes to maverick planner results

    std::vector<double> max_min;
    two_int_vect pgmimg;

    // sa_msgs::msg::SWM swm_msg;

    // void map_callback(nav_msgs::msg::OccupancyGrid msg);

    // void swm_callback(sa_msgs::msg::SWM msg);

    // void PublishPolygon();

    // void HandleService(const std::shared_ptr<sa_msgs::srv::SemanticMapInARegionServer::Request> request,
    //                    std::shared_ptr<sa_msgs::srv::SemanticMapInARegionServer::Response> response);

    // nav2_msgs::msg::Costmap createCostMapFromSWMRos(sa_msgs::msg::SWM swm_in);

    double ComputePathCost(const nav2_msgs::msg::Costmap& costmap, const std::vector<geometry_msgs::msg::PoseStamped>& path);

    // void maverick_path_callback(const nav_msgs::msg::Path::SharedPtr msg);

    std::vector<std::vector<double>> generateEndPoints();

    nav_msgs::msg::Path runMaverick(std::vector<double> source_location, std::vector<double> target_location);

    nav_msgs::msg::Path runA_star(std::vector<double> source_location, std::vector<double> target_location);

    nav_msgs::msg::Path runGCS(std::vector<double> source_location, std::vector<double> target_location);

    nav_msgs::msg::Path runMCTS_GCS(std::vector<double> source_location, std::vector<double> target_location);

    void getResults(nav_msgs::msg::Path path);

    void getResults(std::vector<nav_msgs::msg::Path> path);

    void getResults(std::vector<std::vector<nav_msgs::msg::Path>> path);

public:
    PlannerNode(const rclcpp::NodeOptions & options);

};


PlannerNode::PlannerNode(const rclcpp::NodeOptions & options) 
        : Node("planner_node", options)
{
    RCLCPP_INFO(this->get_logger(), "Planner Node has started");

    int vehicle_type = 0; // ground vehicle

    double length = 0.0;

    // 10 tests
    // three planners per test
    // stat for each planner/test
    //      path length
    //      path cost
    //      computation time
    //      path smoothness
    //      # of collisions
    //      Success

    struct PlanResults {
        double path_cost;
        double path_length;
        double computation_time;
        double path_sensitivity;
        int collisions;
        bool success;
    };

    // std::vector<std::vector<PlanResults>> stat_results;
    // std::vector<double> map_complexity;

    // int num_of_tests = 1;
    // stat_results.resize(2*num_of_tests);
    // map_complexity.reserve(static_cast<std::size_t>(num_of_tests));

    // load parameters for random map generator
    double width = 600.0;
    double height = 600.0;
    size_t num_polygons = 100;

    GenerateSWM swm_generator(width, height, num_polygons);

    // convert cost map to occupancy grid msg
    occupancy_grid.header = cost_map.header;
    occupancy_grid.info.width = cost_map.metadata.size_x;
    occupancy_grid.info.height = cost_map.metadata.size_y;
    occupancy_grid.info.resolution = cost_map.metadata.resolution;
    occupancy_grid.data.resize(cost_map.metadata.size_x * cost_map.metadata.size_y);
    for (size_t i = 0; i < cost_map.data.size(); ++i) {
        int cost = cost_map.data[i];
        occupancy_grid.data[i] = cost;
    }

    auto {start_location, goal_location} = generateEndPoints();

    nav_msgs::msg::Path maverick_path = runMaverick(start_location, goal_location);
    getResults(maverick_path);

    nav_msgs::msg::Path a_star_path = runA_star(start_location, goal_location);
    getResults(a_star_path);

    nav_msgs::msg::Path gcs_path = runGCS(start_location, goal_location);
    getResults(gcs_path);

    nav_msgs::msg::Path mcts_gcs_path = runMCTS_GCS(start_location, goal_location);
    getResults(mcts_gcs_path);

    // may need a way to record averaged results
    // i.e. 10+ runs each and compare total scores relative to each other

    return;

    for (int i = 0; i < num_of_tests; i++) {
        if (i > 9) {
            break;
        }

        stat_results[i].resize(3); // three types of planners to record information about 

        auto result = generateEndPoints();
        std::vector<double> start_location = result[0];
        std::vector<double> goal_location = result[0];

        // // generate map
        // SWM = swm_generator.getSWM();
        // cv::Mat swm_image = swm_generator.getSWMImage();
        // cost_map = swm_generator.getCostMap();

        // // get start and goal locations
        // double start_x;
        // double start_y;
        // double goal_x;
        // double goal_y;
        // bool good_start_point = false;
        // bool good_goal_point = false;

        // while (!(good_start_point && good_goal_point)) {
        //     std::cout << "inside while loop" << std::endl;
        //     std::random_device rd;
        //     std::mt19937 gen(rd());

        //     std::uniform_int_distribution<> distrib_x(0, width);
        //     std::uniform_int_distribution<> distrib_y(0, height);

        //     if (good_start_point == false) {
        //         start_x = distrib_x(gen);
        //         start_y = distrib_y(gen);
        //     }
        //     if (good_goal_point == false) {
        //         goal_x = distrib_x(gen);
        //         goal_y = distrib_y(gen);
        //     }

        //     double start_traversability = cost_map.data[round(start_x)*cost_map.metadata.size_y + round(start_y)];
        //     double goal_traversability = cost_map.data[round(goal_x)*cost_map.metadata.size_y + round(goal_y)];

        //     if (start_traversability > 0) {
        //         good_start_point = true;
        //         std::cout << "Start traversability: " << start_traversability << std::endl;
        //     }
        //     if (goal_traversability > 0) {
        //         good_goal_point = true;
        //         std::cout << "Goal traversability: " << goal_traversability << std::endl;
        //     }
        // }


        // set up parameters
        double path_cost;
        double path_length;
        double computation_time;
        double path_sensitivity;
        int collisions;
        bool success;

        cv::Mat swm_image_with_path;

        // plan GCS planner
        try {
            auto start_time = this->get_clock()->now();

            // plan GCS planner
            source_location[0] = start_x;
            source_location[1] = start_y;
            target_location[0] = goal_x;
            target_location[1] = goal_y;
            auto planner = std::make_unique<TrajectoryPlanner>(SWM, vehicle_type, source_location, target_location);

            // get metrics
            auto end_time = this->get_clock()->now();
            auto gcs_time = end_time - start_time;
            computation_time = gcs_time.seconds();

            this->path = planner->getTrajectory();

            length = planner->getPathLength();
            double gcs_cost = ComputePathCost(cost_map, path);
            double gcs_length = swm_generator.getPathLength(path);
            std::cout << "gcs_length: " << gcs_length << std::endl;
            if (std::isnan(gcs_length)) {
                gcs_length = length;
                std::cout << "gcs_length was nan now is: " << length << std::endl;
            }
            double gcs_sensitivity = swm_generator.getPathCurvature(path);

            swm_image_with_path = swm_generator.getSWMImage(path);

            PlanResults planresults;
            stat_results[i][0] = planresults;
            stat_results[i][0].path_cost = gcs_cost;
            stat_results[i][0].path_length = gcs_length;
            stat_results[i][0].path_sensitivity = gcs_sensitivity;
            stat_results[i][0].computation_time = computation_time;

            if (gcs_cost == -1 || gcs_length == -1) {
                i -= 1;
                continue;
            }
        }
        catch(const std::exception& e)
        {
            success = false;
            std::cout << "Maverick plan " << i << " failed with error: "  << e.what() << std::endl;
            i -= 1; // run 10 tests where the gcs planner is a success
            continue;
        }

        // plan A* planner

        // load information for A* planner
        PlanResults a_star_results;
        occupancy_grid.header = cost_map.header;
        occupancy_grid.info.width = cost_map.metadata.size_x;
        occupancy_grid.info.height = cost_map.metadata.size_y;
        occupancy_grid.info.resolution = cost_map.metadata.resolution;
        occupancy_grid.data.resize(cost_map.metadata.size_x * cost_map.metadata.size_y);
        for (size_t i = 0; i < cost_map.data.size(); ++i) {
            int cost = cost_map.data[i];
            occupancy_grid.data[i] = cost;
        }

        auto start_time = this->get_clock()->now();
        auto a_star_path = aStarPathfinder(occupancy_grid, start_x, start_y, goal_x, goal_y); // plan A* path
        auto end_time = this->get_clock()->now();
        auto a_star_time = end_time - start_time;
        computation_time = a_star_time.seconds();
        

        cv::Mat swm_image_with_Astar_path;
        double astar_cost;
        double astar_length;
        double astar_sensitivity;
        if (a_star_path.has_value()) {
            for (auto pt : *a_star_path) {
                std::cout << "(" << pt.pose.position.x << "," << pt.pose.position.y << "): " << unsigned(cost_map.data[round(pt.pose.position.x)*cost_map.metadata.size_y + round(pt.pose.position.y)]) << std::endl;
            }
            swm_image_with_Astar_path = swm_generator.addPathToImage(swm_image_with_path, *a_star_path, 2);
            // astar_cost = swm_generator.compute_path_cost(*a_star_path);
            astar_cost = ComputePathCost(cost_map, *a_star_path);
            astar_length = swm_generator.getPathLength(*a_star_path);
            astar_sensitivity = swm_generator.getPathCurvature(*a_star_path);

            stat_results[i][1] = PlanResults();
            stat_results[i][1].path_cost = astar_cost;
            stat_results[i][1].path_length = astar_length;
            stat_results[i][1].path_sensitivity = astar_sensitivity;
            stat_results[i][1].computation_time = computation_time;
        }

        bool save_a_star_path = true;
        if (save_a_star_path) {
            std::string filename_ = "swm_image_with_A_star_path.png";
            cv::imwrite(filename_, swm_image_with_Astar_path);
        }

        // plan Maverick planner
        PlanResults maverick_results;
        geometry_msgs::msg::Pose start_location;
        start_location.position.x = start_x;
        start_location.position.y = start_y;
        geometry_msgs::msg::Pose goal_location;
        goal_location.position.x = goal_x;
        goal_location.position.y = goal_y;

        // std::cout << "loading maverick planner" << std::endl;
        // auto maverick_planner = MaverickPlanner();
        // maverick_planner.set_map(occupancy_grid, SWM);
        // maverick_planner.set_start_end(start_location, goal_location);
        // start_time = this->get_clock()->now();
        // auto maverick_path = maverick_planner.run_planner();
        // end_time = this->get_clock()->now();
        // auto maverick_time = end_time - start_time;
        // computation_time = maverick_time.seconds();

        std::vector<geometry_msgs::msg::PoseStamped> maverick_path_vector;

        // for (auto pt : maverick_path) {
        //     geometry_msgs::msg::PoseStamped pose;
        //     pose.pose.position.x = pt.x;
        //     pose.pose.position.y = pt.y;
        //     maverick_path_vector.push_back(pose);
        // }

        cv::Mat swm_image_with_maverick_path;
        double maverick_cost;
        double maverick_length;
        double maverick_sensitivity;
        if (!maverick_path_vector.empty()) {
            swm_image_with_maverick_path = swm_generator.addPathToImage(swm_image_with_Astar_path, maverick_path_vector, 1);
            std::string filename_ = "swm_image_with_maverick_path.png";
            cv::imwrite(filename_, swm_image_with_maverick_path);

            // maverick_cost = swm_generator.compute_path_cost(maverick_path_vector);
            maverick_cost = ComputePathCost(cost_map, maverick_path_vector);
            maverick_length = swm_generator.getPathLength(maverick_path_vector);
            maverick_sensitivity = swm_generator.getPathCurvature(maverick_path_vector);

            // std::cout << "maverick cost: " << maverick_cost << std::endl;
            // std::cout << "maverick length: " << maverick_length << std::endl;
            // std::cout << "maverick sensitivity: " << maverick_sensitivity << std::endl;

            if (maverick_cost == -1 || maverick_length == -1) {
                i -= 1;
                continue;
            }

            stat_results[i][2] = PlanResults();
            stat_results[i][2].path_cost = maverick_cost;
            stat_results[i][2].path_length = maverick_length;
            stat_results[i][2].path_sensitivity = maverick_sensitivity;
            stat_results[i][2].computation_time = computation_time;
        }

        // record
        std::cout << "Completed cycle: " << i << std::endl;

    }

    double average_gcs_computation_time = 0.0;
    double average_gcs_length = 0.0;
    double average_gcs_cost = 0.0;
    double average_gcs_sensitivity = 0.0;

    double average_a_star_computation_time = 0.0;
    double average_a_star_length = 0.0;
    double average_a_star_cost = 0.0;
    double average_a_star_sensitivity = 0.0;

    double average_maverick_computation_time = 0.0;
    double average_maverick_length = 0.0;
    double average_maverick_cost = 0.0;
    double average_maverick_sensitivity = 0.0;

    std::cout << "length of stat results: " << stat_results.size() << std::endl;

    for (int i=0; i < stat_results.size()-1; i++) {
        if (i > 9) {
            break;
        }
        std::cout << "Start of next result segment" << std::endl;

        // std::cout << stat_results[i][0].computation_time << std::endl;
        // std::cout << stat_results[i][0].path_length << std::endl;
        // std::cout << stat_results[i][0].path_cost << std::endl;
        // std::cout << stat_results[i][0].path_sensitivity << std::endl;
        average_gcs_computation_time += stat_results[i][0].computation_time;
        average_gcs_length += stat_results[i][0].path_length;
        average_gcs_cost += stat_results[i][0].path_cost;
        average_gcs_sensitivity += stat_results[i][0].path_sensitivity;

        // std::cout << stat_results[i][1].computation_time << std::endl;
        // std::cout << stat_results[i][1].path_length << std::endl;
        // std::cout << stat_results[i][1].path_cost << std::endl;
        // std::cout << stat_results[i][1].path_sensitivity << std::endl;
        average_a_star_computation_time += stat_results[i][1].computation_time;
        average_a_star_length += stat_results[i][1].path_length;
        average_a_star_cost += stat_results[i][1].path_cost;
        average_a_star_sensitivity += stat_results[i][1].path_sensitivity;

        // std::cout << stat_results[i][2].computation_time << std::endl;
        // std::cout << stat_results[i][2].path_length << std::endl;
        // std::cout << stat_results[i][2].path_cost << std::endl;
        // std::cout << stat_results[i][2].path_sensitivity << std::endl;
        average_maverick_computation_time += stat_results[i][2].computation_time;
        average_maverick_length += stat_results[i][2].path_length;
        average_maverick_cost += stat_results[i][2].path_cost;
        average_maverick_sensitivity += stat_results[i][2].path_sensitivity;
    }

    std::cout << "Average computation time:" << std::endl;
    std::cout << "GCS: " << average_gcs_computation_time/num_of_tests << std::endl;
    std::cout << "A Star: " << average_a_star_computation_time/num_of_tests << std::endl;
    std::cout << "Maverick: " << average_maverick_computation_time/num_of_tests << std::endl;
    std::cout << "Average path cost:" << std::endl;
    std::cout << "GCS: " << average_gcs_cost/num_of_tests << std::endl;
    std::cout << "A Star: " << average_a_star_cost/num_of_tests << std::endl;
    std::cout << "Maverick: " << average_maverick_cost/num_of_tests << std::endl;
    std::cout << "Average path length:" << std::endl;
    std::cout << "GCS: " << average_gcs_length/num_of_tests << std::endl;
    std::cout << "A Star: " << average_a_star_length/num_of_tests << std::endl;
    std::cout << "Maverick: " << average_maverick_length/num_of_tests << std::endl;
    std::cout << "Average path sensitivity:" << std::endl;
    std::cout << "GCS: " << average_gcs_sensitivity/num_of_tests << std::endl;
    std::cout << "A Star: " << average_a_star_sensitivity/num_of_tests << std::endl;
    std::cout << "Maverick: " << average_maverick_sensitivity/num_of_tests << std::endl;
}

std::vector<std::vector<double>> PlannerNode::generateEndPoints() {
    // generate map
    SWM = swm_generator.getSWM();
    cv::Mat swm_image = swm_generator.getSWMImage();
    cost_map = swm_generator.getCostMap();

    // get start and goal locations
    double start_x;
    double start_y;
    double goal_x;
    double goal_y;
    bool good_start_point = false;
    bool good_goal_point = false;

    while (!(good_start_point && good_goal_point)) {
        std::cout << "inside while loop" << std::endl;
        std::random_device rd;
        std::mt19937 gen(rd());

        std::uniform_int_distribution<> distrib_x(0, width);
        std::uniform_int_distribution<> distrib_y(0, height);

        if (good_start_point == false) {
            start_x = distrib_x(gen);
            start_y = distrib_y(gen);
        }
        if (good_goal_point == false) {
            goal_x = distrib_x(gen);
            goal_y = distrib_y(gen);
        }

        double start_traversability = cost_map.data[round(start_x)*cost_map.metadata.size_y + round(start_y)];
        double goal_traversability = cost_map.data[round(goal_x)*cost_map.metadata.size_y + round(goal_y)];

        if (start_traversability > 0) {
            good_start_point = true;
            std::cout << "Start traversability: " << start_traversability << std::endl;
        }
        if (goal_traversability > 0) {
            good_goal_point = true;
            std::cout << "Goal traversability: " << goal_traversability << std::endl;
        }
    }

    return {{start_x, start_y}, {end_x, end_y}};
}

nav_msgs::msg::Path PlannerNode::runMaverick(std::vector<double> start_location, std::vector<double> goal_location) {
    nav_msgs::msg::Path path;

    // initialize
    auto maverick_planner = MaverickPlanner();
    maverick_planner.set_map(occupancy_grid, SWM);
    maverick_planner.set_start_end(start_location, goal_location);

    start_time = this->get_clock()->now();

    auto maverick_path = maverick_planner.run_planner();
    
    end_time = this->get_clock()->now();
    auto maverick_time = end_time - start_time;
    computation_time = maverick_time.seconds();

    return maverick_path;
}

nav_msgs::msg::Path PlannerNode::runA_star(std::vector<double> start_location, std::vector<double> goal_location) {
    nav_msgs::msg::Path path;

    auto start_time = this->get_clock()->now();

    auto a_star_path = aStarPathfinder(occupancy_grid, start_x, start_y, goal_x, goal_y); // plan A* path

    auto end_time = this->get_clock()->now();
    auto a_star_time = end_time - start_time;
    computation_time = a_star_time.seconds();

    return a_star_path;
}

nav_msgs::msg::Path PlannerNode::runGCS(std::vector<double> start_location, std::vector<double> goal_location) {
    auto start_time = this->get_clock()->now();

    auto gcs_planner = std::make_unique<GCSPlanner>(SWM, vehicle_type, start_location, goal_location);

    auto end_time = this->get_clock()->now();
    auto gcs_time = end_time - start_time;
    computation_time = gcs_time.seconds();

    return gcs_planner->getTrajectory();
}

nav_msgs::msg::Path PlannerNode::runMCTS_GCS(std::vector<double> start_location, std::vector<double> goal_location) {
    nav_msgs::msg::Path path;
    return path;
}

void PlannerNode::getResults(nav_msgs::msg::Path path) {
    std::vector<nav_msgs::msg::Path> paths;
    paths.push_back(path);
    getResults(paths);
}

void PlannerNode::getResults(std::vector<nav_msgs::msg::Path> paths) {
    for (nav_msgs::msg::Path path : paths) {
        double path_cost = 0.0;
        double path_length = 0.0;
        double path_curviture = 0.0;

        path_cost = swm_generator.getPathCost(path);
        path_length = swm_generator.getPathLength(path);
        path_curviture = swm_generator.getPathCurvature(path);
    }
}

void PlannerNode::getResults(std::vector<std::vector<nav_msgs::msg::Path>> path) {
    return std::vector<std::vector<nav_msgs::msg::Path>>;
}

// double PlannerNode::ComputePathCost(const nav2_msgs::msg::Costmap& costmap, const std::vector<geometry_msgs::msg::PoseStamped>& path) {
//     std::vector<geometry_msgs::msg::PoseStamped> swm_path = path;


//     auto get_row_major_index = [&costmap](int x_index, int y_index) -> int
//     {
//         int row_major_index = y_index*costmap.metadata.size_x + x_index;
//         return row_major_index;
//     };

//     auto convert_to_costmap = [&costmap](geometry_msgs::msg::PoseStamped location) -> std::pair<int,int>
//     {
//         int x = round((location.pose.position.x - costmap.metadata.origin.position.x) / costmap.metadata.resolution);
//         int y = round((location.pose.position.y - costmap.metadata.origin.position.y) / costmap.metadata.resolution);
//         return std::make_pair(x, y);
//     };

//     auto get_next_index = [&costmap, &convert_to_costmap](geometry_msgs::msg::PoseStamped next_location, int current_x, int current_y) -> std::pair<int,int>
//     {
//         int next_x, next_y;

//         auto resulting_index = convert_to_costmap(next_location);
//         int location_x = resulting_index.first;
//         int location_y = resulting_index.second;

//         // find the correct direction to move
//         int gradient_x = location_x - current_x;
//         int gradient_y = location_y - current_y;

//         // step in the correct direction
//         if (gradient_x != 0) {
//             next_x = current_x + (gradient_x > 0 ? 1 : -1); // move right or left
//         } else {
//             next_x = current_x; // no horizontal movement needed
//         }

//         if (gradient_y != 0) {
//             next_y = current_y + (gradient_y > 0 ? 1 : -1); // move up or down
//         } else {
//             next_y = current_y; // no vertical movement needed
//         }

//         return std::make_pair(next_x, next_y);
//     };


//     std::vector<std::pair<int,int>> cost_map_path;
//     bool at_goal_node = false;
//     double path_cost = 0.0;
//     int collisions = 0;
//     // const static vector<pair<int, int>> directions = {
//     //     {-1, -1}, {-1, 0}, {-1, 1},
//     //     { 0, -1},          { 0, 1},
//     //     { 1, -1}, { 1, 0}, { 1, 1}
//     // };  

//     geometry_msgs::msg::PoseStamped start_pose = path[0];
//     int start_x_index = round((start_pose.pose.position.x - costmap.metadata.origin.position.x) / costmap.metadata.resolution);
//     int start_y_index = round((start_pose.pose.position.y - costmap.metadata.origin.position.y) / costmap.metadata.resolution);

//     geometry_msgs::msg::PoseStamped goal_pose = path.back();
//     int goal_x_index = round((goal_pose.pose.position.x - costmap.metadata.origin.position.x) / costmap.metadata.resolution);
//     int goal_y_index = round((goal_pose.pose.position.y - costmap.metadata.origin.position.y) / costmap.metadata.resolution);

//     // path.erase(path.begin());
//     cost_map_path.push_back(std::make_pair(start_x_index, start_y_index));
//     // int previous_x_index = start_x_index;
//     // int previous_y_index = start_y_index;

//     int current_x_index = start_x_index;
//     int current_y_index = start_y_index;
//     swm_path.erase(swm_path.begin());

//     int next_x;
//     int next_y;

//     // create list of nodes that travel along the selected gcs path
//     while (!at_goal_node) {
//         // are we at the final location?
//         if (current_x_index == goal_x_index && current_y_index == goal_y_index) {
//             at_goal_node = true;
//         }

//         // find the next step
//         auto next_location = swm_path[0];
//         auto result_next_index = get_next_index(next_location, current_x_index, current_y_index);
//         next_x = result_next_index.first;
//         next_y = result_next_index.second;

//         // take the step
//         cost_map_path.push_back(std::make_pair(next_x, next_y));

//         // update
//         auto next_location_in_costmap = convert_to_costmap(next_location);
//         current_x_index = next_x;
//         current_y_index = next_y;
//         if (current_x_index == goal_x_index && current_y_index == goal_y_index) {
//                 at_goal_node = true;
//         }
//         if (next_location_in_costmap.first == next_x && next_location_in_costmap.second == next_y) {
//             swm_path.erase(swm_path.begin());
//         }

//     }

//     // compute cost of the path
//     int previous_index = get_row_major_index(cost_map_path[0].first, cost_map_path[0].second);
//     for (int i = 1; i < cost_map_path.size(); i++) {
//         int index = get_row_major_index(cost_map_path[i].first, cost_map_path[i].second);

//         // find step size (either 1 or sqrt(2))
//         double step_size = sqrt(pow(cost_map_path[i].first - cost_map_path[i-1].first,2) + pow(cost_map_path[i].second - cost_map_path[i-1].second,2));

//         // path_cost += step_size/allowed_speed;
//         double traversability = costmap.data[index];
//         double traversability_cost = 100 - traversability;

//         path_cost += step_size * traversability_cost;

//         // update
//         previous_index = index;
//     }

//     // record path


//     std::cout << "The SWM Optimization Path cost is: " << path_cost << " with " << collisions << " collisions" << std::endl;

//     return path_cost;
// }



int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto planner = std::make_shared<PlannerNode>(rclcpp::NodeOptions());
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(planner);

    RCLCPP_INFO(planner->get_logger(), "Starting client node, shut down with CTRL-C");
    executor.spin();
    RCLCPP_INFO(planner->get_logger(), "Keyboard interrupt, shutting down.\n");

    rclcpp::shutdown();

  return 0;
}
