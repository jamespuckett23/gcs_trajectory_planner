#ifndef GCSPLANNER_H
#define GCSPLANNER_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <filesystem>
#include <fstream>
#include <algorithm> 

#include "rclcpp/rclcpp.hpp"
#include <boost/geometry/index/rtree.hpp>
#include <opencv2/opencv.hpp>

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include "sa_msgs/srv/semantic_map_in_a_region_server.hpp"

#include <drake/planning/trajectory_optimization/gcs_trajectory_optimization.h>
#include <drake/planning/distance_and_interpolation_provider.h>
#include <drake/geometry/optimization/graph_of_convex_sets.h>
#include <drake/geometry/optimization/hpolyhedron.h>
#include <drake/geometry/optimization/point.h>
#include <drake/geometry/shape_specification.h>
#include <drake/geometry/optimization/vpolytope.h>
#include <drake/common/symbolic/expression/expression.h>
#include <drake/common/symbolic/expression/variable.h>
#include <drake/common/copyable_unique_ptr.h>
#include <drake/common/trajectories/composite_trajectory.h>
#include <drake/solvers/mathematical_program_result.h>
#include <drake/solvers/cost.h>
#include <drake/solvers/clarabel_solver.h>
#include <drake/geometry/optimization/iris.h>

#include <../include/gcs_trajectory_planner/Common.h>

typedef boost::geometry::model::d2::point_xy<double> boost_point;
typedef boost::geometry::model::polygon<boost_point> Polygon;
typedef boost::geometry::model::box<boost_point> Box;
typedef std::pair<Box, Polygon> boost_value;
typedef std::pair<polygon, unsigned> value;

using namespace drake::planning::trajectory_optimization;
using namespace drake::geometry::optimization;
using namespace drake::symbolic;
using namespace std::chrono_literals;


enum VehicleType {
    GroundVehicle, // 0
    AeralVehicle,  // 1
    Pedestrian     // 2
};

struct SWM_Feature {
    drake::geometry::optimization::GraphOfConvexSets::Vertex* gcs_vertex;
    drake::geometry::optimization::VPolytope vertex;
    drake::planning::trajectory_optimization::GcsTrajectoryOptimization::Subgraph* region;
    std::pair<std::string, NodeInfo> element;
    std::string name;
    int name_int;
    boost::geometry::model::polygon<point_type> polygon;
};



class GCSPlanner
{
private:
    // input values for the class
    std::vector<double> source_location;
    std::vector<double> target_location;
    int vehicle_type;
    double vehicle_radius;

    SemMap SM; // general Semantic World Model object
    GcsTrajectoryOptimization gcs;
    GcsTrajectoryOptimization::Subgraph* source_region; // starting location converted to graph coordinates
    GcsTrajectoryOptimization::Subgraph* target_region; // ending location converted to graph coordinates
    GcsTrajectoryOptimization::Subgraph* main_region;

    // parameters for the main region in the GCS optimizer
    int bezier_curve_order;
    double h_min;
    double h_max;

    std::vector<geometry_msgs::msg::PoseStamped> path;
    double pathLength;

    // From the SWM, create a gcs
    // Three regions (source, main, target)
    // The main region includes each vertex from the SWM
    // also loads terrain costs and constraints into the graph
    void GenerateGCS();

    void GenerateIRISGraph();

    void AddCosts(const std::vector<std::pair<std::string, NodeInfo>>& element);

    // checks if the terrain (element) is traversable by the vehicle_type
    // and if the feature is too small to be considered (improves computation speed)
    bool check_terrain(const std::pair<std::string, NodeInfo>& element, const Eigen::MatrixXd& vertices, const int& vehicle_type);

    std::pair<ConvexSets, std::vector<std::pair<int, int>>> CreateMainRegionComponents(const std::vector<VPolytope>& vertex_list);

    inline bool DoesPairExist(const std::vector<std::pair<int, int>>& edges, const int i, const int j);

    inline bool ArePolytopesTouching(const ::VPolytope& poly1, const VPolytope& poly2);

    double getTerrainParameter(const std::pair<std::string, NodeInfo>& element);

    // void mergePolygonPair(polygon& poly1, const polygon& poly2);

    VPolytope ShrinkVertices(const Eigen::MatrixXd& vertices);



    double perpendicularDistance(const Eigen::Vector2d& point, const Eigen::Vector2d& lineStart, const Eigen::Vector2d& lineEnd);

    void RDPRecursive(const Eigen::MatrixXd& polygon, int start, int end, double epsilon, std::vector<bool>& keep);

    Eigen::MatrixXd RDP(const Eigen::MatrixXd& polygon, double epsilon);


    void drawPolygonsWithColors(const std::vector<std::pair<int, polygon>>& polygons,
                                               const std::vector<std::pair<point_type, point_type>>& points,
                                               const std::string& filename);

    // std::vector<Eigen::MatrixXd> earClipping(const Eigen::MatrixXd& vertices);

    // bool isPointInTriangle(const Eigen::Vector2d& pt, const Eigen::Vector2d& v1, const Eigen::Vector2d& v2, const Eigen::Vector2d& v3);

    // double sign(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& p3);

    // std::vector<Eigen::MatrixXd> CheckVertexConvexity(Eigen::MatrixXd vertices);

    // bool isConvex(const Eigen::MatrixXd& vertices);

public:
    GCSPlanner(SemMap& SWM,
                      int vehicle_type, 
                      std::vector<double> source_location,
                      std::vector<double> target_location);

    nav_msgs::msg::Path getTrajectory();

    double getPathLength();

    // add feature for re-planning that only replans if the vehicle type is different 
    // (if it is the same, then the GCS map will be the same and can save computation time)
};


// #include <chrono>
// #include <functional>
// #include <memory>
// #include <string>
// #include <filesystem>
// #include <fstream>
// #include <algorithm> 

// #include "rclcpp/rclcpp.hpp"
// #include "UTM.h"
// #include "read_data.h"


// #include "std_msgs/msg/string.hpp"
// #include "geometry_msgs/msg/pose_array.hpp"
// #include "geometry_msgs/msg/polygon.hpp"
// #include "geometry_msgs/msg/polygon_stamped.hpp"
// #include "nav_msgs/msg/path.hpp"
// #include <visualization_msgs/msg/marker_array.hpp>
// #include "sa_msgs/srv/semantic_map_in_a_region_server.hpp"

// #include <drake/planning/trajectory_optimization/gcs_trajectory_optimization.h>
// #include <drake/planning/distance_and_interpolation_provider.h>
// #include <drake/geometry/optimization/graph_of_convex_sets.h>
// #include <drake/geometry/optimization/hpolyhedron.h>
// #include <drake/geometry/optimization/point.h>
// #include <drake/geometry/shape_specification.h>
// #include <drake/geometry/optimization/vpolytope.h>
// #include <drake/common/symbolic/expression/expression.h>
// #include <drake/common/symbolic/expression/variable.h>
// #include <drake/common/copyable_unique_ptr.h>
// #include <drake/common/trajectories/composite_trajectory.h>
// #include <drake/solvers/mathematical_program_result.h>
// #include <drake/solvers/cost.h>


// using namespace drake::planning::trajectory_optimization;
// using namespace drake::geometry::optimization;
// using namespace drake::symbolic;
// using namespace std::chrono_literals;


// enum VehicleType {
//     GroundVehicle, // 0
//     AeralVehicle,  // 1
//     Pedestrian     // 2
// };

// struct SWM_Feature {
//     drake::geometry::optimization::GraphOfConvexSets::Vertex* gcs_vertex;
//     drake::geometry::optimization::VPolytope vertex;
//     drake::planning::trajectory_optimization::GcsTrajectoryOptimization::Subgraph* region;
//     std::pair<std::string, NodeInfo> element;
//     std::string name;
//     int name_int;
//     boost::geometry::model::polygon<point_type> polygon;
// };



// class GCSPlanner
// {
// private:
//     // input values for the class
//     std::vector<double> source_location;
//     std::vector<double> target_location;
//     int vehicle_type;
//     double vehicle_radius;

//     SemMap SM; // general Semantic World Model object
//     GcsTrajectoryOptimization gcs;
//     GcsTrajectoryOptimization::Subgraph* source_region; // starting location converted to graph coordinates
//     GcsTrajectoryOptimization::Subgraph* target_region; // ending location converted to graph coordinates
//     GcsTrajectoryOptimization::Subgraph* main_region;

//     // parameters for the main region in the GCS optimizer
//     int bezier_curve_order;
//     double h_min;
//     double h_max;

//     std::vector<geometry_msgs::msg::PoseStamped> path;
//     double pathLength;

//     // From the SWM, create a gcs
//     // Three regions (source, main, target)
//     // The main region includes each vertex from the SWM
//     // also loads terrain costs and constraints into the graph
//     void GenerateGCS();

//     void AddCosts(const std::vector<std::pair<std::string, NodeInfo>>& element);

//     // checks if the terrain (element) is traversable by the vehicle_type
//     // and if the feature is too small to be considered (improves computation speed)
//     bool check_terrain(const std::pair<std::string, NodeInfo>& element, const Eigen::MatrixXd& vertices, const int& vehicle_type);

//     std::pair<ConvexSets, std::vector<std::pair<int, int>>> CreateMainRegionComponents(const std::vector<Eigen::MatrixXd>& polygon_list);

//     inline bool ArePolytopesTouching(const ::VPolytope& poly1, const VPolytope& poly2);

//     double getTerrainParameter(const std::pair<std::string, NodeInfo>& element);

//     void mergePolygonPair(polygon& poly1, const polygon& poly2);

//     Eigen::MatrixXd ShrinkVertices(const Eigen::MatrixXd& vertices);

// public:
//     GCSPlanner(SemMap& SWM,
//                       int vehicle_type, 
//                       std::vector<double> source_location,
//                       std::vector<double> target_location);

//     std::vector<geometry_msgs::msg::PoseStamped> getTrajectory();

//     double getPathLength();

//     // add feature for re-planning that only replans if the vehicle type is different 
//     // (if it is the same, then the GCS map will be the same and can save computation time)
// };


#endif // GCSPLANNER_H