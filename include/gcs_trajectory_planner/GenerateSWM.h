#ifndef GENERATESWM_H
#define GENERATESWM_H

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/polygon/voronoi.hpp>
#include <boost/random.hpp>
#include <opencv2/opencv.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_msgs/msg/costmap.hpp"
// #include "read_data.h"
// #include "Planner.h"
// #include "UTM.h"
// #include "sa_msgs/srv/semantic_map_in_a_region_server.hpp"
#include <vector>
#include <iostream>
#include <random>
#include <optional>

#include <../include/gcs_trajectory_planner/Common.h>

namespace bg = boost::geometry;
namespace bp = boost::polygon;
using Point_Int = bp::point_data<int>;
using VoronoiDiagram = bp::voronoi_diagram<double>;

struct TerrainProperties {
    cv::Scalar color;
    double traversability;
    int terrainClass;
    std::string label;
};

class GenerateSWM
{
private:
    double min_x;
    double max_x;
    double min_y;
    double max_y;
    int num_polygons;

    VoronoiDiagram vd;
    std::vector<polygon> swm_polygons;
    // WorldModel world_model_msg;
    // SemanticWorldModel world_model;
    SemMap swm;
    nav2_msgs::msg::Costmap cost_map;


    // Define the terrain properties map
    std::unordered_map<int, TerrainProperties> terrainPropertiesMap = {
                    {1, {cv::Scalar(0, 0, 255), 0.0, 1, "Obstacle"}},        // Obstacle - Gray
                    {2, {cv::Scalar(0, 100, 0), 50.0, 2, "Tree/Forest"}},    // Tree/Forest/Bush - Dark Green
                    {3, {cv::Scalar(255, 0, 0), 20.0, 3, "Water"}},          // Water - Blue
                    {4, {cv::Scalar(0, 255, 0), 90.0, 4, "Grass"}},          // Grass - Light Green
                    {5, {cv::Scalar(19, 69, 139), 75.0, 5, "Hill"}}          // Hill - Brown
    };

    // void draw_voronoi(cv::Mat& image, const VoronoiDiagram& vd, const cv::Scalar& color);

    std::vector<Point_Int> generate_random_points(double min_x, double max_x, double min_y, double max_y, size_t num_points);

    void convert_to_polygons();

    void create_swm();

public:
    GenerateSWM(const std::vector<std::pair<double, double>> min_max_pair, const int min_number_of_polygons);

    std::vector<polygon> getPolygons();

    SemMap getSWM();

    nav2_msgs::msg::Costmap getCostMap();

    cv::Mat getCostMapImage();

    // void drawVoronoiDiagram(std::string filename);

    cv::Mat getSWMImage(const std::optional<std::vector<geometry_msgs::msg::PoseStamped>>& path = std::nullopt, int path_type = 0, const std::optional<bool>& show_edges = std::nullopt);

    cv::Mat addPathToImage(cv::Mat swm_image, std::vector<geometry_msgs::msg::PoseStamped>& path, int path_type = 0);

    double getPathCost(std::vector<geometry_msgs::msg::PoseStamped>& path);

    double getPathLength(std::vector<geometry_msgs::msg::PoseStamped>& path);

    double getPathCurvature(const std::vector<geometry_msgs::msg::PoseStamped>& path);

    double getPathCost(nav_msgs::msg::Path path);

    void regenerateSWM(const std::optional<std::vector<std::pair<double, double>>> min_max_pair = std::nullopt, 
                       const std::optional<int> min_number_of_polygons = std::nullopt);
};

#endif // GENERATESWM_H