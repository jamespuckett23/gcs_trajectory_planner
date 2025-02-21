#pragma once

#include <random>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/polygon/voronoi.hpp>
#include <boost/random.hpp>
#include <opencv2/opencv.hpp>
// #include "../../msg/WorldModel.msg"
// #include "../../msg/Node.msg"

#include <nav_msgs/msg/path.h>

#include <drake/planning/trajectory_optimization/gcs_trajectory_optimization.h>
#include <drake/geometry/optimization/vpolytope.h>

// This is a trimed version of the Semantic Map developed by Anant. This is closer to a struct to hold mapping information rather than the indepth class that Anant wrote.

typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> point_type;
typedef boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> point_type3D;
typedef boost::geometry::model::polygon<point_type > polygon;

struct two_int_vect{
    std::vector<int> a; // Map OG
    std::vector<int> b; // Img PGM
    std::array<float,2> ori;
};

struct waypnt {
	std::vector<point_type> xy;
	int width;
};

struct state_s {
	int gv, pedestrian, av;
};

struct shape_s {
	point_type origin;
	std::vector<point_type> boundary;
	polygon Polygon;
	waypnt waypoint;
	double r;
	std::string type;
	point_type centroid;
};

struct sensorVisibility_s {
	std::string fromSide, fromAbove;
};

struct Feature {

	std::string featureName;
	std::string type; // name of the feature type from traversability mapping 
	int zoomLevel = -1;
	std::string parent;
	state_s traversability; // right now just use greyscale values for all 3 states
	state_s occupiability;  // use 0 for everything right now, not currently being used
	sensorVisibility_s sensorVisibility; // use "no" right now, not currently being used
	shape_s shapeParams; // polygon information for the feature
	int index;
	cv::Vec4i hierarchyVals;
};

class FeatureBase {
public:
	Feature currentFeature;
	
	FeatureBase(); 
	// void ResetFeature();
	// void FeatureShape(point_type ori, std::string type, std::vector<double> details);
	// void shiftOrigin(double x, double y);
};

class FeatureUnknown : public FeatureBase {


};

struct NodeInfo {

    std::vector<std::string> childNodes;
    FeatureBase* nodeType = nullptr;
    int zoomLevel = -1;
    std::vector<point_type> boundary;
    polygon Polygon;
    std::vector<int> consistsOf;
};

struct region_details {
	std::vector<Feature> inputs; // polygon vertices
  	std::map<int, std::string> idx2name; 
  	std::map<std::string, int> name2idx; 
	point_type origin;
    std::vector<point_type> boundary_ballpark;
	cv::Mat occupancy_grid;
	float resolution;
	std::string layer;
};

struct Element {
    // drake::geometry::optimization::GraphOfConvexSets::Vertex* gcs_vertex;
    drake::geometry::optimization::VPolytope vertex;
    // drake::planning::trajectory_optimization::GcsTrajectoryOptimization::Subgraph* region;
    NodeInfo node;
    // std::pair<std::string, NodeInfo> element;
    std::string name;
    int index;
    boost::geometry::model::polygon<point_type> polygon;
};

class SemMap
{
private:
    /* data */
public:
    std::unordered_map<std::string, NodeInfo> fm;
	std::unordered_map<std::string, std::vector<std::string>> neighbors;

    SemMap();
    ~SemMap();
};