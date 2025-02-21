#include <../include/gcs_trajectory_planner/GCSPlanner.h>

using namespace drake::planning::trajectory_optimization;
using namespace drake::geometry::optimization;
using namespace drake::symbolic;
using namespace std::chrono_literals;


GCSPlanner::GCSPlanner(SemMap& SWM,
                       int vehicle_type, 
                       std::vector<double> source_location_incoming,
                       std::vector<double> target_location_incoming) 
        : gcs(2) // number of decision variables (x,y,z)
{
    // Load class parameters
    SM = SWM;
    vehicle_type = vehicle_type;
    bezier_curve_order = 2;     // set gcs configurations
    h_min = 0;
    h_max = 10.0;
    source_location = source_location_incoming;
    target_location = target_location_incoming;

    std::cout << "about to generate gcs" << std::endl;
    GenerateGCS();
    std::cout << "just generated gcs" << std::endl;

    gcs.AddPathLengthCost(2);
    gcs.AddTimeCost();

    std::cout << "About to solve" << std::endl;
    std::pair<drake::trajectories::CompositeTrajectory<double>, drake::solvers::MathematicalProgramResult> result = gcs.SolvePath(*source_region, *target_region);
    drake::trajectories::CompositeTrajectory<double> gcs_trajectory = result.first;
    drake::solvers::MathematicalProgramResult prog_results = result.second;

    std::cout << "Successful: " << prog_results.is_success() << std::endl;
    // std::cout << "optimal cost: " << prog_results.get_optimal_cost() << std::endl;
    // std::cout << "complexity: " << gcs.EstimateComplexity() << std::endl;

    // Create solution path between start and goal in ROS nav2 msg Path type
    double distance = 0.0;
    geometry_msgs::msg::PoseStamped pose;
    double start_time = gcs_trajectory.start_time();
    double end_time = gcs_trajectory.end_time();
    double step_size = (end_time-start_time)/1000.0; // publish # number of steps between the start and finish times
    path.reserve(1000);

    for (double time = start_time + step_size; time < end_time; time += step_size)
    {
        // Create pose in path 
        pose.pose.position.x = gcs_trajectory.value(time)(0);
        pose.pose.position.y = gcs_trajectory.value(time)(1);
        path.push_back(pose);
        
        Eigen::VectorXd vel1 = gcs_trajectory.EvalDerivative(time, 1);
        Eigen::VectorXd vel2 = gcs_trajectory.EvalDerivative(time+step_size, 1);
        double average_speed = 0.5 * (vel1.norm() + vel2.norm());
        distance += average_speed * step_size;
    }

    std::cout << "total distance to travel path: " << distance << std::endl;
    this->pathLength = distance;
}

void GCSPlanner::GenerateGCS() {
    // coordinate the vertex and elements while creating the graph of convex sets
    std::vector<VPolytope> vertex_list;
    // std::vector<std::pair<std::string, NodeInfo>> element_list;
    std::unordered_map<std::string, Element> element_list;

    std::vector<std::pair<int, polygon>> polygons;
    std::vector<std::pair<point_type, point_type>> lines;

    // used to create the source and target regions
    Eigen::MatrixXd source_vertex_region;
    VPolytope source_to_main_control_point;

    Eigen::MatrixXd target_vertex_region;
    VPolytope main_to_target_control_point;

    // use the boost library to find which vertex the source and target locations are located in
    boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> source_point{this->source_location[0], this->source_location[1]};
    boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> target_point{this->target_location[0], this->target_location[1]};

    // create the source/target vertices and then add each to their respective gcs regions
    Eigen::MatrixXd source_vertex(2, 1);
    source_vertex << source_location[0], source_location[1];
    Eigen::MatrixXd target_vertex(2, 1);
    target_vertex << target_location[0], target_location[1];
    source_region = &this->gcs.AddRegions(MakeConvexSets(Point(source_vertex)), 0);
    target_region = &this->gcs.AddRegions(MakeConvexSets(Point(target_vertex)), 0);

    std::string source_vertex_name;
    int source_vertex_index;
    std::string target_vertex_name;
    int target_vertex_index;

    double number_of_faces = 0.0;
    bool found_source = false;
    bool found_target = false;

    // load each feature into a polygon vertex in the graph of convex sets
    // create a list of required edges to be created next
    double min_distance_to_source_point = std::numeric_limits<double>::infinity();
    double min_distance_to_target_point = std::numeric_limits<double>::infinity();
    std::pair<std::string, NodeInfo> closest_element_to_source;
    std::pair<std::string, NodeInfo> closest_element_to_target;

    auto swm_nodes = SM.fm;

    std::cout << "about to begin the for loop" << std::endl;
    int node_index = 0;
    for (auto node = swm_nodes.begin(); node != swm_nodes.end(); ++node) {
        auto& poly = node->second.Polygon;

        // Determine the number of vertices in the polygon
        std::size_t num_vertices = poly.outer().size();
        Eigen::MatrixXd vertices(2, num_vertices); // Assuming 2D points

        for (std::size_t i = 0; i < num_vertices; ++i) {    // Iterate over the points of the Boost.Geometry polygon and fill the Eigen matrix
            vertices.col(i) << poly.outer()[i].get<0>(), poly.outer()[i].get<1>();
        }

        if (node->first == "Ballpark" || node->first == "Feature_1" || node->first == "Feature_2" || node->first == "Feature_6") {
            continue;
        }

        polygons.push_back(std::make_pair(int(round(node->second.nodeType->currentFeature.traversability.gv)), poly));

        // checks if the node's terrain is traversable at all for the vehicle type
        // also defines the vehicle's radius which is used to shrink the size of the vertex for obstacle avoidance
        if (this->check_terrain(*node, vertices, this->vehicle_type)) {
            const static double epsilon = 1.0;

            Eigen::MatrixXd simplified_vertices = vertices;

            number_of_faces += simplified_vertices.cols();
            auto vertex = VPolytope(simplified_vertices);
            vertex_list.push_back(vertex.GetMinimalRepresentation());

            Element element;
            element.node = node->second;
            element.name = node->first;
            element.index = node_index;
            element.polygon = poly;
            element.vertex = vertex.GetMinimalRepresentation();
            element_list[element.name] = element;

            node_index += 1;

            if (boost::geometry::covered_by(source_point, poly)) {

                // add source_point as a neighbor of poly
                // add source_point as a convex set (only a pt)
                // reference the pt using the string name when necessary around line 290

                source_vertex_name = node->first + "_source";
                SM.neighbors[node->first].push_back(source_vertex_name);

                Eigen::Matrix<double, 2, 1> mat_source;
                mat_source(0,0) = source_point.get<0>();
                mat_source(1,0) = source_point.get<1>();

                Element source_element;
                source_element.node = node->second;
                source_element.name = source_vertex_name;
                source_element.index = node_index;
                source_element.polygon = poly;
                source_element.vertex = VPolytope(Eigen::Ref<const Eigen::MatrixXd>(mat_source));
                element_list[source_element.name] = source_element;
                // source_vertex_name = node->first + "_source";
                // source_vertex_region = simplified_vertices;

                // Eigen::Matrix<double, 2, 1> mat_source;
                // mat_source(0,0) = source_point.get<0>();
                // mat_source(1,0) = source_point.get<1>();

                source_to_main_control_point = VPolytope(Eigen::Ref<const Eigen::MatrixXd>(mat_source));
                vertex_list.push_back(source_to_main_control_point);
                // element_list.push_back(*element);
                node_index += 1;
            }
            if (boost::geometry::covered_by(target_point, poly)) {
                target_vertex_name = node->first + "_target";
                SM.neighbors[node->first].push_back(target_vertex_name);

                Eigen::Matrix<double, 2, 1> mat_target;
                mat_target(0,0) = target_point.get<0>();
                mat_target(1,0) = target_point.get<1>();

                Element target_element;
                target_element.node = node->second;
                target_element.name = target_vertex_name;
                target_element.index = node_index;
                target_element.polygon = poly;
                target_element.vertex = VPolytope(Eigen::Ref<const Eigen::MatrixXd>(mat_target));
                element_list[target_element.name] = target_element;


                // target_vertex_name = node->first;
                // target_vertex_region = simplified_vertices;
                // main_to_target_control_point = vertex.GetMinimalRepresentation();

                // Eigen::Matrix<double, 2, 1> mat_target;
                // mat_target(0,0) = target_point.get<0>();
                // mat_target(1,0) = target_point.get<1>();
                main_to_target_control_point = VPolytope(Eigen::Ref<const Eigen::MatrixXd>(mat_target));
                vertex_list.push_back(main_to_target_control_point);
                // element_list.push_back(*element);
                node_index += 1;
            }
        }
        else {
            // This terrian is an obstacle and will be avoided
            continue;
        }
    }

    std::vector<std::pair<int, int>> edges;
    ConvexSets regions_cs(element_list.size());

    // int num_polytopes = element_list.size();

    // auto start = std::chrono::high_resolution_clock::now();

    // List convex sets in the main region
    std::cout << "about to add the vertex list" << std::endl;
    int size_of_element_vertex = 0;
    // for (auto vertex : vertex_list) {
    //     auto cs_list = MakeConvexSets(vertex); // returns a vector of all convex sets found -> typically only a vector of one
    //     for (auto cs : cs_list) {
    //         regions_cs.push_back(cs);
    //         size_of_element_vertex += 1;
    //     }
    // }
    // std::cout << "size of vertex_list: " << size_of_element_vertex << std::endl;

    size_of_element_vertex = 0;
    std::cout << "about to add the edges" << std::endl;
    for (const auto& element_pair : element_list) {
        auto element = element_pair.second;
        // Add neighbors to the list of edges
        for (auto neighbor : SM.neighbors[element.name]) {
            edges.push_back(std::make_pair(element.index, element_list[neighbor].index));
        }

        // Add convex sets to the main region
        auto convex_set_list = MakeConvexSets(element.vertex); // returns a vector of all convex sets found -> typically only a vector of one
        for (auto convex_set : convex_set_list) {
            regions_cs[element.index] = convex_set;
            // regions_cs.push_back(convex_set);
            size_of_element_vertex += 1;
        }
    }

    std::cout << "size of element_list: " << size_of_element_vertex << std::endl;
    std::cout << "size of regions_cs: " << regions_cs.size() << std::endl;

    // // Find neighbors
    // int index = 0;
    // boost::geometry::index::rtree<std::pair<boost::geometry::model::box<point_type>, std::tuple<std::string, int, polygon>>, boost::geometry::index::quadratic<16>> rtree;
    // for (const auto& element : element_list) {
    //     if (element.first == "Ballpark") {
    //         std::cout << "found ballpark" << std::endl;
    //         index += 1;
    //         continue;
    //     }
    //     rtree.insert(std::make_pair(boost::geometry::return_envelope<boost::geometry::model::box<point_type>>(element.second.Polygon), std::make_tuple(element.first, index, element.second.Polygon)));
    //     index += 1;
    // }

    // for (int i = 0; i < num_polytopes; ++i) {
    //     polygon query_poly = element_list[i].second.Polygon;
    //     std::vector<std::pair<boost::geometry::model::box<point_type>, std::tuple<std::string, int, polygon>>> result;
    //     rtree.query(boost::geometry::index::intersects(boost::geometry::return_envelope<boost::geometry::model::box<point_type>>(query_poly)), std::back_inserter(result));

    //     int comparing_index = 0;
    //     if (result.size() > 100) {
    //         std::cout << "size of result: " << result.size() << std::endl;
    //         std::cout << element_list[i].first << std::endl;
    //     }
    //     for (const auto& value : result) {
    //         if (boost::geometry::intersects(std::get<2>(value.second), query_poly)) {
    //             edges.emplace_back(i, std::get<1>(value.second));
    //             point_type p1;
    //             point_type p2;
    //             boost::geometry::centroid(query_poly, p1);
    //             boost::geometry::centroid(std::get<2>(value.second), p2);
    //             lines.push_back(std::make_pair(p1, p2));
    //             // continue;
    //         }
    //     }
    // }

    std::cout << "about to add the regions" << std::endl;

    // Load main convex sets/edges
    // from the results, create the main region to plan through that includes vertices and edges
    // bezier_curve_order determines how many control points are selected inside each vertex 
    // (# of control points = bezier_curve_order + 1)
    // h_min is the minimum amount of time for the path to travel through a vertex
    // h_max is the maximum amount of time for the path to travel through a vertex
    main_region = &gcs.AddRegions(regions_cs, edges, bezier_curve_order, h_min, h_max, std::string("main"));

    std::cout << "number of vertices: " << regions_cs.size() << std::endl;
    std::cout << "number of edges: " << edges.size() << std::endl;
    std::cout << "average number of faces: " << number_of_faces/(regions_cs.size()-2) << std::endl;
    
    if (source_vertex_name.empty() || target_vertex_name.empty()) {
        std::cout << "ERROR: Did not find source or target node" << std::endl;
    }
    else if (source_vertex_name == target_vertex_name) {
        // The source and target vertex are inside the same region
        // plan a straight line between source and target locations

        // Create edges between regions and consider connecting locations
        std::cout << "Source and target are in the same region" << std::endl;
        auto& source_to_main = gcs.AddEdges(*source_region, *main_region);
        auto& main_to_target = gcs.AddEdges(*main_region, *target_region);
    }
    else {
        // Ensure that the source and target regions connect to the main region at the correct location
        // Convert VPolytope data type to ConvexSet_ptr of HPolyhedron data type
        std::shared_ptr<HPolyhedron> start_HPolyhedron_ptr = std::make_shared<HPolyhedron>(element_list[source_vertex_name].vertex);
        std::shared_ptr<HPolyhedron> target_HPolyhedron_ptr = std::make_shared<HPolyhedron>(element_list[target_vertex_name].vertex);
        const ConvexSet* source_connecting_location_convex_set_ptr = dynamic_cast<const ConvexSet*>(start_HPolyhedron_ptr.get());
        const ConvexSet* target_connecting_location_convex_set_ptr = dynamic_cast<const ConvexSet*>(target_HPolyhedron_ptr.get());
        
        // Create edges between regions and consider connecting locations
        auto& source_to_main = this->gcs.AddEdges(*source_region, *main_region, source_connecting_location_convex_set_ptr); //, std::vector<std::pair<int,int>>{std::make_pair(0,source_vertex_index)});
        auto& main_to_target = this->gcs.AddEdges(*main_region, *target_region, target_connecting_location_convex_set_ptr); //, std::vector<std::pair<int,int>>{std::make_pair(target_vertex_index,0)});
        std::cout << "added region edges" << std::endl;
    } // , std::vector<std::pair<int,int>>{std::make_pair(1,source_vertex_index)}

    AddCosts(element_list);

    // std::string filename = std::string("connectivity_graph");
    // drawPolygonsWithColors(polygons, lines, filename);
}

void GCSPlanner::AddCosts(const std::unordered_map<std::string, Element>& element_list) {
    auto vertices = this->main_region->Vertices();

    // avoid the source and target vertices that were added to the end. They will always be the final two spots in the list
    // std::string source_vertex_name = "Region" + std::to_string(vertices.size()-2);
    // std::string target_vertex_name = "Region" + std::to_string(vertices.size()-1);


    for (auto element_pair : element_list)
    {
        Element element = element_pair.second;
        // if (vertices[i]->name().find(source_vertex_name) != std::string::npos || vertices[i]->name().find(target_vertex_name) != std::string::npos) {
        //     // // this vertex is a start or target point that does not have an accurate mapping to the element because it has been artifically added
        //     continue;
        // }
        int vertex_id = element.index;

        auto vertex = vertices[vertex_id];
        auto x = vertex->x();
        // auto element = element_list[];
        double scale = this->getTerrainParameter(element.node);

        // 5 points (x,y,z) in the vertex
        // 16 values in the list -> the 16th value indicates how many points were selected in the vertex
        // the constraint ensures the last point of the vertex is equal to the first point of the next vertex
        // i.e. the distance from the first point of a vertex to the last point of the vertex is a slight under 
        //  approximation of the distance that the vehicle will travel of the Bezier curve
        const static int last_point_in_u_vertex__x = x.size() - 3;
        const static int last_point_in_u_vertex__y = x.size() - 2;
        const static int first_point_in_v_vertex__x = 0;
        const static int first_point_in_v_vertex__y = 1;

        for (const auto& edge : vertex->outgoing_edges()) 
        {
            edge->AddConstraint(edge->xu()[last_point_in_u_vertex__x] == edge->xv()[first_point_in_v_vertex__x] && \
                                edge->xu()[last_point_in_u_vertex__y] == edge->xv()[first_point_in_v_vertex__y]);

            // Add feature:
            // Ensure that the trajectory does not come within a threshold distance of an obstacle
        }
        
        Expression distance = 0.0;
        for (size_t index = 0; index < x.size()-4; index += 2)
        {
            // the program can only solve convex programs: thus the distance is squared rather than the
            // more typical euclidean distance which is a nonconvex problem
            distance += pow(x(index) - x(index+2), 2) +
                        pow(x(index+1) - x(index+3), 2); 
        }

        // vertex->AddCost(distance/scale);
        vertex->AddCost(distance * scale*scale);
    }
}


bool GCSPlanner::check_terrain(const std::pair<std::string, NodeInfo>& node, const Eigen::MatrixXd& vertices, const int& vehicle_type) {
    std::string feature_name = node.second.nodeType->currentFeature.featureName;
    switch (vehicle_type)
    {
        case VehicleType::GroundVehicle:
            if (node.second.nodeType->currentFeature.traversability.gv == 0.0 || node.second.nodeType->currentFeature.traversability.gv == 255.0) {
                return false;
            }
            vehicle_radius = 2.5; // m
            break;

        case VehicleType::AeralVehicle:
            if (node.second.nodeType->currentFeature.traversability.av == 0.0) {
                return false;
            }
            vehicle_radius = 1.0; // m
            break;

        case VehicleType::Pedestrian:
            if (node.second.nodeType->currentFeature.traversability.pedestrian == 0.0) {
                return false;
            }
            vehicle_radius = 0.4; // m
            break;

        default:
            vehicle_radius = 0.0; // m
            break;
    }

    return true;
}

inline bool GCSPlanner::ArePolytopesTouching(const VPolytope& poly1, const VPolytope& poly2) {
    return poly1.IntersectsWith(poly2);
}

nav_msgs::msg::Path GCSPlanner::getTrajectory() {
    nav_msgs::msg::Path nav_msg_path;
    nav_msg_path.poses = path;
    return nav_msg_path;
}

double GCSPlanner::getPathLength() {
    return pathLength;
}

double GCSPlanner::getTerrainParameter(const NodeInfo& node) {
    // parameter scales
    const static double traversability_param = 1.0; // will be 0.6
    const static double occupiability_param = 0.3;
    const static double visability_param = 0.1;

    // add costs (vehicle dependent)
    double traversability_cost = 0.0; // normalized between 0 and 100
    double occupiability = 0.0; // normalized between 0 and 100
    double visability = 0.0; // normalized between 0 and 100

    switch (vehicle_type)
    {
        case VehicleType::GroundVehicle:
            // if (node.second.nodeType->currentFeature.traversability.gv == 1) {
            //     traversability_cost = 5.0;
            // }
            // else if (node.second.nodeType->currentFeature.traversability.gv == 2) {
            //     traversability_cost = 2.0;
            // }
            // else if (node.second.nodeType->currentFeature.traversability.gv == 3 || node.second.nodeType->currentFeature.traversability.gv == 250) {
            //     traversability_cost = 1.0;
            // }

            // comment out for traditional costmap format. For json artificial cost, use the following line
            traversability_cost = 100 - node.nodeType->currentFeature.traversability.gv; // from 0->100
            // occupiability = node.second.nodeType->currentFeature.occupiability.gv * 100;
            // if (node.second.nodeType->currentFeature.sensorVisibility.fromSide != "yes") 
            //     visability = 10.0;
            break;

        case VehicleType::AeralVehicle:
            traversability_cost = 100 - node.nodeType->currentFeature.traversability.av; // from 0->100
            occupiability = node.nodeType->currentFeature.occupiability.av * 100;
            if (node.nodeType->currentFeature.sensorVisibility.fromAbove != "yes") 
                visability = 10.0;
            break;

        case VehicleType::Pedestrian:
            traversability_cost = 100 - node.nodeType->currentFeature.traversability.pedestrian; // from 0->100
            occupiability = node.nodeType->currentFeature.occupiability.pedestrian * 100;
            if (node.nodeType->currentFeature.sensorVisibility.fromSide != "yes") 
                visability = 10.0;
            break;
            
        default:
            std::cerr << "Unknown vehicle type: " << vehicle_type << std::endl;
    }

    // scale is from a range of 0-100 (traversability - 60%, occupiability - 30%, visability - 10%)
    // double scale = traversability_param*traversability_cost + occupiability_param*occupiability + visability_param*visability;
    double scale = traversability_cost;

    return scale;
}

void GCSPlanner::drawPolygonsWithColors(const std::vector<std::pair<int, polygon>>& polygons,
                                               const std::vector<std::pair<point_type, point_type>>& lines,
                                               const std::string& filename) {
    // Create a blank image (white background)
    cv::Mat image(1500, 1500, CV_8UC3, cv::Scalar(255, 255, 255));
    // Iterate through each polygon and color pair
    for (const auto& colorPolygon : polygons) {
        int colorValue = colorPolygon.first;
        const polygon& polygon = colorPolygon.second;
        // Convert Boost polygon points to OpenCV points
        std::vector<cv::Point> cvPolygon;
        for (const auto& pt : polygon.outer()) {
            cvPolygon.emplace_back(pt.get<0>(), -pt.get<1>());
        }
        // Determine the OpenCV color from the integer value
        cv::Scalar color;
        if (colorValue < 5) color = cv::Scalar(0, 0, 139);   // Dark red
        else if (colorValue < 25) color = cv::Scalar(0, 0, 255); // light Red
        else if (colorValue < 50) color = cv::Scalar(0, 255, 255); // yellow
        else if (colorValue < 75) color = cv::Scalar(0, 165, 255); // orange
        else if (colorValue < 100) color = cv::Scalar(0, 255, 0); // green
        // else if (colorValue == 255) color = cv::Scalar(0, 0, 0); // black
        // else color = cv::Scalar(255, 255, 255); // White for default
        // Fill the polygon with the corresponding color
        cv::fillConvexPoly(image, cvPolygon, color);
    }

    // Draw a rectangle defined by the two Boost points
    // lines should be black
    cv::Scalar color(0, 0, 0);
    int thickness = 1;             // Thickness of the line
    int lineType = cv::LINE_8;     // Type of the line (8-connected)
    for (auto points : lines) {
        cv::Point p1(points.first.get<0>(), -points.first.get<1>());
        cv::Point p2(points.second.get<0>(), -points.second.get<1>());
        cv::line(image, p1, p2, color, thickness, lineType);
    }

    // Add a legend to the image
    int legend_x = 50;   // X position for the legend
    int legend_y = 1300; // Y position for the start of the legend (adjusted for the bottom)
    int box_size = 30;   // Size of each color box in the legend
    int text_offset = 40;

    // Define legend items with their corresponding descriptions
    std::vector<std::pair<cv::Scalar, std::string>> legend_items = {
        {cv::Scalar(0, 0, 139), "Traversability 0 - 5 (Dark Red)"},
        {cv::Scalar(0, 0, 255), "Traversability 6 - 25 (Light Red)"},
        {cv::Scalar(0, 255, 255), "Traversability 26 - 50 (Yellow)"},
        {cv::Scalar(0, 165, 255), "Traversability 51 - 75 (Orange)"},
        {cv::Scalar(0, 255, 0), "Traversability 76 - 100 (Green)"}
    };

    // Draw each legend item
    for (const auto& item : legend_items) {
        // Draw the color box
        cv::rectangle(image, cv::Point(legend_x, legend_y), 
                      cv::Point(legend_x + box_size, legend_y + box_size), 
                      item.first, -1);
        
        // Draw the text next to the box
        cv::putText(image, item.second, 
                    cv::Point(legend_x + box_size + text_offset, legend_y + box_size - 5), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, 
                    cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
        
        // Move down for the next legend item
        legend_y += box_size + 20;
    }
    
    // Save the image
    cv::imwrite("output_image.png", image);
}