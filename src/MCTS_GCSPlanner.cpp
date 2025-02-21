// #include "../include/gcs_trajectory_planner/MCTS_GCSPlanner.h"


// MCTS_GCS_Planner::MCTS_GCS_Planner() {

// }

// void MCTS_GCS_Planner::setStart_Goal(std::vector<double> start, std::vector<double> goal) {
//     start_location = start;
//     goal_location = goal;
// }

// int MCTS_GCS_Planner::setSWM(SemMap swm) {
//     swm = SWM;

//     GenerateGCS();

//     return 1;
// }

// nav_msgs::msg::Path MCTS_GCS_Planner::getPath() {
//     return path;
// }

// void MCTS_GCS_Planner::runPlanner() {

// }

// void MCTS_GCS_Planner::GenerateGCS() {
//     // coordinate the vertex and elements while creating the graph of convex sets
//     std::vector<VPolytope> vertex_list;
//     // std::vector<std::pair<std::string, NodeInfo>> element_list;
//     std::unordered_map<std::string, Element> element_list;

//     std::vector<std::pair<int, polygon>> polygons;
//     std::vector<std::pair<point_type, point_type>> lines;

//     // used to create the source and target regions
//     Eigen::MatrixXd source_vertex_region;
//     VPolytope source_to_main_control_point;

//     Eigen::MatrixXd target_vertex_region;
//     VPolytope main_to_target_control_point;

//     // use the boost library to find which vertex the source and target locations are located in
//     boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> source_point{start_location[0], start_location[1]};
//     boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> target_point{goal_location[0], goal_location[1]};

//     // create the source/target vertices and then add each to their respective gcs regions
//     Eigen::MatrixXd source_vertex(2, 1);
//     source_vertex << source_location[0], source_location[1];
//     Eigen::MatrixXd target_vertex(2, 1);
//     target_vertex << target_location[0], target_location[1];
//     source_region = &this->gcs.AddRegions(MakeConvexSets(Point(source_vertex)), 0);
//     target_region = &this->gcs.AddRegions(MakeConvexSets(Point(target_vertex)), 0);

//     std::string source_vertex_name;
//     int source_vertex_index;
//     std::string target_vertex_name;
//     int target_vertex_index;

//     double number_of_faces = 0.0;
//     bool found_source = false;
//     bool found_target = false;

//     // load each feature into a polygon vertex in the graph of convex sets
//     // create a list of required edges to be created next
//     double min_distance_to_source_point = std::numeric_limits<double>::infinity();
//     double min_distance_to_target_point = std::numeric_limits<double>::infinity();
//     std::pair<std::string, NodeInfo> closest_element_to_source;
//     std::pair<std::string, NodeInfo> closest_element_to_target;

//     auto swm_nodes = swm.fm;

//     // std::cout << "start: " << source_location[0] << ", "<< source_location[1] << std::endl;
//     // std::cout << "goal: " << target_location[0] << ", "<< target_location[1] << std::endl;
//     int node_index = 0;
//     for (auto node = swm_nodes.begin(); node != swm_nodes.end(); ++node) {
//         auto& poly = node->second.Polygon;

//         // Determine the number of vertices in the polygon
//         std::size_t num_vertices = poly.outer().size();
//         Eigen::MatrixXd vertices(2, num_vertices); // Assuming 2D points

//         for (std::size_t i = 0; i < num_vertices; ++i) {    // Iterate over the points of the Boost.Geometry polygon and fill the Eigen matrix
//             vertices.col(i) << poly.outer()[i].get<0>(), poly.outer()[i].get<1>();
//         }

//         if (node->first == "Ballpark" || node->first == "Feature_1" || node->first == "Feature_2" || node->first == "Feature_6") {
//             continue;
//         }

//         polygons.push_back(std::make_pair(int(round(node->second.nodeType->currentFeature.traversability.gv)), poly));

//         // checks if the node's terrain is traversable at all for the vehicle type
//         // also defines the vehicle's radius which is used to shrink the size of the vertex for obstacle avoidance
//         if (this->check_terrain(*node, vertices, this->vehicle_type)) {
//             const static double epsilon = 1.0;

//             Eigen::MatrixXd simplified_vertices = vertices;

//             number_of_faces += simplified_vertices.cols();
//             auto vertex = VPolytope(simplified_vertices);
//             vertex_list.push_back(vertex.GetMinimalRepresentation());

//             Element element;
//             element.node = node->second;
//             element.name = node->first;
//             element.index = node_index;
//             element.polygon = poly;
//             element.vertex = vertex.GetMinimalRepresentation();
//             element_list[element.name] = element;

//             if (boost::geometry::covered_by(source_point, poly)) {
//                 source_vertex_name = node->first;
//                 source_vertex_region = simplified_vertices;

//                 Eigen::Matrix<double, 2, 1> mat_source;
//                 mat_source(0,0) = source_point.get<0>();
//                 mat_source(1,0) = source_point.get<1>();

//                 source_to_main_control_point = VPolytope(Eigen::Ref<const Eigen::MatrixXd>(mat_source));
//                 vertex_list.push_back(source_to_main_control_point);
//                 // element_list.push_back(*element);
//             }
//             if (boost::geometry::covered_by(target_point, poly)) {
//                 target_vertex_name = node->first;
//                 target_vertex_region = simplified_vertices;
//                 main_to_target_control_point = vertex.GetMinimalRepresentation();

//                 Eigen::Matrix<double, 2, 1> mat_target;
//                 mat_target(0,0) = target_point.get<0>();
//                 mat_target(1,0) = target_point.get<1>();
//                 main_to_target_control_point = VPolytope(Eigen::Ref<const Eigen::MatrixXd>(mat_target));
//                 vertex_list.push_back(main_to_target_control_point);
//                 // element_list.push_back(*element);
//             }

//             node_index += 1;
//         }
//         else {
//             // This terrian is an obstacle and will be avoided
//             continue;
//         }
//     }

//     std::vector<std::pair<int, int>> edges;
//     ConvexSets regions_cs;

//     // List convex sets in the main region
//     for (auto vertex : vertex_list) {
//         auto cs_list = MakeConvexSets(vertex); // returns a vector of all convex sets found -> typically only a vector of one
//         for (auto cs : cs_list) {
//             regions_cs.push_back(cs);
//         }
//     }

//     for (const auto& element_pair : element_list) {
//         auto element = element_pair.second;
//         // Add neighbors to the list of edges
//         for (auto neighbor : SM.neighbors[element.name]) {
//             edges.push_back(std::make_pair(element.index, element_list[neighbor].index));
//         }

//         // Add convex sets to the main region
//         auto convex_set_list = MakeConvexSets(element.vertex); // returns a vector of all convex sets found -> typically only a vector of one
//         for (auto convex_set : convex_set_list) {
//             regions_cs.push_back(convex_set);
//         }
//     }

//     // // Find neighbors
//     // int index = 0;
//     // boost::geometry::index::rtree<std::pair<boost::geometry::model::box<point_type>, std::tuple<std::string, int, polygon>>, boost::geometry::index::quadratic<16>> rtree;
//     // for (const auto& element : element_list) {
//     //     if (element.first == "Ballpark") {
//     //         std::cout << "found ballpark" << std::endl;
//     //         index += 1;
//     //         continue;
//     //     }
//     //     rtree.insert(std::make_pair(boost::geometry::return_envelope<boost::geometry::model::box<point_type>>(element.second.Polygon), std::make_tuple(element.first, index, element.second.Polygon)));
//     //     index += 1;
//     // }

//     // for (int i = 0; i < num_polytopes; ++i) {
//     //     polygon query_poly = element_list[i].second.Polygon;
//     //     std::vector<std::pair<boost::geometry::model::box<point_type>, std::tuple<std::string, int, polygon>>> result;
//     //     rtree.query(boost::geometry::index::intersects(boost::geometry::return_envelope<boost::geometry::model::box<point_type>>(query_poly)), std::back_inserter(result));

//     //     int comparing_index = 0;
//     //     if (result.size() > 100) {
//     //         std::cout << "size of result: " << result.size() << std::endl;
//     //         std::cout << element_list[i].first << std::endl;
//     //     }
//     //     for (const auto& value : result) {
//     //         if (boost::geometry::intersects(std::get<2>(value.second), query_poly)) {
//     //             edges.emplace_back(i, std::get<1>(value.second));
//     //             point_type p1;
//     //             point_type p2;
//     //             boost::geometry::centroid(query_poly, p1);
//     //             boost::geometry::centroid(std::get<2>(value.second), p2);
//     //             lines.push_back(std::make_pair(p1, p2));
//     //             // continue;
//     //         }
//     //     }
//     // }

//     // Load main convex sets/edges
//     // from the results, create the main region to plan through that includes vertices and edges
//     // bezier_curve_order determines how many control points are selected inside each vertex 
//     // (# of control points = bezier_curve_order + 1)
//     // h_min is the minimum amount of time for the path to travel through a vertex
//     // h_max is the maximum amount of time for the path to travel through a vertex
//     main_region = &gcs.AddRegions(regions_cs, edges, bezier_curve_order, h_min, h_max, std::string("main"));

//     std::cout << "number of vertices: " << regions_cs.size() << std::endl;
//     std::cout << "number of edges: " << edges.size() << std::endl;
//     std::cout << "average number of faces: " << number_of_faces/(regions_cs.size()-2) << std::endl;
    
//     if (source_vertex_name.empty() || target_vertex_name.empty()) {
//         std::cout << "ERROR: Did not find source or target node" << std::endl;
//     }
//     else if (source_vertex_name == target_vertex_name) {
//         // The source and target vertex are inside the same region
//         // plan a straight line between source and target locations

//         // Create edges between regions and consider connecting locations
//         std::cout << "Source and target are in the same region" << std::endl;
//         auto& source_to_main = gcs.AddEdges(*source_region, *main_region);
//         auto& main_to_target = gcs.AddEdges(*main_region, *target_region);
//     }
//     else {
//         // Ensure that the source and target regions connect to the main region at the correct location
//         // Convert VPolytope data type to ConvexSet_ptr of HPolyhedron data type
//         std::shared_ptr<HPolyhedron> start_HPolyhedron_ptr = std::make_shared<HPolyhedron>(source_to_main_control_point);
//         std::shared_ptr<HPolyhedron> target_HPolyhedron_ptr = std::make_shared<HPolyhedron>(main_to_target_control_point);
//         const ConvexSet* source_connecting_location_convex_set_ptr = dynamic_cast<const ConvexSet*>(start_HPolyhedron_ptr.get());
//         const ConvexSet* target_connecting_location_convex_set_ptr = dynamic_cast<const ConvexSet*>(target_HPolyhedron_ptr.get());
        
//         // Create edges between regions and consider connecting locations
//         auto& source_to_main = this->gcs.AddEdges(*source_region, *main_region, source_connecting_location_convex_set_ptr); //, std::vector<std::pair<int,int>>{std::make_pair(0,source_vertex_index)});
//         auto& main_to_target = this->gcs.AddEdges(*main_region, *target_region, target_connecting_location_convex_set_ptr); //, std::vector<std::pair<int,int>>{std::make_pair(target_vertex_index,0)});
//         std::cout << "added region edges" << std::endl;
//     } // , std::vector<std::pair<int,int>>{std::make_pair(1,source_vertex_index)}

//     AddCosts(element_list);
// }