#include "../../include/gcs_trajectory_planner/GenerateSWM.h"

GenerateSWM(double width, double height, const int min_number_of_polygons) {
    std::cout << "Generating SWM" << std::endl;

    // load constraints for new SWM
    min_x = 0.0;
    max_x = width;
    min_y = 0.0;
    max_y = height;
    num_polygons = min_number_of_polygons;
    cost_map.metadata.resolution = 0.0;

    // Generate random points for Voronoi diagram
    std::vector<Point_Int> points = generate_random_points(min_x, max_x, min_y, max_y, num_polygons);

    // Build the Voronoi diagram
    bp::construct_voronoi(points.begin(), points.end(), &vd);

    convert_to_polygons();

    create_swm();
}

std::vector<Point_Int> GenerateSWM::generate_random_points(double min_x, double max_x, double min_y, double max_y, size_t num_points) {
    std::vector<Point_Int> points;
    boost::random::mt19937 rng;
    boost::random::uniform_real_distribution<> dist_x(min_x, max_x);
    boost::random::uniform_real_distribution<> dist_y(min_y, max_y);

    for (size_t i = 0; i < num_points; ++i) {
        points.emplace_back(static_cast<int>(dist_x(rng)), static_cast<int>(dist_y(rng)));
    }

    // add all four corners to ensure the corner spaces are covered
    points.emplace_back(static_cast<int>(min_x), static_cast<int>(min_y));
    points.emplace_back(static_cast<int>(min_x), static_cast<int>(max_y));
    points.emplace_back(static_cast<int>(max_x), static_cast<int>(min_y));
    points.emplace_back(static_cast<int>(max_x), static_cast<int>(max_y));

    return points;
}

void GenerateSWM::convert_to_polygons() {
    polygon boundary_box;
    boost::geometry::append(boundary_box, point_type(min_x, min_y));
    boost::geometry::append(boundary_box, point_type(max_x, min_y));
    boost::geometry::append(boundary_box, point_type(max_x, max_y));
    boost::geometry::append(boundary_box, point_type(min_x, max_y));
    boost::geometry::append(boundary_box, point_type(min_x, min_y));

    boost::geometry::correct(boundary_box);

    auto voronoiCellToPolygon = [](const boost::polygon::voronoi_cell<double>& cell, const boost::polygon::voronoi_diagram<double>& vd) {
        polygon poly;
        std::vector<point_type> points;

        // Iterate over each edge of the cell
        for (auto edge = cell.incident_edge();;) {
            if (edge->is_primary()) { // Primary edges correspond to the cell boundaries
                if (edge->vertex0() && edge->vertex1()) {
                    // Convert edge vertices to polygon points
                    double x1 = static_cast<double>(edge->vertex0()->x());
                    double y1 = static_cast<double>(edge->vertex0()->y());
                    double x2 = static_cast<double>(edge->vertex1()->x());
                    double y2 = static_cast<double>(edge->vertex1()->y());
                    
                    points.emplace_back(x1, y1);
                    points.emplace_back(x2, y2);
                }
            }
            edge = edge->next(); // Move to the next edge in the cell
            if (edge == cell.incident_edge()) break; // Stop if we complete a loop
        }

        // Set points into the polygon
        // Remove duplicate points and ensure it's a valid polygon
        boost::geometry::assign_points(poly, points);
        boost::geometry::correct(poly);
        return poly;
    };

    // Iterate over each cell in the Voronoi diagram
    for (auto cell = vd.cells().begin(); cell != vd.cells().end(); ++cell) {
        if (cell->contains_point()) { // Only process cells that contain a point (not boundaries)
            auto voronoi_polygon = voronoiCellToPolygon(*cell, vd);

            // add the parts of the polygon that intersect with the boundary box
            std::vector<polygon> clipped_polygon;
            boost::geometry::intersection(boundary_box, voronoi_polygon, clipped_polygon);
            
            if (!clipped_polygon.empty()) {
                for (auto poly : clipped_polygon) {
                    swm_polygons.push_back(poly);
                }
            }

            // swm_polygons.push_back(voronoi_polygon);
            
        }
    }
}

void GenerateSWM::create_swm() {
    // use swm_polygons to create swm
    std::unordered_map<std::string, NodeInfo> fm;
    for (int i=0; i < swm_polygons.size(); i++) {
        auto polygon = swm_polygons[i];
        
        // Feature name
        std::string name = "Feature_" + std::to_string(i);

        // Feature information
        NodeInfo node_info;
        node_info.Polygon = polygon;

        // select type of terrain from a probability set
        std::random_device rd;  // Seed generator
        std::mt19937 gen(rd()); // Mersenne Twister RNG
        std::uniform_real_distribution<> dis(0.0, 1.0);
        double probability = dis(gen);

        double traversability;
        if (probability < 0.1) {
            // obstacle
            traversability = 0.0;
        }
        else if (probability < 0.3) {
            // tree/forest/bush
            traversability = 50.0;
        }
        else if (probability < 0.4) {
            // water
            traversability = 20.0;
        }
        else if (probability < 0.80) {
            // grass
            traversability = 90.0;
        }
        else {
            // hill
            traversability = 75.0;
        }

        node_info.nodeType = new FeatureUnknown;
        node_info.nodeType->currentFeature.traversability.gv = traversability;

        // Node node;
        // node.name = name;
        // node.polygon = polygon;
        // node.cost = traversability; // may need to be 100-traversability
        // world_model[name] = node
        fm[name] = node_info;

    }
    swm.fm = fm;

    // List the neighbors to each polygon
    std::unordered_map<std::string, std::vector<std::string>> neighbors;
    for (auto element : swm.fm) {
        std::vector<std::string> element_neighbor;
        for (auto second_element : swm.fm) {
            if (element.first != second_element.first && 
                boost::geometry::touches(element.second.Polygon, second_element.second.Polygon)) {
                element_neighbor.push_back(second_element.first);
            }
        }
        neighbors[element.first] = element_neighbor;
    }
    swm.neighbors = neighbors;
}

// void GenerateSWM::drawVoronoiDiagram(std::string filename) {
//     // Create an OpenCV image to draw the Voronoi diagram
//     int width = 60*max_x - 60*min_x, height = 60*max_y - 60*min_x; // Dimensions for the output image
//     cv::Mat image = cv::Mat::zeros(height, width, CV_8UC3);

//     // Draw the Voronoi diagram with a random color
//     cv::Scalar voronoi_color(0, 255, 0); // Green for the Voronoi edges
//     for (auto it = vd.edges().begin(); it != vd.edges().end(); ++it) {
//         if (it->is_primary()) {
//             const auto* v0 = it->vertex0();
//             const auto* v1 = it->vertex1();
//             if (v0 && v1) {
//                 cv::line(
//                     image,
//                     cv::Point(static_cast<int>(v0->x()), static_cast<int>(v0->y())),
//                     cv::Point(static_cast<int>(v1->x()), static_cast<int>(v1->y())),
//                     color, 1);
//             }
//         }
//     }

//     // Save image
//     cv::imwrite(filename, image);
//     std::cout << "Saved Voronoi diagram as " << filename << std::endl;
// }

// void GenerateSWM::draw_voronoi(cv::Mat& image, const VoronoiDiagram& vd, const cv::Scalar& color) {
//     for (auto it = vd.edges().begin(); it != vd.edges().end(); ++it) {
//         if (it->is_primary()) {
//             const auto* v0 = it->vertex0();
//             const auto* v1 = it->vertex1();
//             if (v0 && v1) {
//                 cv::line(
//                     image,
//                     cv::Point(static_cast<int>(v0->x()), static_cast<int>(v0->y())),
//                     cv::Point(static_cast<int>(v1->x()), static_cast<int>(v1->y())),
//                     color, 1);
//             }
//         }
//     }
// }

std::vector<polygon> GenerateSWM::getPolygons() {
    return swm_polygons;
}

SemMap GenerateSWM::getSWM() {
    return swm;
}


cv::Mat GenerateSWM::getSWMImage(const std::optional<std::vector<geometry_msgs::msg::PoseStamped>>& path, int path_type, const std::optional<bool>& show_edges) {
    int width = max_x - min_x, height = max_y - min_y;
    cv::Mat swm_image = cv::Mat::zeros(height, width, CV_8UC3);

    cv::Scalar gcs_color = cv::Scalar(255, 255, 255);       // White for GCS
    cv::Scalar maverick_color = cv::Scalar(255, 255, 0);    // Yellow for Maverick
    cv::Scalar a_star_color = cv::Scalar(255, 0, 255);      // Magenta for A*

    // Draw terrain elements
    for (auto& element : swm.fm) {
        const polygon& poly = element.second.Polygon;
        double traversability = element.second.nodeType->currentFeature.traversability.gv;
        int terrainClass = 0;

        for (const auto& entry : terrainPropertiesMap) {
            if (entry.second.traversability == traversability) {
                terrainClass = entry.first;
                break;
            }
        }

        cv::Scalar color = (terrainClass > 0) ? terrainPropertiesMap[terrainClass].color : cv::Scalar(255, 255, 255);
        std::vector<cv::Point> points;
        for (const auto& vertex : poly.outer()) {
            points.emplace_back(vertex.get<0>(), height - vertex.get<1>());
        }

        std::vector<std::vector<cv::Point>> pts{points};
        cv::fillPoly(swm_image, pts, color);
    }

    // Key/legend drawing
    int key_x_start = width + 20;
    int key_box_size = 20;
    int key_y_start = 20;
    int text_offset = key_box_size + 5;

    // Extended image to include the key
    int legend_width = 200;
    cv::Mat swm_with_key = cv::Mat::zeros(height, width + legend_width, CV_8UC3);
    swm_image.copyTo(swm_with_key(cv::Rect(0, 0, width, height)));

    int y_offset = key_y_start;
    for (const auto& entry : terrainPropertiesMap) {
        const auto& properties = entry.second;
        cv::rectangle(swm_with_key, cv::Rect(key_x_start, y_offset, key_box_size, key_box_size), properties.color, cv::FILLED);
        cv::putText(swm_with_key, properties.label, cv::Point(key_x_start + text_offset, y_offset + key_box_size - 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        y_offset += key_box_size + 10;
    }

    // Add legend for planners
    y_offset += 20;
    cv::putText(swm_with_key, "Planners:", cv::Point(key_x_start, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
    y_offset += 20;
    cv::putText(swm_with_key, "GCS", cv::Point(key_x_start, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.6, gcs_color, 1);
    y_offset += 25;
    cv::putText(swm_with_key, "Maverick", cv::Point(key_x_start, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.6, maverick_color, 1);
    y_offset += 25;
    cv::putText(swm_with_key, "A*", cv::Point(key_x_start, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.6, a_star_color, 1);

    // Draw path if provided
    if (path.has_value()) {
        for (size_t i = 1; i < path->size(); ++i) {
            const auto& start = path->at(i - 1).pose.position;
            const auto& end = path->at(i).pose.position;
            cv::Point pt_start(start.x, height - start.y);
            cv::Point pt_end(end.x, height - end.y);

            cv::Scalar path_color = (path_type == 1) ? maverick_color : (path_type == 2 ? a_star_color : gcs_color);
            cv::line(swm_with_key, pt_start, pt_end, path_color, 2, cv::LINE_AA);
        }
    }

    // Draw edges if requested
    if (show_edges.value_or(false)) {
        for (auto& element_outside : swm.fm) {
            for (auto& element_inside : swm.fm) {
                if (element_outside.first == element_inside.first) continue;

                if (boost::geometry::intersects(element_outside.second.Polygon, element_inside.second.Polygon)) {
                    point_type centroid_outside, centroid_inside;
                    boost::geometry::centroid(element_outside.second.Polygon, centroid_outside);
                    boost::geometry::centroid(element_inside.second.Polygon, centroid_inside);

                    cv::Point cv_centroid_outside(boost::geometry::get<0>(centroid_outside), boost::geometry::get<1>(centroid_outside));
                    cv::Point cv_centroid_inside(boost::geometry::get<0>(centroid_inside), boost::geometry::get<1>(centroid_inside));

                    cv::line(swm_with_key, cv_centroid_outside, cv_centroid_inside, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
                }
            }
        }
    }

    return swm_with_key; // Return the image with the legend
}


cv::Mat GenerateSWM::addPathToImage(cv::Mat swm_image, std::vector<geometry_msgs::msg::PoseStamped>& path, int path_type) {
    std::cout << "Adding path" << std::endl;
    double height = max_y - min_x;
        
    cv::Scalar gcs_color = cv::Scalar(255, 255, 255);  // White for GCS
    cv::Scalar maverick_color = cv::Scalar(255, 255, 0); // Yellow for Maverick
    cv::Scalar a_star_color = cv::Scalar(255, 0, 255); // Magenta for A*

    
    for (const geometry_msgs::msg::PoseStamped& single_path : path) {
        // Loop over each PoseStamped in the Path and draw the path onto the image
        auto& pose_start = single_path.pose.position;
        auto& pose_end = single_path.pose.position;

        // Scale the coordinates to image size
        cv::Point pt_start((pose_start.x), height - (pose_start.y));
        cv::Point pt_end((pose_end.x), height - (pose_end.y));

        // Draw a line segment between the points
        cv::Scalar path_color;
        if (path_type == 0 )
            path_color = gcs_color;
        else if (path_type == 1) {
            // Maverick planner
            path_color = maverick_color;
        }
        else if (path_type == 2) {
            // A* planner
            path_color = a_star_color;
        }
        cv::line(swm_image, pt_start, pt_end, path_color, 2, cv::LINE_AA);
    }

    return swm_image;
}

void GenerateSWM::regenerateSWM(const std::optional<std::vector<std::pair<double, double>>> min_max_pair, 
                                 const std::optional<int> min_number_of_polygons) {
    std::cout << "Generating SWM" << std::endl;

    // update parameters if necessary
    if (min_max_pair.has_value()) {
        min_x = min_max_pair.value()[0].first;
        max_x = min_max_pair.value()[0].second;
        min_y = min_max_pair.value()[1].first;
        max_y = min_max_pair.value()[1].second;
    }
    if (min_number_of_polygons.has_value()) {
        num_polygons = min_number_of_polygons.value();
    }

    // Generate random points for Voronoi diagram
    std::vector<Point_Int> points = generate_random_points(min_x, max_x, min_y, max_y, num_polygons);

    // Build the Voronoi diagram
    boost::polygon::construct_voronoi(points.begin(), points.end(), &vd);

    convert_to_polygons();

    create_swm();
}

double GenerateSWM::getPathCost(std::vector<geometry_msgs::msg::PoseStamped>& path) {
    // the cost is structured by converting the traversability to a traversability cost multiplied by the distance
    // i.e. traversability_cost = 100 - traversability
    // cost = traversability_cost * distance

    // computes path cost through SWM
    double total_cost = 0.0;
    auto element_list = swm.fm;

    auto distance = [](geometry_msgs::msg::PoseStamped& start, geometry_msgs::msg::PoseStamped& end) {
        return std::sqrt(
            std::pow(end.pose.position.x - start.pose.position.x, 2) +
            std::pow(end.pose.position.y - start.pose.position.y, 2)
        );
    };
    

    for (int i = 0; i < path.size() - 1; ++i) {
        // define constants
        auto& start = path[i];
        auto& end = path[i + 1];

        // define constants
        double cost = 0.0;
        double start_traversability = -1.0;
        double end_traversability = -1.0;
        polygon start_poly;
        polygon end_poly;

        for (auto element : element_list) {
            if (boost::geometry::covered_by(point_type(start.pose.position.x, start.pose.position.y), element.second.Polygon)) {
                start_traversability = element.second.nodeType->currentFeature.traversability.gv;
                start_poly = element.second.Polygon;
            } 
            if (boost::geometry::covered_by(point_type(end.pose.position.x, end.pose.position.y), element.second.Polygon)) {
                end_traversability = element.second.nodeType->currentFeature.traversability.gv;
                end_poly = element.second.Polygon;
            }
        }

        if (start_traversability == end_traversability) {
            cost = distance(start, end) * (100.0 - start_traversability);
        }
        else {
            // find intersection point between the start polygon and the line between the start and end points
            std::vector<point_type> intersection_points;
            boost::geometry::model::linestring<point_type> intersection_line;
            intersection_line.push_back(point_type(start.pose.position.x, start.pose.position.y));
            intersection_line.push_back(point_type(end.pose.position.x, end.pose.position.y));
            boost::geometry::intersection(start_poly, intersection_line, intersection_points);

            if (intersection_points.empty()) {
                boost::geometry::intersection(end_poly, intersection_line, intersection_points);
                if (intersection_points.empty()) {
                    continue;
                }
            }

            // convert intersection point_type to geometry_msgs::posestamped
            geometry_msgs::msg::PoseStamped pt;
            pt.pose.position.x = intersection_points[0].get<0>();
            pt.pose.position.y = intersection_points[0].get<1>();

            // compute cost in the start polygon
            cost = distance(start, end) * (100.0 - start_traversability);

            // compute cost in the end polygon
            // cost += distance(pt, end) * (100.0 - end_traversability);
        }
        
        total_cost += cost;
    }
    return total_cost;
}

nav2_msgs::msg::Costmap GenerateSWM::getCostMap() {
    if (cost_map.metadata.resolution != 0.0) {
        return cost_map;
    }

    double resolution = 0.2;

    cost_map.metadata.resolution = resolution;
    cost_map.metadata.size_x = static_cast<uint32_t>((max_x - min_x) / resolution);
    cost_map.metadata.size_y = static_cast<uint32_t>((max_y - min_y) / resolution);
    cost_map.metadata.origin.position.x = min_x;
    cost_map.metadata.origin.position.y = min_y;
    cost_map.data.resize(cost_map.metadata.size_x * cost_map.metadata.size_y, -1); // Unknown space initially

    // Iterate through each polygon and rasterize it into the cost map
    std::cout << "starting for loop" << std::endl;
    for (auto element : swm.fm) {
        std::cout << "next terrain polygon" << std::endl;
        auto& poly = element.second.Polygon;
        double traversability = element.second.nodeType->currentFeature.traversability.gv;

        // Rasterize each polygon by getting the bounding box and iterating over it
        boost::geometry::model::box<point_type> bbox;
        boost::geometry::envelope(poly, bbox);
        
        int min_col = std::max(0, static_cast<int>((bbox.min_corner().get<0>() - min_x) / resolution));
        int max_col = std::min(static_cast<int>(cost_map.metadata.size_x), static_cast<int>((bbox.max_corner().get<0>() - min_x) / resolution));
        int min_row = std::max(0, static_cast<int>((bbox.min_corner().get<1>() - min_y) / resolution));
        int max_row = std::min(static_cast<int>(cost_map.metadata.size_y), static_cast<int>((bbox.max_corner().get<1>() - min_y) / resolution));

        for (int row = min_row; row < max_row; ++row) {
            for (int col = min_col; col < max_col; ++col) {
                // Calculate the center of each cell
                double cell_x = min_x + col * resolution + resolution / 2.0;
                double cell_y = min_y + row * resolution + resolution / 2.0;

                // Check if the cell center is within the polygon
                if (boost::geometry::within(point_type(cell_x, cell_y), poly)) {
                    int index = row * cost_map.metadata.size_x + col;
                    cost_map.data[index] = static_cast<int>(traversability);
                }
            }
        }
    }

    return cost_map;
}

double GenerateSWM::getPathLength(std::vector<geometry_msgs::msg::PoseStamped>& path) {
    auto distance = [](geometry_msgs::msg::PoseStamped& start, geometry_msgs::msg::PoseStamped& end) {
        return std::sqrt(
            std::pow(end.pose.position.x - start.pose.position.x, 2) +
            std::pow(end.pose.position.y - start.pose.position.y, 2)
        );
    };
    double length = 0.0;

    for (int i=0; i < path.size(); i++) {
        length += distance(path[i], path[i+1]);
    }

    return length;
}

cv::Mat GenerateSWM::getCostMapImage() {
    // Define terrain colors
    std::unordered_map<int, cv::Scalar> terrainColors = {
        {1, cv::Scalar(0, 0, 0)},       // Obstacle - black
        {2, cv::Scalar(34, 139, 34)},   // Forest/Tree - dark green
        {3, cv::Scalar(255, 0, 0)},     // Water - blue
        {4, cv::Scalar(0, 255, 0)},     // Grass - green
        {5, cv::Scalar(0, 165, 255)}    // Hill - orange
    };

    // Initialize the OpenCV image matrix with the costmap size
    int width = cost_map.metadata.size_x;
    int height = cost_map.metadata.size_y;
    cv::Mat cost_map_image(height, width, CV_8UC3, cv::Scalar(255, 255, 255));  // default white background

    // Iterate over each cell in the costmap
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int cost = cost_map.data[y * width + x];  // Access costmap data
            int terrainClass;

            // Determine the terrain class based on traversability values
            switch (cost) {
                case 0:
                    terrainClass = 1;  // obstacle
                    break;
                case 60:
                    terrainClass = 2;  // tree/forest/bush
                    break;
                case 30:
                    terrainClass = 3;  // water
                    break;
                case 90:
                    terrainClass = 4;  // grass
                    break;
                case 75:
                    terrainClass = 5;  // hill
                    break;
                default:
                    terrainClass = 0;  // unclassified or unknown
            }

            // Set the color based on terrain type, defaulting to white if unknown
            cv::Scalar color = terrainColors.count(terrainClass) ? terrainColors[terrainClass] : cv::Scalar(255, 255, 255);
            
            // Draw the cell as a filled rectangle
            cv::rectangle(cost_map_image, cv::Point(x, y), cv::Point(x + 1, y + 1), color, cv::FILLED);
        }
    }

    return cost_map_image;
}

double GenerateSWM::getPathCurvature(const std::vector<geometry_msgs::msg::PoseStamped>& path) {
    auto calculateCurvature = [](const geometry_msgs::msg::PoseStamped& p1,
                                 const geometry_msgs::msg::PoseStamped& p2,
                                 const geometry_msgs::msg::PoseStamped& p3) -> double {
        double x1 = p1.pose.position.x, y1 = p1.pose.position.y;
        double x2 = p2.pose.position.x, y2 = p2.pose.position.y;
        double x3 = p3.pose.position.x, y3 = p3.pose.position.y;

        // Calculate the area of the triangle formed by (p1, p2, p3)
        double area = std::abs((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1)) / 2.0;

        // Lengths of the triangle sides
        double a = std::hypot(x2 - x1, y2 - y1);
        double b = std::hypot(x3 - x2, y3 - y2);
        double c = std::hypot(x3 - x1, y3 - y1);

        // Circumradius formula: R = (a * b * c) / (4 * area)
        double curvature = 0.0;
        if (area > 1e-6) {
            double R = (a * b * c) / (4.0 * area);
            curvature = 1.0 / R;
        }
        return curvature;
    };

    if (path.size() < 3) {
        std::cerr << "Path must contain at least 3 points to compute curvature." << std::endl;
        return 0.0;
    }

    double curvature_sum = 0.0;
    int curvature_count = 0;

    // Compute curvature for each triplet of points
    for (size_t i = 1; i < path.size() - 1; ++i) {
        double curvature = calculateCurvature(path[i - 1], path[i], path[i + 1]);
        curvature_sum += curvature * curvature;
        curvature_count++;
    }

    // Calculate RMS curvature
    double rms_curvature = std::sqrt(curvature_sum / curvature_count);
    return rms_curvature;
}

double GenerateSWM::getPathCost(nav_msgs:msg::Path path) {
    
    auto get_row_major_index = [&costmap](int x_index, int y_index) -> int
    {
        int row_major_index = y_index*costmap.metadata.size_x + x_index;
        return row_major_index;
    };

    auto convert_to_costmap = [&costmap](geometry_msgs::msg::PoseStamped location) -> std::pair<int,int>
    {
        int x = round((location.pose.position.x - costmap.metadata.origin.position.x) / costmap.metadata.resolution);
        int y = round((location.pose.position.y - costmap.metadata.origin.position.y) / costmap.metadata.resolution);
        return std::make_pair(x, y);
    };

    auto get_next_index = [&costmap, &convert_to_costmap](geometry_msgs::msg::PoseStamped next_location, int current_x, int current_y) -> std::pair<int,int>
    {
        int next_x, next_y;

        auto resulting_index = convert_to_costmap(next_location);
        int location_x = resulting_index.first;
        int location_y = resulting_index.second;

        // find the correct direction to move
        int gradient_x = location_x - current_x;
        int gradient_y = location_y - current_y;

        // step in the correct direction
        if (gradient_x != 0) {
            next_x = current_x + (gradient_x > 0 ? 1 : -1); // move right or left
        } else {
            next_x = current_x; // no horizontal movement needed
        }

        if (gradient_y != 0) {
            next_y = current_y + (gradient_y > 0 ? 1 : -1); // move up or down
        } else {
            next_y = current_y; // no vertical movement needed
        }

        return std::make_pair(next_x, next_y);
    };


    std::vector<std::pair<int,int>> cost_map_path;
    bool at_goal_node = false;
    double path_cost = 0.0;
    int collisions = 0;
    // const static vector<pair<int, int>> directions = {
    //     {-1, -1}, {-1, 0}, {-1, 1},
    //     { 0, -1},          { 0, 1},
    //     { 1, -1}, { 1, 0}, { 1, 1}
    // };  

    geometry_msgs::msg::PoseStamped start_pose = path[0];
    int start_x_index = round((start_pose.pose.position.x - costmap.metadata.origin.position.x) / costmap.metadata.resolution);
    int start_y_index = round((start_pose.pose.position.y - costmap.metadata.origin.position.y) / costmap.metadata.resolution);

    geometry_msgs::msg::PoseStamped goal_pose = path.back();
    int goal_x_index = round((goal_pose.pose.position.x - costmap.metadata.origin.position.x) / costmap.metadata.resolution);
    int goal_y_index = round((goal_pose.pose.position.y - costmap.metadata.origin.position.y) / costmap.metadata.resolution);

    // path.erase(path.begin());
    cost_map_path.push_back(std::make_pair(start_x_index, start_y_index));
    // int previous_x_index = start_x_index;
    // int previous_y_index = start_y_index;

    int current_x_index = start_x_index;
    int current_y_index = start_y_index;
    path.poses.erase(path.poses.begin());

    int next_x;
    int next_y;

    // create list of nodes that travel along the selected gcs path
    while (!at_goal_node) {
        // are we at the final location?
        if (current_x_index == goal_x_index && current_y_index == goal_y_index) {
            at_goal_node = true;
        }

        // find the next step
        auto next_location = .poses[0];
        auto result_next_index = get_next_index(next_location, current_x_index, current_y_index);
        next_x = result_next_index.first;
        next_y = result_next_index.second;

        // take the step
        cost_map_path.push_back(std::make_pair(next_x, next_y));

        // update
        auto next_location_in_costmap = convert_to_costmap(next_location);
        current_x_index = next_x;
        current_y_index = next_y;
        if (current_x_index == goal_x_index && current_y_index == goal_y_index) {
                at_goal_node = true;
        }
        if (next_location_in_costmap.first == next_x && next_location_in_costmap.second == next_y) {
            path.poses.erase(path.poses.begin());
        }

    }

    // compute cost of the path
    int previous_index = get_row_major_index(cost_map_path[0].first, cost_map_path[0].second);
    for (int i = 1; i < cost_map_path.size(); i++) {
        int index = get_row_major_index(cost_map_path[i].first, cost_map_path[i].second);

        // find step size (either 1 or sqrt(2))
        double step_size = sqrt(pow(cost_map_path[i].first - cost_map_path[i-1].first,2) + pow(cost_map_path[i].second - cost_map_path[i-1].second,2));

        // path_cost += step_size/allowed_speed;
        double traversability = costmap.data[index];
        double traversability_cost = 100 - traversability;

        path_cost += step_size * traversability_cost;

        // update
        previous_index = index;
    }

    // record path


    std::cout << "The SWM Optimization Path cost is: " << path_cost << " with " << collisions << " collisions" << std::endl;

    return path_cost;
}