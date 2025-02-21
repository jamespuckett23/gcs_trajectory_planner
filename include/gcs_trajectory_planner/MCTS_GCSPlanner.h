// #ifndef MCTS_GCS_PLANNER_H
// #define MCTS_GCS_PLANNER_H

// #include "state.h"
// #include "Common.h"
// #include "MCTS_GCS.h"
// #include "mcts_gcs_trajectory_optimization.h"
// #include "mcts_graph_of_convex_sets.h"

// #include "nav_msgs/msg/path.hpp"
// #include "sa_msgs/srv/semantic_map_in_a_region_server.hpp"


// class MCTS_GCS_Planner {
//     public:
//         MCTS_GCS_Planner();

//         void setStart_Goal(std::vector<double> start, std::vector<double> goal);

//         int setSWM(SemMap SWM);

//         void runPlanner();

//         nav_msgs::msg::Path getPath();

//     private:
//         std::vector<double> start_location;
//         std::vector<double> goal_location;

//         SemMap swm;
//         drake::planning::trajectory_optimization::GcsTrajectoryOptimization gcs;

//         nav_msgs::msg::Path path;

//         void Generate_GCS();

//         nav_msgs::msg::Path run_MCTS_GCS();
// }

// #endif // MCTS_GCS_PLANNER_H