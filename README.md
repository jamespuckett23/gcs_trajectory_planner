# MCTS-GCS Algorithm

# Installation


# Repo layout
- Use GenerateSWM to facilitate the generation and maintance of the world model (SWM)
- The TrajectoryPlanner class loads and runs the GCS planner through the world model
- MaverickPlanner: comparison planner that is based off a graph-based and sampling-based combination
- A*: plans optimal graph-based path through a costmap that is generated from the world model
- gcs_trajectory_optimization_original: GCS trajectory planner from Drake
- MCTS: virtual implementation of MCTS taken from https://github.com/michaelbzms/MonteCarloTreeSearch/tree/main
    - This will be used to directly interface with the GCS environment
    - JobScheduler can be used to parallize the simulation phase
- GCS-MCTS: overriding class of MCTS to interface with the GCS algorithm
- gcs_trajectory_optimization_mcts: GCS trajectory planner from Drake that has been updated to use the MCTS search algorithm


# TODO
- ROS infrastructure
    - Launch file
    - GenerateSWM node: generates a new swm based off service request
    - runPlanner node: generates path from service request (can use MCTS-GCS planner, GCS planner, A* planner, or Maverick planner)
- Write each file again to work in this ROS infrastructure
- Optimize CMakeLists.txt (dependencies, ROS msgs/launch/etc, requirements)
- Correct package.xml
- Add generate instructions on how to use/install Drake with package
- Files to update:
    - main.cpp
    - GCSPlanner.cpp -> this should be fine
        - Get to work with local Drake copy
        - In AddCosts - indexing by element.index rather than by cycling through two identical vectors. Check they reference the same polygons
    - Double-check Maverick and A* planners
    - GenerateSWM
        - Clean up/optimize
        - Compare to getPathCost functions -> one is vector of geometry_msgs poses and the other is a nav_msgs path. Ideally should have the same total cost
    - MCTS structure
        - mcts GCS_state - tentatively done
        - mcts_graph_of_convex_sets - started
        - mcts_gcs_trajectory_optimization - started
        - MCTS_GCSPlanner - not started