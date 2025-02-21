#include "../../include/gcs_trajectory_planner/MCTS_GCS.h"

GCS_state::GCS_state() {
    // initalize the tree
    // run algorithm

    // swm
    // gcs

    current_node = "";
    current_pose.orientation.w = 1.0;

    goal_node = "";
    goal_pose.orientation.w = 1.0;

    path_cost = 0.0;
}

GCS_state::GCS_state(const GCS_state &other) {
    // map formats
    swm = other.swm;
    gcs = other.gcs;

    // agent information
    current_node = other.current_node;
    current_pose = other.current_pose;

    goal_node = other.goal_node;
    goal_pose = other.goal_pose;

    path_cost = other.path_cost;
}

bool GCS_state::is_terminal() const {
    return current_node == goal_node && current_pose.position.x == goal_pose.position.x && current_pose.position.y == goal_pose.position.y;
}

MCTS_state *GCS_state::next_state(const MCTS_move *move) const {
    GCS_state *new_state = new GCS_state(*this);

    new_state->path_cost = new_state->take_action((const GCS_move *) move);

    return new_state;
}

double GCS_state::take_action(const GCS_move *move) {
    // remember previous state
    path_to_current_node.push_back(move->action.second);

    // update position
    // TODO: this next pose will be the centroid until a continous step is added
    current_pose = move->next_pose;

    // update node
    // TODO: the second node might still be the same as the first node depending on the continuous action space
    current_node = move->action.second;

    // update path cost
    // TODO: as the step gets more complicated, this will need to be re-written
    return path_cost + compute_step_cost(move->action.first, move->action.second);
}

queue<MCTS_move *> *GCS_state::actions_to_try() const {
    // const_cast<GCS_state *>(this)->
    return generate_all_moves();
}

queue<MCTS_move *> *GCS_state::generate_all_moves() const {
    // queue<MCTS_move *> *Q = new queue<MCTS_move *>();
    std::queue<MCTS_move *>* Q;

    // add all legal moves (discrete action space)
    for (auto neighbor : swm->neighbors[current_node]) {
        Q->push(new GCS_move(make_pair(current_node, neighbor)));
    }

    // TODO:
        // - the above for loop should be replaced with a function call that determines if we need to stay inside
        // the same node (continous actions) or if we can branch out to a new node (discrete actions)

    return Q;
}

double GCS_state::rollout() const {
    #define MAX_STEPS 50
    GCS_state s(*this); // copy current state to bypass const

    bool noerror;
    double path_cost = s.get_path_cost();

    uniform_real_distribution<double> dist(0.0, 1.0);
    std::random_device rd;
    std::default_random_engine generator(rd());

    // used in the rollout to not re-visit the same nodes
    std::vector<std::string> visited_nodes;

    visited_nodes = path_to_current_node; // don't make a loop
    visited_nodes.push_back(s.current_node); // don't re-visit the current node

    for (int i=0; i<MAX_STEPS; i++) {
        if (s.is_terminal()) {
            return s.get_path_cost();
        }
        
        // currently take a random action
        // TODO:
            // - pick an action according to our current/best exploration/expoitation policy
        GCS_move *m = pick_random_move(s, dist, generator, visited_nodes);

        // TODO: you need to figure out how to use take_action and next_state. 
        // does the rollout need to update and remember the states?
        // or is it a simple action step? Either way, the path cost needs to be rememebered
        path_cost = s.take_action(m);

        // maybe add error handling if the action space becomes more complicated especially with
        // the random probability actions
    }

    return s.get_path_cost();
}

GCS_move* GCS_state::pick_random_move(GCS_state &s, uniform_real_distribution<double> &dist, default_random_engine &gen, std::vector<std::string> &visited_nodes) const {

    std::vector<std::string> current_neighbors = swm->neighbors[s.current_node];

    if (current_neighbors.empty()) {
        throw std::runtime_error("In MCTS rollout, node does not have any neighbors");
    }

    double edge_probability = 1.0/current_neighbors.size(); // 0.2

    bool found_edge = false;
    int selected_edge = 0;
    while (!found_edge) {
        double probability = dist(gen); // 0.58

        selected_edge = int(std::floor(probability / edge_probability));

        // check if the node has been visited before
        if (std::find(visited_nodes.begin(), visited_nodes.end(), current_neighbors[selected_edge]) != visited_nodes.end()) {
            found_edge = true;
        }
    }

    visited_nodes.push_back(current_neighbors[selected_edge]);

    // TODO: pick a more intellegent selection
    // vector of connecting edges from the current node
    // vector of previously visited nodes


    // pick a value between 0 and 1 -> selected action

    // normalize the flow probabilities across all possible edge actions

    // take the action that corresponds to the selected action


    // taken from the MCIP GCS implementation on drake's github
    // searches through the outgoing edges of the current vertex and picks based off the randomly selected path
    // double maximum_flow = 0;
    // const Vertex* max_flow_vertex{nullptr};
    // const Edge* max_flow_edge = nullptr;
    // for (const Edge* e : path_vertices.back()->outgoing_edges()) {
    //     const double flow = result.GetSolution(e->phi());
    //     // If the edge has not been visited and has a flow greater than the
    //     // current maximum, then this is our new maximum.
    //     if (flow >= 1 - tolerance && flow > maximum_flow &&
    //         !visited_vertices.contains(&e->v())) {
    //       maximum_flow = flow;
    //       max_flow_vertex = &e->v();
    //       max_flow_edge = e;
    //     }
    //   }

    return new GCS_move(std::make_pair(s.current_node, current_neighbors[selected_edge]));
}

double GCS_state::get_path_cost() {
    return path_cost;
}

double GCS_state::compute_step_cost(std::string node, std::string next_node) {
    // at the moment, this is a centroid cost between the two polygons factored by the likelihood of it being a 
    // part of the shortest path (from GCS - MICP)
    auto element_list = swm->fm;
    point_type node_center;
    point_type next_node_center;

    boost::geometry::centroid(element_list[node].Polygon, node_center);
    boost::geometry::centroid(element_list[next_node].Polygon, next_node_center);

    
    // get the activation variable (officially it should be binary, but because it is relaxed, it is relative)
    // i.e. between 0 and 1
    double flow = 1.0;

    // auto edge_id;

    // gcs.Edges[edge_id].GetSolutionPhiXu() // flow probability that Xu exists in the shortest path
    // gcs.Edges[edge_id].GetSolutionPhiXv() // same for Xv

    // auto phi_var = gcs.Edges[edge_id].phi;

    return boost::geometry::distance(node_center, next_node_center) * flow;
}

void GCS_state::print() const {
    std::cout << "Current Node: " << current_node << std::endl;
    std::cout << "Current Pose: (" << current_pose.position.x << ", " << current_pose.position.y << ")" << std::endl;
}

// bool GCS_state::player1_turn() const {

// }
