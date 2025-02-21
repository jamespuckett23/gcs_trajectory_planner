#ifndef MCTS_GCS_H
#define MCTS_GCS_H

#include "state.h"
#include "Common.h"

#include "MCTS.h"

#include "geometry_msgs/msg/pose_array.hpp"
#include "sa_msgs/srv/semantic_map_in_a_region_server.hpp"

#include "mcts_graph_of_convex_sets.h"
#include "mcts_gcs_trajectory_optimization.h"
// #include <drake/planning/trajectory_optimization/gcs_trajectory_optimization.h>


struct GCS_move : public MCTS_move {
    // std::string node;
    geometry_msgs::msg::Pose curr_pose;
    geometry_msgs::msg::Pose next_pose;
    std::pair<std::string, std::string> action; // from curr_node to goal_node

    // TODO: consider how to distinguish from discrete and continuous action spaces
        // - maybe record current_node and cuurent_position, then list goal_node and goal_position. You'd 
        // have to check if its a valid move (i.e. neighboring nodes). You could list current_neighbors as
        // well. This might be very memory intensive (use ptrs?)

    GCS_move(std::pair<std::string, std::string> a) : action(a) {}
    bool operator==(const MCTS_move& other) const override {
        const GCS_move &o = (const GCS_move &) other;
        return action.first == o.action.first && action.second == o.action.second;
    }
};

class GCS_state : public MCTS_state {
    // char board[3][3]{};
    // bool player_won(char player) const;
    // char calculate_winner() const;
    // char turn, winner;
    // void change_turn();

    SemMap* swm;
    drake::planning::trajectory_optimization::MctsGcsTrajectoryOptimization* gcs;

    // GCS_state *previous_state;
    std::vector<std::string> path_to_current_node; // vector of node names

    std::string current_node;
    geometry_msgs::msg::Pose current_pose;

    std::string goal_node;
    geometry_msgs::msg::Pose goal_pose;

    double path_cost;

    // note where you've been
    std::vector<std::string> current_path_to_node;

    double take_action(const GCS_move *move);
    
    double get_path_cost();

    double compute_step_cost(std::string node, std::string node_next);

    queue<MCTS_move *> *generate_all_moves() const;

    GCS_move* pick_random_move(GCS_state &s, uniform_real_distribution<double> &dist, default_random_engine &gen, std::vector<std::string> &visited_nodes) const;

public:
    GCS_state();
    GCS_state(const GCS_state &other);
    // char get_turn() const;
    // char get_winner() const;
    bool is_terminal() const override;
    MCTS_state *next_state(const MCTS_move *move) const override;
    queue<MCTS_move *> *actions_to_try() const override;
    double rollout() const override;                        // the rollout simulation in MCTS
    void print() const override;
    bool player1_turn() const override { return true; }
};

#endif // MCTS_GCS_H