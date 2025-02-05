#include "../include/gcs_trajectory_planner/MCTS_GCSPlanner.h"


GCS_state::GCS_state() {
    // initalize the tree
    // run algorithm
}

GCS_state::GCS_state(const GCS_state &other) {

}

bool GCS_state::is_terminal() const {

}

MCTS_state *GCS_state::next_state(const MCTS_move *move) const {

}

queue<MCTS_move *> *GCS_state::actions_to_try() const {

}

double GCS_state::rollout() const {

}

void GCS_state::print() const {

}

bool GCS_state::player1_turn() const {

}


struct GCS_move : public MCTS_move {
    int x, y;
    char player;
    GCS_move(int x, int y, char p) : x(x), y(y), player(p) {}
    bool operator==(const MCTS_move& other) const;
};