// #ifndef PLANNER_H
// #define PLANNER_H

// using namespace std;

// #define NUM_OF_JOINTS 16
// struct Node{
//     double* angles;
//     Node* parent;
//     Node(double* angles, Node* parent) : angles(angles), parent(parent) {}
//     Node() {
//         angles = new double[NUM_OF_JOINTS];
//         parent = nullptr;
//     }
// };

// class RRTPlanner{  
//     public:
//     mjModel* model;
//     mjData* data;
//     double* start_anglesV_rad;
//     double* goal_anglesV_rad;
//     std::vector<Node*> tree;
//     std::vector<std::pair<int, int>> allowed_collisions;
//     double eps;
//     Node* random_config(Node* q_goal);
//     Node* nearest_neighbor(std::vector<Node*>& tree, Node* random_node);
//     double distance(double* angles1, double* angles2);
//     bool new_config(Node* q, Node* q_near, Node* q_new);
//     std::pair<bool, Node*> extend(std::vector<Node*>& tree, Node* q);
//     bool is_valid_state(mjModel* model, mjData* data, const double* joint_states, std::vector<std::pair<int, int>>& allowed_collisions);
//     Node* build_rrt(int);
//     void extract_path(Node* final_node, double*** plan, int* planlength);

//     RRTPlanner(mjModel* model, mjData* data, double* start_anglesV_rad, double* goal_anglesV_rad, double eps, std::vector<std::pair<int, int>> allowed_collisions)
//         : model(model), data(data), start_anglesV_rad(start_anglesV_rad), goal_anglesV_rad(goal_anglesV_rad), eps(eps), allowed_collisions(allowed_collisions) {}
// };

// #endif
#ifndef PLANNER_H
#define PLANNER_H

#include <vector>
#include <utility>
#include "mujoco/mujoco.h"

#define NUM_OF_JOINTS 16

struct Node {
    double* angles;
    Node* parent;
    Node(double* angles, Node* parent) : angles(angles), parent(parent) {}
    Node() {
        angles = new double[NUM_OF_JOINTS];
        parent = nullptr;
    }
};

class RRTPlanner {
public:
    mjModel* model;
    mjData* data;
    double* start_anglesV_rad;
    double* goal_anglesV_rad;
    std::vector<Node*> tree;
    std::vector<std::pair<int, int>> allowed_collisions;
    double eps;

    Node* random_config(Node* q_goal);
    Node* nearest_neighbor(std::vector<Node*>& tree, Node* random_node);
    double distance(double* angles1, double* angles2);
    bool new_config(Node* q, Node* q_near, Node* q_new);
    std::pair<bool, Node*> extend(std::vector<Node*>& tree, Node* q);
    bool is_valid_state(mjModel* model, mjData* data, const double* joint_states, std::vector<std::pair<int, int>>& allowed_collisions);
    Node* build_rrt(int K);
    void extract_path(Node* final_node, double*** plan, int* planlength);

    RRTPlanner(mjModel* model, mjData* data, double* start_anglesV_rad, double* goal_anglesV_rad, double eps, std::vector<std::pair<int, int>> allowed_collisions)
        : model(model), data(data), start_anglesV_rad(start_anglesV_rad), goal_anglesV_rad(goal_anglesV_rad), eps(eps), allowed_collisions(allowed_collisions) {}
};

#endif // PLANNER_H