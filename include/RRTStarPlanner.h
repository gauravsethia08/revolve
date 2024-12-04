#ifndef RRT_STAR_PLANNER_H
#define RRT_STAR_PLANNER_H

#include <math.h>            // For mathematical functions like sqrt(), ceil(), and M_PI
#include <vector>            // For using std::vector
#include <algorithm>         // For std::find()
#include <iostream>          // For std::cout
#include <limits>            // For std::numeric_limits
#include <cstdlib>           // For rand()
#include <utility>           // For std::pair
#include "mujoco/mujoco.h"  // For MuJoCo API functions and types

using namespace std;

// Node struct to represent a configuration in the RRT* tree
struct Node {
    double* angles;   // Joint angles
    Node* parent;     // Parent node
    double cost;      // Cost from the start

    Node(double* angles, Node* parent);
    Node(int num_of_dofs);
};

// RRTStarPlanner class to implement RRT* algorithm
class RRTStarPlanner {
public:
    mjModel* model;  // MuJoCo model
    mjData* data;    // MuJoCo data
    double* start_anglesV_rad; // Start joint angles
    double* goal_anglesV_rad;  // Goal joint angles
    double eps;  // Step size
    vector<Node*> tree;  // Tree of nodes in the RRT* algorithm
    vector<pair<int, int>> allowed_collisions;  // Allowed collisions
    double neighborhood_radius;  // Radius for the neighborhood search
    int NUM_OF_DOFS;  // Number of DOFs in the planning problem
    bool is_local;  // Flag to indicate if the planner is local or global

    // Constructor to initialize the RRTStarPlanner
    RRTStarPlanner(mjModel* model, mjData* data, double* start_anglesV_rad, 
                   double* goal_anglesV_rad, double eps, 
                   vector<pair<int, int>> allowed_collisions, double neighborhood_radius, int num_of_dofs, bool is_local);

    // Function to build the RRT* tree
    Node* build_rrt_star(int K);

    // Function to rewire the tree after adding a new node
    void rewire(Node* q_new, vector<Node*>& q_near);

    // Function to find nearby nodes to a given node
    vector<Node*> near_vertices(Node* q);

    // Function to calculate the cost between two nodes
    double cost(Node* q1, Node* q2);

    // Function to check if a path between two nodes is obstacle-free
    bool obstacle_free(Node* q1, Node* q2);

    // Function to generate a random configuration (with goal biasing)
    Node* random_config(Node* q_goal);

    // Function to check if a configuration is valid
    bool is_valid_state(mjModel* model, mjData* data, const double* joint_states, 
                        vector<pair<int, int>>& allowed_collisions);

    // Function to find the nearest neighbor in the tree to a random node
    Node* nearest_neighbor(vector<Node*>& tree, Node* random_node);

    // Function to compute the distance between two configurations
    double distance(double* angles1, double* angles2);

    // Function to create a new configuration from a neighboring node
    bool new_config(Node* q, Node* q_near, Node* q_new);

    // Function to extract the path from the final node to the root
    void extract_path(Node* final_node, double*** plan, int* planlength);
};

#endif // RRT_STAR_PLANNER_H
