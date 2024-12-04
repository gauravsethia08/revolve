#include "RRTStarPlanner.h"

// Node constructor that takes in joint angles and a parent node
Node::Node(double* angles, Node* parent) : angles(angles), parent(parent), cost(0) {}

// Default Node constructor that initializes joint angles array and sets parent to nullptr
Node::Node(int num_of_dofs) {
    angles = new double[num_of_dofs]; // Allocate space for joint angles
    parent = nullptr;
    cost = 0; // Default cost is zero
}

RRTStarPlanner::RRTStarPlanner(mjModel* model, mjData* data, double* start_anglesV_rad, double* goal_anglesV_rad, 
                double eps, vector<pair<int, int>> allowed_collisions, double neighborhood_radius, int num_of_dofs)
    : model(model), data(data), start_anglesV_rad(start_anglesV_rad), goal_anglesV_rad(goal_anglesV_rad),
        eps(eps), allowed_collisions(allowed_collisions), neighborhood_radius(neighborhood_radius), NUM_OF_DOFS(num_of_dofs) {}

Node* RRTStarPlanner::build_rrt_star(int K) {
    Node* q_init = new Node(start_anglesV_rad, nullptr);
    q_init->cost = 0;
    tree.push_back(q_init);
    Node* q_goal = new Node(goal_anglesV_rad, nullptr);

    for (int i = 0; i < K; i++) {
        Node* q_rand = random_config(q_goal);
        Node* q_nearest = nearest_neighbor(tree, q_rand);
        Node* q_new = new Node(NUM_OF_DOFS);

        if (new_config(q_rand, q_nearest, q_new)) {
            vector<Node*> q_near = near_vertices(q_new);
            Node* q_min = q_nearest;
            double c_min = q_nearest->cost + cost(q_nearest, q_new);

            for (Node* q_near_node : q_near) {
                if (obstacle_free(q_near_node, q_new) && 
                    q_near_node->cost + cost(q_near_node, q_new) < c_min) {
                    q_min = q_near_node;
                    c_min = q_near_node->cost + cost(q_near_node, q_new);
                }
            }

            q_new->parent = q_min;
            q_new->cost = c_min;
            tree.push_back(q_new);

            rewire(q_new, q_near);

            if (distance(q_new->angles, q_goal->angles) <= 1e-3) {
                cout << "Goal reached!" << endl;
                return q_new;
            }
        }
    }

    cout << "Goal not reached" << endl;
    return nullptr;
}

void RRTStarPlanner::rewire(Node* q_new, vector<Node*>& q_near) {
    for (Node* q_near_node : q_near) {
        if (q_near_node != q_new->parent && 
            obstacle_free(q_new, q_near_node) && 
            q_new->cost + cost(q_new, q_near_node) < q_near_node->cost) {
            q_near_node->parent = q_new;
            q_near_node->cost = q_new->cost + cost(q_new, q_near_node);
        }
    }
}

vector<Node*> RRTStarPlanner::near_vertices(Node* q) {
    vector<Node*> near_nodes;
    for (Node* node : tree) {
        if (distance(node->angles, q->angles) <= neighborhood_radius) {
            near_nodes.push_back(node);
        }
    }
    return near_nodes;
}

double RRTStarPlanner::cost(Node* q1, Node* q2) {
    return distance(q1->angles, q2->angles);
}

bool RRTStarPlanner::obstacle_free(Node* q1, Node* q2) {
    double dist = distance(q1->angles, q2->angles);
    int steps = ceil(dist / eps);
    for (int i = 0; i <= steps; i++) {
        double alpha = (double)i / steps;
        double* interpolated_angles = new double[NUM_OF_DOFS];
        for (int j = 0; j < NUM_OF_DOFS; j++) {
            interpolated_angles[j] = q1->angles[j] + alpha * (q2->angles[j] - q1->angles[j]);
        }
        if (!is_valid_state(model, data, interpolated_angles, allowed_collisions)) {
            delete[] interpolated_angles;
            return false;
        }
        delete[] interpolated_angles;
    }
    return true;
}

Node* RRTStarPlanner::random_config(Node* q_goal) {
    double bias = (double)(rand()) / RAND_MAX;
    if (bias <= 0.1) {
        return q_goal;
    }
    double* random_angles = new double[NUM_OF_DOFS];
    for (int i = 0; i < NUM_OF_DOFS; i++) {
        random_angles[i] = ((double)rand() / RAND_MAX) * 2 * M_PI;
    }
    Node* random_node = new Node(random_angles, nullptr);
    return random_node;
}

bool RRTStarPlanner::is_valid_state(mjModel* model, mjData* data, const double* joint_states, vector<pair<int, int>>& allowed_collisions) {
    for (int i = 0; i < NUM_OF_DOFS; i++) {
        data->qpos[i] = joint_states[i];
    }
    mj_forward(model, data);
    
    int contact_count = data->ncon;
    std::cout << "Contacts detected: " << contact_count << std::endl;
    
    if (contact_count == 0) {
        return true;
    }
    
    for (int j = 0; j < contact_count; ++j) {
        const mjContact& contact = data->contact[j];
        int geom1 = contact.geom1;
        int geom2 = contact.geom2;
        std::cout << " Contact " << j << ": "
                << "geom1 = " << geom1 << ", geom2 = " << geom2
                << ", dist = " << contact.dist
                << ", normal = [" << contact.frame[0] << ", " << contact.frame[1] << ", " << contact.frame[2] << "]"
                << std::endl;
        
        pair<int, int> current_collision_body = {min(geom1, geom2), max(geom1, geom2)};
        if (std::find(allowed_collisions.begin(), allowed_collisions.end(), current_collision_body) == allowed_collisions.end()) {
            return false;
        }
    }
    return true;
}

Node* RRTStarPlanner::nearest_neighbor(vector<Node*>& tree, Node* random_node) {
    double min_dist = numeric_limits<double>::max();
    Node* nearest_node = nullptr;
    for (Node* node : tree) {
        double dist = distance(node->angles, random_node->angles);
        if (dist < min_dist) {
            min_dist = dist;
            nearest_node = node;
        }
    }
    return nearest_node;
}

double RRTStarPlanner::distance(double* angles1, double* angles2) {
    double dist = 0;
    for (int i = 0; i < NUM_OF_DOFS; i++) {
        dist += (angles1[i] - angles2[i]) * (angles1[i] - angles2[i]);
    }
    return sqrt(dist);
}

bool RRTStarPlanner::new_config(Node* q, Node* q_near, Node* q_new) {
    double dist = distance(q->angles, q_near->angles);
    double step_size = min(eps, dist) / dist;
    bool advance = false;
    for (double alpha = step_size; alpha <= 1; alpha += step_size) {
        Node* temp_node = new Node(NUM_OF_DOFS);
        for (int i = 0; i < NUM_OF_DOFS; i++) {
            temp_node->angles[i] = q_near->angles[i] + alpha * (q->angles[i] - q_near->angles[i]);
        }
        if (is_valid_state(model, data, temp_node->angles, allowed_collisions)) {
            advance = true;
            *q_new = *temp_node;
            delete temp_node;
            break;
        } else {
            delete temp_node;
            break;
        }
    }
    return advance;
}

void RRTStarPlanner::extract_path(Node* final_node, double*** plan, int* planlength) {
    int len = 0;
    Node* current = final_node;
    while (current != nullptr) {
        len++;
        current = current->parent;
    }
    *planlength = len;
    *plan = (double**)malloc(len * sizeof(double*));
    current = final_node;
    for (int i = len - 1; i >= 0; i--) {
        (*plan)[i] = current->angles;
        current = current->parent;
    }
}
