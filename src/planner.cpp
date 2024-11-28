#include <math.h>
#include <random>
#include <vector>
#include <array>
#include <algorithm>

#include <tuple>
#include <string>
#include <stdexcept>
#include <regex> // For regex and split logic
#include <iostream> // cout, endl
#include <fstream> // For reading/writing files
#include <assert.h> 
#include <chrono>
#include <utility>
#include <thread>
#include <GLFW/glfw3.h>
#include "mujoco/mujoco.h"


using namespace std;
#define NUM_OF_JOINTS 16


// Callback for GLFW errors
void glfwErrorCallback(int error, const char* description) {
    std::cerr << "GLFW Error (" << error << "): " << description << std::endl;
}

// Keyboard callback to exit the simulation
void keyboardCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
    }
}

struct Node{
    double* angles;
	Node* parent;
	Node(double* angles, Node* parent) : angles(angles), parent(parent) {}
	Node() {
		angles = new double[NUM_OF_JOINTS];
		parent = nullptr;
	}
};

class RRTPlanner{
	
	public:
	mjModel* model;
	mjData* data;
	double* start_anglesV_rad;
	double* goal_anglesV_rad;
	double eps;
	vector<Node*> tree;
	vector<pair<int, int>> allowed_collisions;


	RRTPlanner(mjModel* model, mjData* data, double* start_anglesV_rad, double* goal_anglesV_rad, double eps, vector<pair<int, int>> allowed_collisions)
		: model(model), data(data), start_anglesV_rad(start_anglesV_rad), goal_anglesV_rad(goal_anglesV_rad), eps(eps), allowed_collisions(allowed_collisions) {}
	
	Node* random_config(Node* q_goal){
		// Generate a random configuration with some probability, make random config as goal
		double bias = (double)(rand()) / RAND_MAX;
		if(bias <= 0.1){
			return q_goal;
		}

		// TODO : Account for joint limits
		double* random_angles = new double[NUM_OF_JOINTS];
		for(int i = 0; i<NUM_OF_JOINTS; i++){
			random_angles[i] = ((double)rand() / RAND_MAX) * 2 * M_PI;
		}

		Node* random_node = new Node(random_angles, nullptr);
		return random_node;
	}

	bool is_valid_state(mjModel* model, mjData* data, const double* joint_states, vector<pair<int, int>>& allowed_collisions){
		// Take the joint states and apply them to the model
		for (int i = 0; i < NUM_OF_JOINTS; i++) {
			data->qpos[i] = joint_states[i];
		}

		// Step the simulation
		mj_forward(model, data);

		// Check for collisions
		int contact_count = data->ncon;
		std::cout << "Contacts detected: " << contact_count << std::endl;

		// No collisions detected, then its a valid state
		if (contact_count == 0) {
			return true;
		}

		// If there is a collision, check if the bodies in contact are allowed to collide
		for (int j = 0; j < contact_count; ++j) {
			const mjContact& contact = data->contact[j];
			int geom1 = contact.geom1;
			int geom2 = contact.geom2;


			std::cout << "  Contact " << j << ": "
				<< "geom1 = " << geom1
				<< ", geom2 = " << geom2
				<< ", dist = " << contact.dist
				<< ", normal = [" << contact.frame[0] << ", " << contact.frame[1] << ", " << contact.frame[2] << "]"
				<< std::endl;

			// This is assuming the mujoco returns a sorted pair of geom ids, thus we sort the  pair and then check for collision        
			pair<int, int> current_collision_body = {min(geom1, geom2), max(geom1, geom2)};

			if (std::find(allowed_collisions.begin(), allowed_collisions.end(), current_collision_body) == allowed_collisions.end()) {
				return false;
			}
		}
		return true;
	}


	Node* build_rrt(int K){
		Node* q_init = new Node(start_anglesV_rad, nullptr);
		tree.push_back(q_init);

		Node* q_goal = new Node(goal_anglesV_rad, nullptr);

		for(int i = 0; i<K; i++){
			Node* random_node = random_config(q_goal);
			
			pair<bool, Node*> result = extend(tree, random_node);

			if (result.first){
				if (distance(result.second->angles, q_goal->angles) <= (double)1e-3){
					tree.push_back(new Node(goal_anglesV_rad, result.second));
					cout << "Goal reached!" << endl;
					return tree.back();
				}
			}
		}
		cout << "Goal not reached" << endl;
		return nullptr;
	}

	pair<bool, Node*> extend(vector<Node*>& tree, Node* q){
		
		Node* q_near = nearest_neighbor(tree, q);
		Node* q_new = new Node();

		if(new_config(q, q_near, q_new)){
			// Add edge
			q_new->parent = q_near;
			// Add vertex
			tree.push_back(q_new);
			return make_pair(true, q_new);
		}
		return make_pair(false, q_near);
	}

	Node* nearest_neighbor(vector<Node*>& tree, Node* random_node){
		double min_dist = numeric_limits<double>::max();
		Node* nearest_node = nullptr;
		for(Node* node : tree){
			double dist = distance(node->angles, random_node->angles);
			if(dist < min_dist){
				min_dist = dist;
				nearest_node = node;
			}
		}
		return nearest_node;
	}

	double distance(double* angles1, double* angles2){
		double dist = 0;
		for(int i = 0; i<NUM_OF_JOINTS; i++){
			dist += (angles1[i] - angles2[i])*(angles1[i] - angles2[i]);
		}
		return sqrt(dist);
	}

	bool new_config(Node* q, Node* q_near, Node* q_new){
		double dist = distance(q->angles, q_near->angles);
		double step_size = min(eps, dist)/dist;
		bool advance = false;

		for(double alpha = step_size; alpha <= 1; alpha += step_size){
			Node* temp_node = new Node();
			for(int i = 0; i<NUM_OF_JOINTS; i++){
				temp_node->angles[i] = q_near->angles[i] + alpha * (q->angles[i] - q_near->angles[i]);
			}
			if(is_valid_state(model, data, temp_node->angles, allowed_collisions)){
				advance = true;
				*q_new = *temp_node;
				delete temp_node;
				break;
			}
			else{
				delete temp_node;
				break;
			}
		}

		return advance;
	}

	// // TODO : Implement Path Shortening
	void extract_path(Node* final_node, double*** plan, int* planlength){
		int len = 0;
		Node* current = final_node;
		while(current != nullptr){
			len++;
			current = current->parent;
		}

		*planlength = len;
		*plan = (double**) malloc(len*sizeof(double*));
		current = final_node;
		for(int i = len-1; i>=0; i--){
			(*plan)[i] = current->angles;
			current = current->parent;
		}
	}		
};

int main()
{
	// Load the model
	mjModel* model = mj_loadXML("../scene_right.xml", NULL, NULL, 0);

	if (!model) {
		std::cerr << "Failed to load model: " << std::endl;
		// mj_deactivate();
		return 1;
	}

	mjData* data = mj_makeData(model);

	// Setup visualization 
	if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW." << std::endl;
        mj_deleteData(data);
        mj_deleteModel(model);
        // mj_deactivate();
        return 1;
    }

    cout << "GLFW initialized successfully." << endl;

    glfwSetErrorCallback(glfwErrorCallback);

    // Create GLFW window
    GLFWwindow* window = glfwCreateWindow(1200, 900, "MuJoCo Collision Checker", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window." << std::endl;
        glfwTerminate();
        mj_deleteData(data);
        mj_deleteModel(model);
        // mj_deactivate();
        return 1;
    }

    cout << "Window created successfully." << endl;

    glfwMakeContextCurrent(window);
    glfwSetKeyCallback(window, keyboardCallback);
    glfwSwapInterval(1); // Enable vsync

    // Create MuJoCo visualization context
    mjvScene scene;
    mjv_defaultScene(&scene);
    mjv_makeScene(model, &scene, 1000);

    mjrContext context;
    mjr_defaultContext(&context);
    mjr_makeContext(model, &context, mjFONTSCALE_150);

    // Initialize camera
    mjvCamera camera;
    mjv_defaultCamera(&camera);

    mjvOption opt;
    mjv_defaultOption(&opt);


	// Initialize the planner
	double start_anglesV_rad[NUM_OF_JOINTS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double goal_anglesV_rad[NUM_OF_JOINTS] = {0.193, 1.03, 0.862, 0.373, 0.103, 0.96, 1.03, 0.363, 0.0, 0.933, 1.02, 0.566, 1.07, 0.802, 0.847};
	vector<pair<int, int>> allowed_collisions = {{12, 43}, {22, 43}, {32, 43}, {42, 43}};

	RRTPlanner planner(model, data, start_anglesV_rad, goal_anglesV_rad, 0.1, allowed_collisions);

	Node goal_node = Node(goal_anglesV_rad, nullptr);
	Node* result = planner.build_rrt(1000);

	double** plan;
	int planlength;
	// extract path
	if (result) {
        planner.extract_path(result, &plan, &planlength);
	}
    else{
        cout << "No Goal Found" << endl;
    }

	cout << "Plan Length: " << planlength << endl;

	cin.get(); 
	
	if (result) {
		for (int i = 0; i < planlength; i++) {
			for (int j = 0; j < NUM_OF_JOINTS; j++) {
				data->qpos[j] = plan[i][j];
			}
			mj_forward(model, data);

			// Render scene
			mjrRect viewport = {0, 0, 1200, 900};
			mjv_updateScene(model, data, &opt, nullptr, &camera, mjCAT_ALL, &scene);
			mjr_render(viewport, &scene, &context);

			// Swap OpenGL buffers
			glfwSwapBuffers(window);
			glfwPollEvents();

			// Add a small delay to visualize the motion
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	}

	// Cleanup
    mjr_freeContext(&context);
    mjv_freeScene(&scene);
    glfwDestroyWindow(window);
    glfwTerminate();
    mj_deleteData(data);
    mj_deleteModel(model);
}