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

#include "RRTStarPlanner.h"
#include "visualizer.h"


using namespace std;

// void globalPlanner()
// {

// }

void localPlanner(mjModel* model, mjData* data, Visualizer& visualizer)
{
    // Initialize the planner
    int num_of_dofs {16};
	double start_anglesV_rad[num_of_dofs] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double goal_anglesV_rad[num_of_dofs] = {0.193, 1.03, 0.862, 0.373, 0.103, 0.96, 1.03, 0.363, 0.0, 0.933, 1.02, 0.566, 1.07, 0.802, 0.847};
	vector<pair<int, int>> allowed_collisions = {{12, 43}, {22, 43}, {32, 43}, {42, 43}};

	RRTStarPlanner planner(model, data, start_anglesV_rad, goal_anglesV_rad, 0.1, allowed_collisions, 0.5, num_of_dofs);

    // // Build the plan appropriately
	// Node goal_node = Node(goal_anglesV_rad, nullptr);
	// Node* result = planner.build_rrt_star(1000);

	// double** plan;
	// int planlength;
	// // extract path
	// if (result) {
    //     planner.extract_path(result, &plan, &planlength);
	// }
    // else{
    //     cout << "No Goal Found" << endl;
    // }

	// cout << "Plan Length: " << planlength << endl;

    // if (result) {
    //     for (int i = 0; i < planlength; i++) {
    //         for (int j = 0; j < num_of_dofs; j++) {
    //             data->qpos[j] = plan[i][j];
    //         }
    //         mj_forward(model, data);
    //         visualizer.updateScene();
    //     }
    // }
}

int main()
{
	// Load the model
	mjModel* model = mj_loadXML("../scene_right.xml", NULL, NULL, 0);

	if (!model) {
		std::cerr << "Failed to load model: " << std::endl;
		return 1;
	}

	mjData* data = mj_makeData(model);

    Visualizer visualizer(model, data);
    visualizer.setupVisualization();

    localPlanner(model, data, visualizer);

    visualizer.cleanupVisualization();
}