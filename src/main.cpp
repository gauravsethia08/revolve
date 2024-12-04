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

void globalPlanner(mjModel* model, mjData* data, Visualizer& visualizer)
{
    int num_of_dofs {7};
    double start_anglesV_rad[num_of_dofs] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.793, 0.0};
    double goal_anglesV_rad[num_of_dofs] = {0.115885, 0.342909, -0.231399, -0.810337, 0.0577116, 1.39522, 2.89704};//{0.0, 0.331, -0.203, -0.835, -0.145, 1.83, -2.9}; //{-0.145, 0.423, 0, -1.57079, -1.57, 1.4, 0.464};
    
    // cout << "Body 0" << mj_id2name(model, mjOBJ_BODY, 7) << endl;
    // for(int i = 0; i < 250; i++){
    //     if (mj_id2name(model, mjOBJ_GEOM, i))
    //         cout << "Geom " << i << " " << mj_id2name(model, mjOBJ_GEOM, i) << endl;
    // }
    // cout << "Geom 0" << mj_id2name(model, mjOBJ_GEOM, 7) << endl;
    cout << "Length of data->qpos: " << model->nq << endl;

    vector<pair<int, int>> allowed_collisions = {{12,14}, {14,16}, {16,21}, {21,26},{26,30}, {26,31}, {26,32}, {30,31}, {30, 32}, {31,32}, {30, 50}, {31,50}, {32,50}, {50, 59}};

    RRTStarPlanner planner(model, data, start_anglesV_rad, goal_anglesV_rad, 0.1, allowed_collisions, 0.5, num_of_dofs, false);

    // Build the plan appropriately
    Node goal_node = Node(goal_anglesV_rad, nullptr);
    Node* result = planner.build_rrt_star(1000);

    double** plan;
    int planlength{-1};
    // extract path
    if (result) {
        planner.extract_path(result, &plan, &planlength);
    }
    else{
        cout << "No Goal Found" << endl;
    }

    cout << "Plan Length: " << planlength << endl;

    if (result) {
        for (int i = 0; i < planlength; i++) {
            for (int j = 0; j < num_of_dofs; j++) {
                data->qpos[j] = plan[i][j];
            }
            // mj_forward(model, data);
            mj_step(model, data);
            visualizer.updateScene();
        }
    }
}

void localPlanner(mjModel* model, mjData* data, Visualizer& visualizer)
{
    // Initialize the planner
    int num_of_dofs {16};
	double start_anglesV_rad[num_of_dofs] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double goal_anglesV_rad[num_of_dofs] = {-0.00725476, 0.287009, -0.0878901, -0.0324261, -0.00516717, 0.355593, -0.0676449, -0.0278528, 0.00202279, 0.38239, -0.00351541, -0.00111891, 1.39599, 0.165781, 0.725218, 1.15223}; //{0.0, 0.924, 0.165, -0.061, 0.0, 1.06, 0.0, 0.0, 0.0, 1.09, 0.0, 0.0, 0.767, 1.16, 1.4, 0.0};//{0.193, 1.03, 0.862, 0.373, 0.103, 0.96, 1.03, 0.363, 0.0, 0.933, 1.02, 0.566, 1.07, 0.802, 0.847};
	vector<pair<int, int>> allowed_collisions = {{61, 102}, {63, 102}, {65, 102}, {67, 102}, {69, 102}, {71, 102}, {73, 102}, {75, 102}, {77,102}, 
                                                {79,102}, {81,102}, {83,102}, {85,102}, {87, 102}, {89, 102}, {91, 102}, {93, 102},
                                                {95,102}, {97,102}, {99,102}, {101,102},
                                                {61, 103}, {63, 103}, {65, 103}, {67, 103}, {69, 103}, {71, 103}, {73, 103}, {75, 103}, {77,103}, 
                                                {79,103}, {81,103}, {83,103}, {85,103}, {87, 103}, {89, 103}, {91, 103}, {93, 103},
                                                {95,103}, {97,103}, {99,103}, {101,103}};

	RRTStarPlanner planner(model, data, start_anglesV_rad, goal_anglesV_rad, 0.1, allowed_collisions, 0.5, num_of_dofs, true);

    // Build the plan appropriately
	Node goal_node = Node(goal_anglesV_rad, nullptr);
	Node* result = planner.build_rrt_star(1000);

	double** plan;
	int planlength{-1};
	// extract path
	if (result) {
        planner.extract_path(result, &plan, &planlength);
	}
    else{
        cout << "No Goal Found" << endl;
    }

	cout << "Plan Length: " << planlength << endl;

    if (result) {
        for (int i = 0; i < planlength; i++) {
            for (int j = 0; j < num_of_dofs; j++) {
                data->qpos[7+j] = plan[i][j];
            }
            // mj_forward(model, data);
            mj_step(model, data);
            visualizer.updateScene();
            
        }
    }
}

void allPlanner(mjModel* model, mjData* data, Visualizer& visualizer)
{
    // Initialize the planner
    int num_of_dofs {23};
	double start_anglesV_rad[num_of_dofs] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.793, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double goal_anglesV_rad[num_of_dofs] = {0.0578656, 0.00484065, -9.67339e-05, -1.55885, -0.000475122, 2.09537, 2.8968, -0.552715, 0.0559791, 1.44565, 0.42832, -0.293626, 0.113725, 1.37835, 0.442406, -0.0584939, 0.217719, 1.26196, 0.380962, 1.39196, -0.108564, 1.21286, -0.104381}; //{0.115885, 0.342909, -0.231399, -0.810337, 0.0577116, 1.39522, 2.89704, -0.00725476, 0.287009, -0.0878901, -0.0324261, -0.00516717, 0.355593, -0.0676449, -0.0278528, 0.00202279, 0.38239, -0.00351541, -0.00111891, 1.39599, 0.165781, 0.725218, 1.15223}; //{0.0, 0.924, 0.165, -0.061, 0.0, 1.06, 0.0, 0.0, 0.0, 1.09, 0.0, 0.0, 0.767, 1.16, 1.4, 0.0};//{0.193, 1.03, 0.862, 0.373, 0.103, 0.96, 1.03, 0.363, 0.0, 0.933, 1.02, 0.566, 1.07, 0.802, 0.847};
	vector<pair<int, int>> allowed_collisions = {{12,14}, {14,16}, {16,21}, {21,26},{26,30}, {26,31}, {26,32}, {30,31}, {30, 32}, {31,32}, {30, 50}, {31,50}, {32,50}, {50, 59},
                                                // Hand
                                                {61, 102}, {63, 102}, {65, 102}, {67, 102}, {69, 102}, {71, 102}, {73, 102}, {75, 102}, {77,102}, 
                                                {79,102}, {81,102}, {83,102}, {85,102}, {87, 102}, {89, 102}, {91, 102}, {93, 102},
                                                {95,102}, {97,102}, {99,102}, {101,102},
                                                {61, 103}, {63, 103}, {65, 103}, {67, 103}, {69, 103}, {71, 103}, {73, 103}, {75, 103}, {77,103}, 
                                                {79,103}, {81,103}, {83,103}, {85,103}, {87, 103}, {89, 103}, {91, 103}, {93, 103},
                                                {95,103}, {97,103}, {99,103}, {101,103}};


	RRTStarPlanner planner(model, data, start_anglesV_rad, goal_anglesV_rad, 0.1, allowed_collisions, 0.5, num_of_dofs, false);

    // Build the plan appropriately
	Node goal_node = Node(goal_anglesV_rad, nullptr);
	Node* result = planner.build_rrt_star(1000);

	double** plan;
	int planlength{-1};
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
            for (int j = 0; j < num_of_dofs; j++) {
                data->qpos[j] = plan[i][j];
            }
            // mj_forward(model, data);
            mj_step(model, data);
            visualizer.updateScene();
        }
    }
}
int main()
{
	// Load the model
	mjModel* model = mj_loadXML("../panda_with_hand.xml", NULL, NULL, 0);

	if (!model) {
		std::cerr << "Failed to load model: " << std::endl;
		return 1;
	}

	mjData* data = mj_makeData(model);

    Visualizer visualizer(model, data);
    visualizer.setupVisualization();

    allPlanner(model, data, visualizer);

    cin.get();

    // globalPlanner(model, data, visualizer);

    // cin.get();

    // localPlanner(model, data, visualizer);


    visualizer.cleanupVisualization();
}