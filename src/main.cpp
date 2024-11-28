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
#include "planner.h"


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