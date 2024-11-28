#include <chrono>
#include <utility>
#include <algorithm>
#include <vector>
#include <iostream>
#include <GLFW/glfw3.h>
#include "mujoco/mujoco.h"



using namespace std;

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


bool is_valid_state(mjModel* model, mjData* data, const vector<float>& joint_states, vector<pair<int, int>>& allowed_collisions){
    // Take the joint states and apply them to the model
    for (int i = 0; i < joint_states.size(); i++) {
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

int main(){

    // Load the model
    mjModel* model = mj_loadXML("../scene_right.xml", NULL, NULL, 0);

    if (!model) {
        std::cerr << "Failed to load model: " << std::endl;
        // mj_deactivate();
        return 1;
    }

    cout << "Model loaded successfully." << endl;

    // Make data
    mjData* data = mj_makeData(model);
    if (!data) {
        std::cerr << "Failed to create mjData." << std::endl;
        mj_deleteModel(model);
        // mj_deactivate();
        return 1;
    }

    cout << "Data created successfully." << endl;

    // vector<float> actions = {0.193, 1.03, 0.862, 0.373, 0.103, 0.96, 1.03, 0.363, 0.0, 0.933, 1.02, 0.566, 1.07, 0.802, 0.847};
    vector<float> actions = {3.14, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    vector<pair<int, int>> allowed_collisions = {{12, 43}, {22, 43}, {32, 43}, {42, 43}};

    auto start = std::chrono::high_resolution_clock::now();
    if (is_valid_state(model, data, actions, allowed_collisions)) {
        cout << "Valid state" << endl;
    }
    else {
        cout << "Invalid state" << endl;
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    cout << "Time taken: " << duration.count() << " microseconds" << endl;
    
//    // Apply some actions
//     std::vector<double> actions(model->nu, 0.0); // Initialize action vector with zeros
//     actions[0] = 0.5;                            // Apply torque to the first actuator
//     actions[1] = -0.5;                           // Apply torque to the second actuator

//     for (int i = 0; i < model->nu; i++) {
//         data->ctrl[i] = actions[i];
//     }

//     // Simulate for a few steps
//     const int steps = 100;
//     for (int i = 0; i < steps; ++i) {
//         mj_step(model, data);

//         // Check for collisions
//         int contact_count = data->ncon;
//         std::cout << "Step " << i << ": " << contact_count << " contacts detected." << std::endl;

//         // Print collision details
//         for (int j = 0; j < contact_count; ++j) {
//             const mjContact& contact = data->contact[j];
//             std::cout << "  Contact " << j << ": "
//                       << "geom1 = " << contact.geom1
//                       << ", geom2 = " << contact.geom2
//                       << ", dist = " << contact.dist
//                       << ", normal = [" << contact.frame[0] << ", " << contact.frame[1] << ", " << contact.frame[2] << "]"
//                       << std::endl;
//         }
//     }

    // Cleanup
    mj_deleteData(data);
    mj_deleteModel(model);
    // mj_deactivate();

// ---------------------------------- Visualization -------------------------------------------------
// Initialize GLFW
    // if (!glfwInit()) {
    //     std::cerr << "Failed to initialize GLFW." << std::endl;
    //     mj_deleteData(data);
    //     mj_deleteModel(model);
    //     // mj_deactivate();
    //     return 1;
    // }

    // cout << "GLFW initialized successfully." << endl;

    // glfwSetErrorCallback(glfwErrorCallback);

    // // Create GLFW window
    // GLFWwindow* window = glfwCreateWindow(1200, 900, "MuJoCo Collision Checker", nullptr, nullptr);
    // if (!window) {
    //     std::cerr << "Failed to create GLFW window." << std::endl;
    //     glfwTerminate();
    //     mj_deleteData(data);
    //     mj_deleteModel(model);
    //     // mj_deactivate();
    //     return 1;
    // }

    // cout << "Window created successfully." << endl;

    // glfwMakeContextCurrent(window);
    // glfwSetKeyCallback(window, keyboardCallback);
    // glfwSwapInterval(1); // Enable vsync

    // // Create MuJoCo visualization context
    // mjvScene scene;
    // mjv_defaultScene(&scene);
    // mjv_makeScene(model, &scene, 1000);

    // mjrContext context;
    // mjr_defaultContext(&context);
    // mjr_makeContext(model, &context, mjFONTSCALE_150);

    // // Initialize camera
    // mjvCamera camera;
    // mjv_defaultCamera(&camera);

    // mjvOption opt;
    // mjv_defaultOption(&opt);

    // // Main visualization loop
    // while (!glfwWindowShouldClose(window)) {
    //     // Measure time taken for each step
    //     auto start = std::chrono::high_resolution_clock::now();

    //     // Apply actions
    //     std::vector<double> actions(model->nu, 0.0);
    //     actions[0] = 0.193;
    //     actions[1] = 1.03;
    //     actions[2] = 0.862;
    //     actions[3] = 0.373;
    //     actions[4] = 0.103;
    //     actions[5] = 0.96;
    //     actions[6] = 1.03;
    //     actions[7] = 0.363;
    //     actions[8] = 0.0;
    //     actions[9] = 0.933;
    //     actions[10] = 1.02;
    //     actions[11] = 0.566;
    //     actions[12] = 1.07;
    //     actions[13] = 0.802;
    //     actions[14] = 0.847;
            
    //     for (int i = 0; i < model->nu; i++) {
    //         data->ctrl[i] = actions[i];
    //     }

    //     // Step the simulation
    //     mj_step(model, data);

    //     // Check for collisions
    //     int contact_count = data->ncon;
    //     std::cout << "Contacts detected: " << contact_count << std::endl;

    //     // Print collision details
    //     for (int j = 0; j < contact_count; ++j) {
    //         const mjContact& contact = data->contact[j];
    //         std::cout << "  Contact " << j << ": geom1 = " << contact.geom1
    //                 << ", geom2 = " << contact.geom2
    //                 << ", dist = " << contact.dist << std::endl;
    //     }

    //     // Render scene
    //     mjrRect viewport = {0, 0, 1200, 900};
    //     mjv_updateScene(model, data, &opt, nullptr, &camera, mjCAT_ALL, &scene);
    //     mjr_render(viewport, &scene, &context);

    //     // Swap OpenGL buffers
    //     glfwSwapBuffers(window);
    //     glfwPollEvents();
    //     // std::cin.get();
    //     // Measure time taken for each step
    //     auto end = std::chrono::high_resolution_clock::now();
    //     std::chrono::duration<double> elapsed = end - start;
    //     std::cout << "Time taken for step: " << elapsed.count() << " seconds" << std::endl;
    // }


    // // Cleanup
    // mjr_freeContext(&context);
    // mjv_freeScene(&scene);
    // glfwDestroyWindow(window);
    // glfwTerminate();
    // mj_deleteData(data);
    // mj_deleteModel(model);
    // mj_deactivate();

    return 0;
}