#include "visualizer.h"
#include <iostream>
#include <thread>
#include <chrono>

using namespace std;

Visualizer::Visualizer(mjModel* model, mjData* data) : model(model), data(data) {}

void Visualizer::setupVisualization()
{
    // Setup visualization 
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW." << std::endl;
        mj_deleteData(data);
        mj_deleteModel(model);
    }

    cout << "GLFW initialized successfully." << endl;

    glfwSetErrorCallback(Visualizer::glfwErrorCallback);
    // Create GLFW window
    window = glfwCreateWindow(1200, 900, "MuJoCo Collision Checker", nullptr, nullptr);

    if (!window) {
        std::cerr << "Failed to create GLFW window." << std::endl;
        glfwTerminate();
        mj_deleteData(data);
        mj_deleteModel(model);
    }

    cout << "Window created successfully." << endl;

    glfwMakeContextCurrent(window);
    glfwSetKeyCallback(window, Visualizer::keyboardCallback);
    glfwSwapInterval(1); // Enable vsync

    // Create MuJoCo visualization context
    mjv_defaultScene(&scene);
    mjv_makeScene(model, &scene, 1000);

    mjr_defaultContext(&context);
    mjr_makeContext(model, &context, mjFONTSCALE_150);

    // Initialize camera
    mjv_defaultCamera(&camera);

    mjv_defaultOption(&opt);
}

void Visualizer::updateScene()
{
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

void Visualizer::cleanupVisualization()
{
    mjr_freeContext(&context);
    mjv_freeScene(&scene);
    glfwDestroyWindow(window);
    glfwTerminate();
    mj_deleteData(data);
    mj_deleteModel(model);
}

// Callback for GLFW errors
void Visualizer::glfwErrorCallback(int error, const char* description) {
    std::cerr << "GLFW Error (" << error << "): " << description << std::endl;
}

// Keyboard callback to exit the simulation
void Visualizer::keyboardCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
    }
}