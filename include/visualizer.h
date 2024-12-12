#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <string>
#include <GLFW/glfw3.h>
#include "mujoco/mujoco.h"

// Forward declarations for MuJoCo types

class Visualizer
{
public:
    // Constructor
    Visualizer(mjModel* model, mjData* data);

    // Setup the visualization environment
    void setupVisualization();

    // Update the visualization scene (render)
    void updateScene();

    // Cleanup resources when done
    void cleanupVisualization();

    void init_camera();

private:
    // Member variables
    mjModel* model;
    mjData* data;
    GLFWwindow* window;
    mjvScene scene;
    mjrContext context;
    mjvCamera camera;
    mjvOption opt;

    // Callback for GLFW errors
    static void glfwErrorCallback(int error, const char* description);

    // Keyboard callback to exit the simulation
    static void keyboardCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
};

#endif // VISUALIZER_H
