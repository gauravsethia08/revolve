import mujoco
import numpy as np

# Load the Allero right hand model
def load_model():
    model_path = "/home/gaurav/mujoco_menagerie/wonik_allegro/scene_right.xml"  # Update with your model path
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    return model, data

# Function to set joint values
def set_joint_values(data, joint_values):
    print(data.qpos)
    assert len(joint_values) == data.ne, "Joint values must match the number of joints."
    for i in range(data.ne):
        data.qpos[i] = joint_values[i]

# Rendering setup
def render(model, data):
    # Create a viewport and context
    viewport = mujoco.MjrRect(0, 0, 800, 600)  # Width and height of the window
    scn = mujoco.MjvScene(model, maxgeom=100)
    con = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_100)

    # Update the scene
    mujoco.mjv_updateScene(model, data, mujoco.mjtCatBit.mjtCatBitAll, scn)
    
    # Render the scene
    mujoco.mjr_render(viewport, scn, con)

# Main function
def main():
    model, data = load_model()

    # Example joint values (replace with your desired values)
    joint_values = np.array([0.0]*16)  # Adjust according to your model's DOF
    # set_joint_values(data, joint_values)

    # Step the simulation
    mujoco.mj_step(model, data)

    # Render the model
    render(model, data)

if __name__ == "__main__":
    main()
