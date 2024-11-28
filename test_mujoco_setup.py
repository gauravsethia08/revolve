import time

import mujoco
import numpy as np
import mujoco.viewer

m = mujoco.MjModel.from_xml_path('scene_right.xml')
d = mujoco.MjData(m)

# print(np.shape(d.qpos))
with open('demo1.txt', 'r') as file:
  demo_data = file.read().splitlines()[0].split(',')

with mujoco.viewer.launch_passive(m, d) as viewer:
    # Close the viewer automatically after 30 wall-seconds.
    start = time.time()
    while viewer.is_running() and time.time() - start < 30:
        step_start = time.time()

        # mj_step can be replaced with code that also evaluates
        # a policy and applies a control signal before stepping the physics.
        mujoco.mj_step(m, d)
    
        d.ctrl[:] = np.array(demo_data)
        # Apply random control signal
        # d.ctrl[:] = np.random.uniform(-3, 3, m.nu)

        # Print contact information
        print(f'Number of contacts: {d.ncon}')
        for i in range(d.ncon):
            contact = d.contact[i]
            geom1 = contact.geom1 
            geom2 = contact.geom2
            print(f'Contact {i}:')
            print(f'  Geom1: {geom1}')
            print(f'  Geom2: {geom2}')
            print(f'  Position: {contact.pos}')
            print(f'  Distance: {contact.dist}')
            
        # Example modification of a viewer option: toggle contact points every two seconds.
        with viewer.lock():
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)

        # Pick up changes to the physics state, apply perturbations, update options from GUI.
        viewer.sync()

        # Rudimentary time keeping, will drift relative to wall clock.
        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)