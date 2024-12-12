# Project Revolve
This project is about developing a motion planning algorithm for a 4 fingered hand. We use Pen-Spin use-case to demonstrate the capabilites. 

While this sounds cool, penspin is a complicated problem and we might not be able to achieve that during the duration of the [course](https://www.cs.cmu.edu/~maxim/classes/robotplanning_grad/). Thus we first develop the motion planning algorithm to move the fingers to a given contact point on the surface of the pen. (To simplify we might get the goal in config space, reducing the complexity of inverse kinematics). 

The 2 possible approaches we have in mind are; 1. High Dimensional varient of RRT; 2. Multi-agent planning, assuming each finger to be an individual agent. 

### 1. Installation and Execution
1. Download MuJoCo 3.2.5 from [here](https://github.com/google-deepmind/mujoco/releases)

2. Clone the repositry 
```bash 
git clone git@github.com:gauravsethia08/revolve.git
```

3. Build
```bash
cd revolve/build
cmake --build . --config Release
./main ../src/config_{N}.txt
```


### 2. To Do's
- [x] Figure out valid contact pairs between different phalanges of the fingers + the palm. Add these into the collision detection model.
- [x] Appropriately tune the epsilon value for taking steps towards the random joint config. We need to balance taking small enough steps such that MuJoCo does not take too long to converge to finding a collision. Also need to balance that epsilon is not too small that it takes forever to get to q_rand and/or run into an obstacle.
- [x] Account for the actual joint limits when generating q_rand (i.e., don't generate a joint of pi if pi is not a physically possible state)
- [ ] Integrate IK for Franka Arm


### Contributors
[Gaurav Sethia](https://github.com/gauravsethia08), [Shara Lawande](https://github.com/gauravsethia08), [Shrudhi Ramesh Shanthi](https://github.com/shrudh-i),  [Siddharth Ghodosara](https://github.com/SiddharthGhodasara), [Sruthi Mukkamala](https://github.com/sruthi-mukk)
