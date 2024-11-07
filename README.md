# Project Revolve
This project is about developing a motion planning algorithm for a 4 fingered hand. We use Pen-Spin use-case to demonstrate the capabilites. 

While this sounds cool, penspin is a complicated problem and we might not be able to achieve that during the duration of the [course](https://www.cs.cmu.edu/~maxim/classes/robotplanning_grad/). Thus we first develop the motion planning algorithm to move the fingers to a given contact point on the surface of the pen. (To simplify we might get the goal in config space, reducing the complexity of inverse kinematics). 

The 2 possible approaches we have in mind are; 1. High Dimensional varient of RRT; 2. Multi-agent planning, assuming each finger to be an individual agent. 

### 1. Installation
1. Download MuJoCo 3.2.5 from [here](https://github.com/google-deepmind/mujoco/releases)

2. Clone the repositry 
```bash 
git clone git@github.com:gauravsethia08/revolve.git
```

3. Create and start a virtual environment
```bash
cd revolve
python -m venv mj_env
source mj_env/bin/activate
```

4. Install the required libraries
```bash
pip install -r requirements.txt
```

5. Test the setup - The ideal behaviour is, the fingers should be applying random actions
```bash 
python test_mujoco_setup.py
```

### Project Management (To be discussed)
1. Create an issue and assign it to person responsible for that
2. Ideally create a branch for each issue
3. Issues merging the branch would automatically close the issue

### Contributors
[Gaurav Sethia](https://github.com/gauravsethia08), [Shara Lawande](https://github.com/gauravsethia08), [Shrudhi Ramesh Shanthi](https://github.com/shrudh-i),  [Siddharth Ghodosara](https://github.com/SiddharthGhodasara), [Sruthi Mukkamala](https://github.com/sruthi-mukk)