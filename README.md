# OOP_project
Requirements:
```txt
pybullet >= 3.0.6
dqrobotics >= 20.4.0.24
numpy >= 1.19.3
matplotlib >= 3.3.2
pynput >= 1.7.1
termcolor >= 1.1.0
plotly >= 4.12.0
scikit-image >= 0.17.2
open3d >= 0.11.1
pytest >= 6.1.2
pyyaml >=5.3.1
pandas >=1.1.4
pyglm==0.4.8b1
seaborn
```

Sure! Here's a more natural and concise version of the README file for your script:

---

# Gripper Experimentation Framework

This script simulates a series of gripper experiments using PyBullet, designed to test the grasping and lifting capabilities of different gripper models.

## Classes Overview

### `Gripper` (Abstract Base Class)
The `Gripper` class is the base for all gripper types. It defines the necessary methods for gripper operations:
- `grasp(target)`: Grasp the target object.
- `preshape()`: Prepares the gripper for grasping.
- `openGripper(position)`: Opens the gripper to a specified position.
- `getJointPosition()`: Returns the current joint positions of the gripper.
- `experiment_setup()`: Initializes the gripper in the 3D space.
- `lifting()`: Lifts the gripper after a successful grasp.

### `three_finger_gripper`
This class extends `Gripper` and implements the grasping mechanisms for a three-finger gripper. It includes:
- Grasp, preshape, and lift operations.
- A method for opening the gripper.
- Setup and joint position management.
- Visualizing the 3D space with debug lines.

### `experiment`
This class handles individual experiments, where a gripper performs a grasping and lifting task on an object. It includes:
- Spawning the object in the simulation.
- Checking whether the grasp was successful by comparing the object’s position before and after lifting.

### `BatchExperiment`
This class runs multiple `experiment` objects simultaneously. It supports:
- Initializing multiple experiments within a grid.
- Running the grasping operations for all experiments.
- Saving results to CSV files.

## Features
- Simulate various gripper types (currently supports a 3-finger gripper).
- Test object grasping and lifting success in 3D space.
- Automatically generate random positions and orientations for experiments.
- Save results to CSV for analysis.

## Usage

1. **Setting Up the Experiment**  
   Initialize the `BatchExperiment` class with parameters like the batch number, number of experiments, and gripper name. The class will handle the creation of the experiment environment and manage the PyBullet connection.

2. **Experiment Execution**  
   After setting up the experiment space, you can trigger the `batchGraspingOperation()` method to run multiple experiments and collect the results.

3. **Results**  
   Results are saved in a CSV file, with each row representing a gripper position and its grasp success (1 for success, 0 for failure).

## Requirements
- PyBullet (`pip install pybullet`)
- Numpy (`pip install numpy`)
- Matplotlib (`pip install matplotlib`)

## File Structure
- `Robots/grippers/threeFingers/sdh.urdf`: 3-finger gripper model.
- `share/cube_small.urdf`: Object models for the experiments (e.g., small cube).
- `utils/thing.py`: Contains utility functions for the experiment (e.g., `Thing` class).

## Notes
- This framework assumes you are running PyBullet in either GUI or shared memory mode.
- You can extend it with additional gripper and object models.
  
For more details, feel free to dive into the code!

--- 

Let me know if you need further adjustments!

Code to generate all the plots, as well as the Classifier:
```
OOP_data_folder/all_the_stuff.ipynb
```

