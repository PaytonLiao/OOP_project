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
---

# 🤖 Gripper Experimentation Framework

A robust simulation and analysis framework for robotic gripper experiments, using PyBullet and machine learning, designed to evaluate and improve grasping and lifting capabilities.

---

## 🚀 Features at a Glance

- 🛠 **Simulate Grippers**: Test a variety of grippers with randomized initial poses.
- 🧠 **Train Classifiers**: Use SVMs and Feedforward Neural Networks to predict grasp success.
- 📊 **Data Visualization**: Analyze 3D orientations and grasp success patterns.
- 📈 **Batch Processing**: Efficient parallel simulations for rapid data generation.
- 🧩 **Modular Design**: Easily extendable with new grippers or objects.

---

## 🏗 Framework Overview

### OOP Architecture 🧩

- **Abstract Base Class**: Shared gripper methods like `grasp()` and `preshape()`.
- **Inheritance**: Specialized grippers like `ThreeFingerGripper`.
- **Polymorphism**: Unified interface for gripper implementations.
- **Composition & Aggregation**: Integration of grippers and experiments for modularity.

### Simulation Workflow (Run the main.py script) 🌀

1. **Initialize Environment**: Map objects and grippers to URDF files.
2. **Randomize Parameters**: Generate varied initial poses.
3. **Run Experiments**: Perform grasps, evaluate success, and record results.
4. **Analyze Results**: Train ML models and visualize performance.

---

## 🧪 Experiments

### Data Generation 📂

- **Randomized Poses**: Use normal and uniform distributions for gripper positions and orientations.
- **Balanced Dataset**: Oversample minority class for equal success/failure representation.
- **Feature Extraction**: Key parameters include 3D positions and Euler angles.

### Visualizations 📊

- **3D Point Clouds**: Analyze success clusters with arrows showing orientations.
- **Feature Distributions**: Kernel density plots for feature importance.
- **t-SNE**: Dimensionality reduction for feature clustering.

---

## 🧠 Classifiers

### 1️⃣ Feedforward Neural Network

- **Architecture**:
  - Input: 6 features (position + orientation).
  - Hidden Layers: Two fully connected layers (64 and 32 units).
  - Output: Binary classification with sigmoid activation.
- **Training**:
  - Optimizer: Adam.
  - Loss: Binary Cross-Entropy.
  - Validation Accuracy: Peaks at ~89% for rectangular objects.

### 2️⃣ Support Vector Machine

- **Parameter Tuning**:
  - Kernels: RBF and Linear.
  - Regularization (C): Tested values of 0.1, 1.0, and 10.0.
- **Results**:
  - Best Kernel: RBF with C=10.0.
  - Accuracy: Up to 88% for cube and rectangle datasets.

### Data Split Experiments 📈

- Varying train-validation ratios showed logarithmic accuracy improvements.
- SVM outperformed FNN for smaller datasets, achieving 80% accuracy with just 10% of the data.

---

## 📋 Requirements

Install dependencies with:

```bash
pip install pybullet numpy matplotlib torch scikit-learn
```

---

## 📂 File Structure

- **Grippers**: `Robots/grippers/threeFingers/sdh.urdf`.
- **Objects**: `share/cube_small.urdf`.
- **Utilities**: `utils/thing.py`.

---

## 🔍 Results and Insights

- **Success Rates**:
  - Cubes: Position dominates success.
  - Rectangles: Orientation plays a critical role.
- **Classifier Metrics**:
  - FNN: F1-Score ~0.89 for rectangles.
  - SVM: Superior for limited data, accuracy ~88%.
- **Visualization**:
  - Success clusters near object centers; failures at edges.

---

## 🎯 Limitations

- Simulations lack irregular objects (e.g., mugs, trophies).
- Scaling issues for >100 simultaneous grippers.
- Basic FNN architecture limits generalization.

---

## 🎉 Achievements

- Generalized framework for multiple grippers and objects.
- Efficient batch processing for rapid data generation.
- Strong ML performance with meaningful insights into grasp dynamics.

---

[GitHub repository](https://github.com/PaytonLiao/OOP_project.git). 🚀

---
