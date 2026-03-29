# F1Tenth Stanley Controller

Welcome to the **F1Tenth Stanley Controller** project. This repository contains a ROS 2 workspace designed for autonomous racing using the Stanley control algorithm on the F1Tenth simulation platform.

## 🚀 Overview

This project provides a robust implementation of the Stanley steering controller, integrated with the F1Tenth Gym and its ROS 2 bridge.

## 📁 Repository Structure

- `src/`: Source code for the controller and simulators.
  - `f1tenth_gym/`: The core simulator engine.
  - `f1tenth_gym_ros/`: ROS 2 bridge for the simulator.
- `LICENSE`: MIT License.
- `README.md`: Project documentation.

## 🛠️ Installation

To set up this workspace, follow these steps:

1. **Clone the repository with submodules:**
   ```bash
   git clone --recursive <repository-url>
   cd stanley_ws
   ```

2. **Install dependencies:**
   Make sure you have ROS 2 (Humble or Foxy recommended) installed.
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the workspace:**
   ```bash
   colcon build
   source install/setup.bash
   ```

## 🏎️ Usage

Start the F1Tenth simulation:
```bash
ros2 launch f1tenth_gym_ros sim.yaml
```

(Add more specific usage instructions for your Stanley controller here!)

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
