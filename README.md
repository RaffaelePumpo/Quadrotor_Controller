# Quadrotor Controller

## Description

This uavcontrol.cpp , C++ code, implements a drone controller for simulation in Gazebo, specifically designed for the Gazebo Fortress environment. The controller is intended for use with the UAVSim class and enables the drone to follow a circular trajectory or reach a desired point in 3D space. The code incorporates functionalities for position and orientation control based on a PD controller.

## Prerequisites

Before running the code, ensure that you have the following software installed:

1. **Gazebo Fortress**: The simulation environment for the drone. Download and install it from the official Gazebo website [here](https://gazebosim.org/docs/fortress/install).

2. **Ignition Transportation Plugin (version 11)**: Download and install the Ignition Transportation plugin version 11 from the Ignition Robotics website [here](https://gazebosim.org/api/transport/11.0/installation.html).

## Dependencies

The code utilizes the following C++ libraries:

- **Eigen3**: A C++ template library for linear algebra. Find it at [https://eigen.tuxfamily.org](https://eigen.tuxfamily.org).

- **matplotlibcpp**: A C++ wrapper for the Matplotlib plotting library. Obtain it at [https://github.com/lava/matplotlib-cpp](https://github.com/lava/matplotlib-cpp).

Ensure these libraries are installed before compiling and running the code.

## Features

- **Trajectory Generation**: The code includes a circular trajectory generation function, allowing the drone to follow circular paths in 3D space.

- **Position and Orientation Control**: Functions for position and orientation control use a PD controller to guide the drone to a desired point and maintain a desired orientation.

- **Arena Interaction**: The drone controller is designed to work within the Gazebo Fortress environment, making it suitable for arena-like simulations.

## Usage

1. Clone this repository inside a ros2 workspace:

   ```bash
   git clone https://github.com/RaffaelePumpo/Quadrotor_Controller-AUVE.git
2. Do a colbuild of the ros2 workspace
3. launch file :
   ```bash
   ros2 launch uav_simulation.launch.py
5. Build and run the c++ file (I have done with QTcreator for ros2)


## Additional Notes

- **Parameter Customization**: Adjust drone parameters such as mass, gravity, and controller gains directly in the code to meet specific requirements.

- **Trajectory Options**: The code offers flexibility by supporting both circular trajectory following and the ability to reach a specified point in space. This versatility allows for experimentation with different simulation scenarios.

- **Simulation Frequency**: The simulation loop operates at a frequency of 200Hz, ensuring a consistent and responsive simulation environment.
