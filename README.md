# inverted-pendulum-control
## Overview:
This project focuses on controlling an inverted pendulum, a system comprising a cart that performs linear motion, and a pendulum capable of rotation. This system is often used for testing controllers due to its non-linear and inherently unstable nature. The primary objective is to maintain the pendulum in an upright position while keeping the cart at a desired location.
![Some](https://www.maplesoft.com/support/help/content/67/plot470.gif)

A common approach in the industry is to use a PID controller, known for its simplicity and effectiveness. In this implementation, two PID controllers are employed: one to maintain the pendulum's upright position and the other to drive the cart to the desired position. These controllers' actions are then combined. However, tuning such a controller can be challenging. Therefore, in this project, Genetic Algorithm (GA) optimization is utilized to learn the controller gains. Initially, random gains are set, and the simulation runs repeatedly, adjusting the gains based on the gathered experience. Given that the simulation represents a black-box function, GA optimization is an appropriate method for this task.

## PID:
A PID controller, which stands for Proportional-Integral-Derivative controller, is a control loop feedback mechanism widely used in industrial control systems. It continuously calculates an action value based on the error, the difference between a desired setpoint and a measured process variable. 

The PID controller output ![u(t)](https://latex.codecogs.com/svg.latex?u(t)) can be expressed as:

![PID Equation](https://latex.codecogs.com/svg.latex?u(t)=K_pe(t)+K_i\int_{0}^{t}e(\tau)d\tau+K_d\frac{d}{dt}e(t))

Where:
Where:
- ![Kp](https://latex.codecogs.com/svg.latex?K_p) is the proportional gain
- ![Ki](https://latex.codecogs.com/svg.latex?K_i) is the integral gain
- ![Kd](https://latex.codecogs.com/svg.latex?K_d) is the derivative gain
- ![et](https://latex.codecogs.com/svg.latex?e(t)) is the error value at time ![t](https://latex.codecogs.com/svg.latex?t)

An intuitive explanation is provided by Brian Douglas in this [video](https://youtu.be/UR0hOmjaHp0?si=htCSecChTK2qcQvb).

## Genetic Algorithm:
A Genetic Algorithm (GA) is an evolutionary heuristic optimization technique that mimics the process of natural selection. In each iteration, the GA maintains a population of potential solutions, where each solution, represented as a set of six PID gains in this case, is known as an individual. The population evolves over generations through two primary processes: mating and mutation. During mating, two individuals are combined to produce two new offspring, transforming the original individuals without changing the population size (in some implementations, mating increases the population size). Mutation applies random changes to individuals to enable exploration (controlled randomness **might** result in discovering a better solution). In our example, this is done by adding random values drawn from a normal distribution with a mean of 1 and a standard deviation of 1 to a subset of the solution's parameters (the PID gains). Both mating and mutation occur with user-defined probabilities. Ultimately, the principle of survival of the fittest is applied to select the optimal solution, ensuring that the best-performing individuals are retained and propagated through subsequent generations.

Mathworks provides a brief and good [explanation](https://www.mathworks.com/help/gads/what-is-the-genetic-algorithm.html) on GA.

## Implmentation using ROS + Gazebo:
The implementation involves modeling an inverted pendulum in Gazebo, while a controller and an optimizer run on ROS. Gazebo and ROS communicate seamlessly: ROS sends control actions to Gazebo, which updates the state of the inverted pendulum and sends the updated state back to ROS. This interaction is illustrated in the following figure:


![Figure](https://raw.githubusercontent.com/mbakr99/inverted-pendulum-control/5ae47a8c8ccd7bb28433a6fc85852af1aa9dce37/inverted_pendulum_pkg/images/inverted_pend_flow.svg)




### Nodes Overview

#### 1. `update_pend_pose_data` Node
- **Purpose**: Publishes the pendulum orientation and the cart position, which facilitates the exchange of relevant inverted pendulum data between Gazebo and ROS.
- **Functionality**:
  - Collects relevant pendulum pose data by listening to the `/gazebo/link_states` topic and calling the `/gazebo/get_link_state` service.
  - Publishes the data using a custom message of type `ControlPoseData` to the `/inverted_pendulum/control_pose_data` topic.
- **Launch file**: `inverted_pendulum_pkg/launch/update_pend_pose.launch`.

#### 2. `controller_node` Node
- **Purpose**: Computes the control action based on the current state of the inverted pendulum.
- **Functionality**:
  - Provides a service `/inverted_pendulum/set_pid_gains` for changing PID gains. The service uses a custom service of type `UpdatePIDParams` that stores a set of controller gains.
  - Computes the control action. The node subscribes to the `/inverted_pendulum/control_pose_data`topic to obtain the inverted pendulum state.
  - Sends the control action to Gazebo by publishing to `/inverted_pendulum/joint_cart_controller/command` provided by `gazebo_ros_control` plugin.  
- **Launch file**: `inverted_pendulum_pkg/launch/controller.launch`

#### 3. `compute_objective_node` Node
- **Purpose**: Evaluate a set of controller gains.
- **Functionality**:
  - Provides a service `/inverted_pendulum/compute_objective` for evaluating a potential set of controller gains. The service uses a custom service of type `CandidateSolution`. The service returns the pendulum and cart tracking errors.  
  - Contains a client to the `/inverted_pendulum/set_pid_gains`. The client parses the data stored in `CandidateSolution` into a `UpdatePIDParams` and sends it to the service server where the controller gains are updated.
  - Runs a simulation in Gazebo for a specific amount of time (this can be set when launching the node using roslaunch by passing setting the argument `sim_time`). 
  - Returns the tracking error at the end of the simulation, serving as an objective function that assesses the quality of the controller.
- **Launch file**: `inverted_pendulum_pkg/launch/compute_objective_launch.launch`

#### 4. `optimizer_node` Node
- **Purpose**: Implements the "learning from experience" process through Genetic Algorithm (GA) optimization.
- **Functionality**:
  - Contains a client to `/inverted_pendulum/compute_objective` for evaluating the population individuals.   
  - Updates the population for a specific number of generations (the optimization parameters can only be changed by modifying the script `inverted_pendulum_pkg/scripts/optimizer.py`. This is going to be modified in the future. )
- **Launch file**: `inverted_pendulum_pkg/launch/optimizer.launch`

![Image](inverted_pendulum_pkg/images/nodes_flow.svg)

### Directories breakdown

#### 1. src directory

| file | purpose |
|-----------------|-----------------|
| update_pend_pose.cpp     | source code of  `update_pend_pose_data` node    |
| control_cart_pendulum.cpp    | source code of  `controller_node` node     |
| compute_objective.cpp     |  source code of  `compute_objective_node` node    |

#### 2. scripts

| file | purpose |
|-----------------|-----------------|
| optimizer.py     | source code of  `optimizer_node` node    |

#### 3. launch

| file | purpose |
|-----------------|-----------------|
| update_pend_pose.cpp     | source code of  `update_pend_pose_data` node    |
| control_cart_pendulum.cpp    | source code of  `controller_node` node     |
| compute_objective.cpp     |  source code of  `compute_objective_node` node    |


