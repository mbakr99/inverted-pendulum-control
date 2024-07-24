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

### ROS (Controller + Optimization):
Four nodes run on ROS. A controller node  
