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

#### 1. `tune_cart_pendulum` Node
- **Purpose**: Tunes the controller of the cart-pole system.
- **Functionality**:
  - Collects relevant pendulum pose data by listening to the `/gazebo/link_states` topic and updating the system state accordingly.
  - Uses mutex locks to ensure safe update/control behavior between the thread responsible for updating the system state and the one running the controller.
  - Publishes the data using a custom message of type `ControlPoseData` to the `/inverted_pendulum/control_pose_data` topic.
  - Provides a service `/inverted_pendulum/evaluate_controller` for evaluating a potential set of controller gains. The service uses a custom service of type `CandidateSolution`. The service returns the pendulum and cart tracking errors.
  -  Runs a simulation in Gazebo for a specific amount of time (this can be set when launching the node using roslaunch by passing setting the argument `sim_time`).  
- **Launch file**: `inverted_pendulum_pkg/launch/tunning_controller.launch`.

#### 2. `tuned_controller` Node
- **Purpose**: Computes the control action using the optimized gains based on the current state of the inverted pendulum.
- **Functionality**:
  - Computes the control action. The node subscribes to the `/inverted_pendulum/link+states`topic to obtain the update pendulum state.
  - Safe multithreading and fixed update and control rates are ensured.
  - Sends the control action to Gazebo by publishing to `/inverted_pendulum/joint_cart_controller/command` provided by `gazebo_ros_control` plugin.  
- **Launch file**: `inverted_pendulum_pkg/launch/tuned_controller.launch`


#### 4. `optimizer_node` Node
- **Purpose**: Implements the "learning from experience" process through Genetic Algorithm (GA) optimization through the deap python library.
- **Functionality**:
  - Contains a client to `/inverted_pendulum/evaluate_controller` for evaluating the population individuals.   
  - Updates the population for a specific number of generations (the optimization parameters can only be changed by modifying the script `inverted_pendulum_pkg/scripts/optimizer.py`. This is going to be modified in the future. )
- **Launch file**: `inverted_pendulum_pkg/launch/optimizer.launch`

![Image](inverted_pendulum_pkg/images/nodes_flow.svg)

### Directories breakdown
**Note**: This section provides a breakdown of the project subdirectories and files. It might be skipped if the reader is not interested in the specifics of the project.
#### 1. nodes directory

| file | purpose |
|-----------------|-----------------|
| tune_cart_pendulum.cpp     | source code of  `tunning_controller` node    |
| run_tuned_controller.cpp    | source code of  `controller_node` node     |
| optimizer.py     |  source code of  `optimizer_node` node    |

#### 2. src

| file | purpose |
|-----------------|-----------------|
| gains_loader.cpp     | utility code to load the optimized gains of the controller    |
| gains_saver.py     | utility code to save the optimized gains of the controller    |

#### 3. include

| file | purpose |
|-----------------|-----------------|
| gains_loader.h     | header containing the decalration of the gains_loader utility code  (used for better code packaging)

#### 4. launch

| file | purpose |
|-----------------|-----------------|
| rviz.launch    | visualizes the inverted pendulum in Rviz   |
| world.launch    | launches empty world in Gazebo. Other launch files use it. Enables accelerating the simulation (up to 4 times currently) through accl_flag|
| spawn.launch     |  spawns the inverted pendulum model. Other launch files use it     |
| inverted_pendulum_control.launch | launches empty world, spawns the inverted pendulum, starts the controller_manager (part of gaebo_ros_control), and starts the tuning process |
| tunning_controller.launch | launches the `tunning_controller` node |
| tuned_controller.launch |  launches the `tuned_controller` node |
| optimizer.launch | launches the  `optimizer_node`` node |

#### 5. msg
| file | purpose |
|-----------------|-----------------|
| ControlPoseData.msg | Definition of the `ControlPoseData` type|

#### 6. srv
| file | purpose |
|-----------------|-----------------|
| CandidateSolution.srv | Definition of the `CandidateSolution` type|


#### 7. urdf
| file | purpose |
|-----------------|-----------------|
| robot.urdf |  Robot description using urdf markup|
| robot.xacro | Robot description using urdf markup. Differs from robot.urdf by exploiting xacro (xml macros) package for better code organization |
| robot_properties.xacro | Contains definitions of gazebo materials, xacro macros, and other properties. This gets included in robot.xacro using `xacro:include` |

__Note__: The current (updated package) uses the robot decription provided by "robomania" as it looks much better than the model I defined. This can be found in the robot_description that I attached along side my package for convenience. I intend to provide a more visually appealing model during future updates. However, I did not find that necessary at the moment.

#### 8. config
| file | purpose |
|-----------------|-----------------|
| inverted_pendulum_control.yaml |  Sets gazebo_ros_control interface through setting the joint_state_publisher and joint_cart_controller |
| my_config.rviz | Stores an Rviz configuration data. the purpose is to avoid the need for adding a robot and a tf objects manually whenever Rviz is launched |

**Note**: joint_cart_controller is of type `effort_controllers/JointEffortController`. For more details on this, refer to [ros_control](https://wiki.ros.org/ros_control) and this [tutorial](https://classic.gazebosim.org/tutorials?tut=ros_control). 

#### 9. CMakeLists.txt

 This file defines the build process. It manages dependencies and enables building projects across different platforms. A few parts of the CMakeLists.txt file are highlighted and explained below:
##### bring dependencies into the build
```
find_package(Python3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  xacro
  message_generation
  control_toolbox
)
find_package(yaml-cpp REQUIRED)

catkin_python_setup()
```
##### generating custom messages and services
```
add_message_files(
  FILES
  ControlPoseData.msg
)
add_service_files(
  FILES
  UpdatePIDParams.srv
  CandidateSolution.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

```
##### generating executables from cpp source files
```
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

add_executable(tunning_controller nodes/cpp/tune_cart_pendulum.cpp)
add_executable(tuned_controller nodes/cpp/run_tuned_controller.cpp src/cpp/gains_loader.cpp)

add_dependencies(tunning_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(tuned_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

```
##### specifying libraries used for linking the targets (executables) to their dependencies
```
target_link_libraries(tunning_controller
  ${catkin_LIBRARIES}
)
target_link_libraries(tuned_controller
  PUBLIC
  ${catkin_LIBRARIES}
  PRIVATE 
  ${YAML_CPP_LIBRARIES}
)
```

##### Mark executable scripts (Python etc.) for installation
```
catkin_install_python(PROGRAMS
  nodes/python/optimizer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS
  nodes/python/debug_roslaunch.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

#### 9. package.xml
Provides meta-information about the package. It also provides high-level dependency management. Please refer to the [package.xml](inverted_pendulum_pkg/package.xml) file for more details.
## Using the package 
#### Building the package 
Clone the repo to your project folder
```
git clone "https://github.com/mbakr99/inverted-pendulum-control.git"

```
#### Restructure the directory 

Then, in a shell, execute the following:
```
mv inverted-pendulum-control/ catkin_ws/
cd catkin_ws
catkin_make

```
#### Apply the changes  

Source the generated setup.bash file to reflect the new build:
```
source catkin_ws/devel/setup.bash

```
#### Visualizing the inverted pendulum in Rviz

```
roslaunch inverted_pendulum_pkg rviz.launch

```

**Note**: At first, the model colors will not be set until a _Fixed frame_ link is set by going to the _Global options_ under the _Display_ panel.
This command also launches the `joint_state_publisher_gui`, enabling the user to simulate joint movement.

#### Tuning the model
```
roslaunch inverted_pendulum_pkg inverted_pendulum_control.launch tune_flag:=true accl_flag:=false

```


This command starts Gazebo, spawns the model, and takes care of applying the control actions to the model. Due to using `gazebo_ros_control` [plugin](https://classic.gazebosim.org/tutorials?tut=ros_gzplugins) and launching the controller manager, the topic `/inverted_pendulum/joint_cart_controller/command` should appear after using `rostopic list`.



At first, the controller struggles to keep the pendulum in the desired upright position while maintaining the cart at the center of the plate.  
![first_gen](inverted_pendulum_pkg/images/first_generations.gif)

 However, as the optimization progresses, the controller gets better 

## Experience that will save you a lot of time :upside_down_face:

- `joint_state_publisher` from `ros_control` , wich is responsible for publishing joint states, crashes when you reset the gazebo simulation. This is why I had to implement the `update_pend_pose_data` node that publishes the pose data.
- The importance of logging in debugging your code. I can't emphaisze how important this is! By comparing the expected behavior (through logging) with what you are actually getting you can determine or narrow down the possible sources of the :bug:.
- Using different logging levels to be able to filter different messages using `rqt_console`.
- Save simulation time ⏱️ when possible! Don't think, "Oh, this costs almost no time; making it more efficient will not matter". It, generally, does. I saved a lot of time by doing the following:
  + I avoided pausing the simulation at the end of each run. This meant I didn't have to unpause the simulation at the beginning of the next iteration (saved around 1 seconds/run -> 1s x population_size x number of generations = hours of simulation).
- It is very important to know the rate at which the callback functions are being executed. In my case, for example, one of the callback functions was used to accumulate the tracking error. At the beginning, I did not set a desired rate to run the callback function, which resulted in inconsistent tracking error results (two identical simulation runs will result in different tracking errors).
- Trust and develop your engineering intuition. I mentioned that I wanted to set a tilt in the pendulum position to make it fall faster. At first, I used `gazebo/set_link_state` service to set the pendulum at 5 degrees. I noticed that there was a jerk in the pendulum motion at the beginning, and I felt that something was wrong. However, I convinced myself that it might be that the controller gains were off at first. I later found out that setting the link states might cause an unexpected behavior by the solver. I solved that by applying a small torque instead.
- You should know that the fixed link that you use as a reference for tf to work properly should be named' world'. I spent a lot of time reading my urdf file over and over again to find the answer [here](https://answers.ros.org/question/393006/urdf-link-not-properly-fixed-to-world/).
- Use the experience of others, do not reinvent the wheel. I am not saying copy someones work. However, a good deal of the problems and bugs that you will face have been experienced by someone else. I noticed that this [work](https://youtu.be/dLnKvFEnSBw?si=Jh9mYsgKbguqkwBv) experienced the same `joint_state_publisher` I mentioned above.
- The importance of mutithreading can not be empahsized enough. I am used to modeling and designign controllers in Simulink, where you just have to focus on the design part and Simulnik takes care of the rest (after setting appropriate sampling rates of course). However, when implementing the same system from scratch, one has to pay attention to the rate at which the system state is available and the rate at which the controller is running. In addition, the correct state should be used. What do I mean by that? Let us say, for some reason, the state information arrives at times [1.0s, 1.1s, 1.2s, 1.5s, 1.6s]. Without proper synchronizationa control action at time 1.5s might use the state information from 1.2s leading to a delay in the control loop, which migh lead to unstable closed-loop system.       
  
   
      
