# PlannerTrack 
PlannerTrack is a Motion planning and control for structured environments such as autonomous driving and racing, including lane following, lane changes, MPC, and MPCC.
1. A modularized package supporting various vehicle dynamics/kinematics models. 
2. The vehicle class is implemented as a shared/static library and leverages CppAD for automatic differentiation, enabling seamless integration with existing planning algorithms.



This repository contains the various motion planning algorthims , including obstacle avoidance strategies in ros2 humble.

List of Vehicles
1. Single Track Ackerman Steering Kinematics Model


List of Motion Planning Algorithms

1. Quadratic Programming (QP) Based Model Predictive Control
2. QP based Model Predictive Contouring Control
3. Model Predictive Path Integral Contol (MPPI) based Path Tracking Algorithm
4. Frenet Frame Based Non-Linear MPC 

List of Obstacle Avoidance Algorithm
1. Corridor Based Obstacle Avoidance ( can be used with 1,2,3 motion planning )
2 . Control Barrier Function (CBF) , only be used with non-linear MPC 

# Docker Image Installation 
1. Change the root directory name in `scripts/deploy/base.sh` 
2. From root of the directory  run  `./scripts/build/build.sh`
After above two steps the Docker Image with the name of **mp_ros2** would have been created

# To run the docker Conatiner

3. From root of the directory run `./scripts/deploy/devel.sh`

# steps to launch the demo
1. run `ros2 launch vehicle_interface vehicle_interface.launch.py` to launch the vehicle interface node
2. run `ros2 launch visualizer visualizer.launch.py` to visualize the sim
3. run `ros2 topic pub /ego_command project_utils/msg/EigenVector data:\ [0.04,-0.002]\ `  input command (acc, steering angle)


## License
MIT  


## Use Case
I initiated this project to independently study algorithms and software development for autonomous vehicle systems. This repository is also available for your personal use in studying, education, research, or development.

If this project supports your work or contributes to your tasks, please feel free to inform me by starring the repository.


## Contribution
Any contribution by creating an issue or sending a pull request is welcome!! 
<!-- Please check [this document about how to contribute](/HOWTOCONTRIBUTE.md).   -->




## Update-2
## visualizer node has been implemented


https://github.com/user-attachments/assets/94ec17ad-1860-4e79-a23e-117c120975bd



## Update-1
## vehicle Interface node has been implemented



<!-- https://github.com/user-attachments/assets/4257c3ff-0bd3-4313-8ca6-d322eb27ab34 -->






## Author
[Prajwal Thakur](https://github.com/prajwalthakur) 

