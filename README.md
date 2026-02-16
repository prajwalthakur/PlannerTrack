# PlannerTrack 

PlannerTrack is a modular multi-agent motion planning and control framework designed for structured environments such as autonomous driving and racing. It supports custom scenario generation, along with advanced control strategies including Model Predictive Control (MPC) and Model Predictive Contouring Control (MPCC).

Key Features

1. Modular Vehicle Modeling
Supports multiple vehicle kinematic and dynamic models through a modular architecture, enabling flexible experimentation and system extensibility.

2. Reusable Vehicle Library
The vehicle abstraction is implemented as a shared/static C++ library, allowing seamless integration with planning and control algorithms while maintaining clear separation between modeling and decision layers.

3. Multi-Agent & Heterogeneous Support
Enables simulation and coordination of multiple vehicles—either homogeneous or heterogeneous—within the same environment.

4. Scenario Configuration via YAML
Provides configurable scenario generation using .yaml files, making it easy to define structured road layouts, vehicle parameters, and experiment setups.


# Docker Image Installation 
1. From root of the directory  run  `./scripts/.build/build.sh`
After above two steps the Docker Image with the name of **mp_ros2** would have been created

# To run the docker Container

3. From root of the directory run `./scripts/.deploy/devel.sh`

# steps to launch the demo for lane change ( In progress )
1. run `ros2 launch lane_change_example lane_change_example.launch.py` to launch the lane change example.
3. run `ros2 topic pub /vehicle_1/control_command project_utils/msg/EigenVector "data: [1.0, 0.0]"`  to control vehicle with id 1 . Input command (acc, steering angle)
4. run `ros2 topic pub /vehicle_2/control_command project_utils/msg/EigenVector "data: [0.8, 0.0]"` to control vehicle with id 2 .

Demo Video:

https://github.com/user-attachments/assets/fe5b9a02-f122-407f-aee5-424504de4339



## License
MIT  


## Use Case
I initiated this project to independently study algorithms and software development for autonomous vehicle systems. This repository is also available for your personal use in studying, education, research, or development.

If this project supports your work or contributes to your tasks, please feel free to inform me by starring the repository.


## Contribution
Any contribution by creating an issue or sending a pull request is welcome!! 
<!-- Please check [this document about how to contribute](/HOWTOCONTRIBUTE.md).   -->



## Author
[Prajwal Thakur](https://github.com/prajwalthakur) 


## Independent Work Declaration

PlannerTrack was initiated and developed independently by Prajwal Thakur
prior to current employment.

This project is maintained as a personal open-source initiative,
developed outside the scope of employment, using personal time and
equipment. It is based solely on publicly available research and
standard algorithms.

This repository does not contain any proprietary, confidential, or
employer-owned intellectual property.
