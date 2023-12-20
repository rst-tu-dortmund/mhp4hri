# Moving Horizon Planning for Human-Robot Interaction
This repository provides an optimization-based robot trajectory planner (and control support for the UR10) considering obstacles and humans in the environment.

We utilize a Moving Horizon Planning approach (similar to MPC) to optimize the robot trajectory based on cost functions regarding the environment.
The environment definition of this repository supports various types of obstacles:
   - static obstacles
   - dynamic obstacles
   - humans

Furthermore, the motion of dynamic obstacles and humans can be predicted and is considered in trajectory optimization. 
An uncertainty estimation approach is provided for an extrapolation approach to human motions to increase human safety.

The planner is provided as ROS packages for ROS Melodic and Noetic (Ubuntu 18 and Ubuntu 20).
Furthermore, a docker installation is provided for simple out-of-the-box usage of the planner (it may also run on systems other than Ubuntu).

Further information about the installation, usage, and other parts can be found in this repository in our [wiki](https://github.com/rst-tu-dortmund/mhp4hri/wiki).

## Citation
If you use this repository for your research, please cite our paper:

Heiko Renz, Maximilian Krämer, and Torsten Bertram. 2024. Moving Horizon Planning for Human-Robot Interaction. In Proceedings of the 2024 ACM/IEEE International Conference on Human-Robot Interaction (HRI ’24), March 11–14, 2024, Boulder, CO, USA. ACM, New York, NY, USA, 5 pages. https://doi.org/10.1145/3610977.3637476

## Acknowledgments
The authors gratefully acknowledge the financial support of the German Research Foundation (DFG, Project number: 497071854)

