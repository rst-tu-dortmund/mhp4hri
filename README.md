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

## Feature Branch "Next-Best-Trajectory"
This branch presents a new feature for environment observation and is currently only uploaded for a review process.
The feature is called "Next-Best-Trajectory" and is a feature that allows the robot to observe the environment and plan a trajectory that optimizes the robot's view towards a point of interest (POI).
Therefore, we build up an online local information distribution in a voxel map (UfoMap) and use this information to plan a trajectory that optimizes the robot's view toward a POI.

The feature is under review and has not yet merged into the main branch.
Furthermore, a few limitations in comparison to the main branches are present:
- only ROS Noetic support
- no docker support (at least not tested)
- no full simulation support (the robot simulation is running, but we currently do not have a camera simulation to update the environment during a simulation)

Furthermore, the new feature extends the UfoMap implementation (https://github.com/UnknownFreeOccupied/ufomap, BSD 2-Clause License). Until the feature is merged into the main branch, the UfoMap is not in the references wiki page but only referenced here:

D. Duberg and P. Jensfelt, "UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown," in IEEE Robotics and Automation Letters, vol. 5, no. 4, pp. 6411-6418, Oct. 2020, doi: 10.1109/LRA.2020.3013861. keywords: {Octrees;Three-dimensional displays;Solid modeling;Path planning;Collision avoidance;Robot sensing systems;Mapping;RGB-D perception;motion and path planning}, 

If you want to apply the feature with a real camera, you must use a camera that provides at least a point cloud and a color image. The feature is currently tested with an Azure Kinect camera. 
Microsoft provides the corresponding ROS package under the MIT License (https://github.com/microsoft/Azure_Kinect_ROS_Driver).
It is not included by default so that every user can decide whether to get the package, depending on his camera or if already self-written drivers exist.
A new dependency in addition to the already listed is Eigen Rand: https://bab2min.github.io/eigenrand/v0.5.0/en/index.html, MIT License.

In order to use the GPU-accelerated version of the information distribution, you need to install the CUDA Toolkit (https://developer.nvidia.com/cuda-downloads). 
It is tested with version 11.8.

## Citation
If you use this repository for your research, please cite our paper:

Heiko Renz, Maximilian Krämer, and Torsten Bertram. 2024. Moving Horizon Planning for Human-Robot Interaction. In Proceedings of the 2024 ACM/IEEE International Conference on Human-Robot Interaction (HRI ’24), March 11–14, 2024, Boulder, CO, USA. ACM, New York, NY, USA, 5 pages. https://doi.org/10.1145/3610977.3637476 [Accepted for publication]

## Acknowledgments
The authors gratefully acknowledge the financial support of the German Research Foundation (DFG, Project number: 497071854)

Furthermore, we gratefully acknowledge the previous work of Christoph Rösmann and Nabil Miri's help while testing this repository.
