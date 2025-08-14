# Multi Hetero Agent Exploration by Unitree GOs 

Author: Sayantani Bhattacharya

## Project Overview

In times of disaster, every second counts, and reaching survivors in hazardous terrains poses significant challenges. Imagine a coordinated team of agile, four-legged robots, working together to navigate treacherous environments like dense forests or mines. These quadrupedal robots autonomously perform simultaneous localization and mapping (SLAM), creating real-time detailed maps of their surroundings. By employing decentralized collaborative system, these robots can share and merge their individual maps, creating a comprehensive understanding of the complete area without relying on a central system. This approach enhances the robustness and speed of search operations, as the failure of a single unit does not compromise the entire mission. Quadrupeds inherently work well in uneven terrains, and harnessing the strengths of SLAM to explore unmapped areas with LIDAR and Visual-Inertial sensor data, these robotic swarms represent a leap forward in disaster response, offering hope and assistance when it’s needed most. By all means this is just the first iteration and needs good work for being deployable onsite.
</br>
For more details about the project please refer my [Portfolio Post](https://sayantani-bhattacharya.github.io/posts/quadruped-fleet/)


## Block diagram

1. Complete system:</br>
<p align="center">
  <img src="/images/system_block.png" alt="Alt text" width="700"/>
</p>
 

<!-- 2. Individual cluster:</br>
    (subject to modifications) </br>
    <p align="right">
     <img src="/images/indv_block.png" alt="Alt text" width="700"/>
    </p> -->

2. System Architecture of GO2:</br>
    <p align="center">
     <img src="/images/go2.png" alt="Alt text" width="700"/>
    </p>

3. System Architecture of GO1:</br>
    <p align="center">
     <img src="/images/go1.png" alt="Alt text" width="700"/>
    </p>


## Tools and References:

### Hardware:
  - Unitree GO1 & GO2.
  - Zed 2i Camera & On-board 4D Lidar.
  - Jetson Orin Nano.
  - buck-convertor (24V->12V).
  - 3D print the mount for unitree.
  - Display port adapters for Jetson.
  - Ethernet cable for initial testing with unitree sdk.
  - Micro SD cards.

### Software : 
  - C++
  - ROS2 Jazzy and Humble
  - Python
  - Unitree SDK - GO1 and GO2
  - Zed SDK
  - Slam, RTabMap, and Nav2 pkg

<!-- ### Reference repositories:
  - Unitree Package abstracted layer developed by Nick Morales, Marno Nel, and Katie Hughes:  https://github.com/ngmor/unitree_nav 
  - Unitree ros2 wrapper: https://github.com/katie-hughes/unitree_ros2 
  - Visual slam route: https://github.com/GogiPuttar/Search-and-Rescue_Robot_Dog_Unitree_Go1
  - Graph based slam route: https://roy2909.github.io/Exploration/

### Reference papers for collaborative exploration:
  - Awesome paper: https://arxiv.org/pdf/2108.08325 || Has a lot of references, inside: see the annotations.
  - Paper of 12 drone c-slam, with github code: http://arxiv.org/pdf/2108.05756 
  - C-SLAM subproblems such as map merging (Lee et al., 2012), practical implementations (Kshirsagar et al., 2018), particle filter techniques.
  - ICRA 2023: generalized back-end for c-slam https://www.youtube.com/watch?v=oypURkSuMc4 

### ROS2-SDK setup for Unitree GO1
ROS2 was set up by cloning and following the instructions for [this](https://github.com/unitreerobotics/unitree_ros2) repository. 


# ROS2 Packages

## `go1_description`
Holds all the models for visualizing the Unitree Go1 in RViz and simulation in Gazebo.

## `go2_description`
Holds all the models for visualizing the Unitree Go2 in RViz and simulation in Gazebo.

## `ros2_unitree_legged_msgs`
Messages and services for controlling Unitree Go1.

## `unitree_exploration`
Frontier Exploration with the Unitree Go1. Takes in the map, finds frontiers and commands 2-D goal poses to the Nav Stack accordingly.

## `unitree_kinematics`
Library for kinematic calculations for controlling the Go1.

## `unitree_nav`
Package for code and launch files for running the Unitree Go1 in high level mode with ROS 2 Humble. 

## `unitree_legged_real`
Holds the launch files for Zed 2i camera nodes, controlling, visualisation and exploration of Unitree Go1.

