# Multi Hetero Agent Exploration by Unitree GOs 

Author: Sayantani Bhattacharya

## Project Overview

To build a swarm of quadrupeds, capable of collaborative exploration missions, in hazardous/dangerous terrains.
Quadrupeds are ideal for moving in uneven areas like mines, forests, etc and thus chosen for the task. The aim is to 
build a system that performs localization, mapping, navigation and attention detection (can be a human/hazard prone area, 
depending on the task). Each quadruped would have these capabilities, and would also have the ability to merge the 
information from the agents in its vicinity (decentralized), to make a more robust system, and explore an area at the 
fastest possible speed. Agents would be aware of each-other’s presence and work in collaboration with each other.

## Block diagram

1. Complete system:</br>
<p align="right">
  <img src="/images/system_block.png" alt="Alt text" width="700"/>
</p>
 

2. Individual cluster:</br>
    (subject to modifications) </br>
    <p align="right">
     <img src="/images/indv_block.png" alt="Alt text" width="700"/>
    </p>

## Tools and References:

### Hardware:
  - Unitree GO1-GO2.
  - Zed Camera/ Lidar (Need to confirm).
  - Jetson Orin Nano.
  - buck-convertor (24V->12V).
  - 3D print the mount for unitree.
  - Display port adapters for Jetson.
  - Ethernet cable for initial testing with unitree sdk.
  - Micro SD cards.

### Software : 
  - C++
  - ROS2
  - Python
  - Unitree SDK - GO1 and GO2
  - Slam, RTabMap, gmapping and Nav2 pkg

### Reference repositories:
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

