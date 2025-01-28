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
  - [Communication framework](https://github.com/NU-MECH-ENG-495/project-Sayantani-Bhattacharya)
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
