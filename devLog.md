# MultiHeteroAgentExploration DevLog

Author: Sayantani Bhattacharya </br>
</br>
This DevLog documents the progress of the project, to keep track for me and as a reference to anyone else who wishes to work on something similar.

## Week5:

#### (a) Accomplishments
  1. Self calibrated and resolved the depth image issue in ZED.
  2. Successfully had a test run Autonomous Navigation for GO1.
  3. Point cloud data for both GO2 4D Lidar, and Zed stereo camera can be visualized in rviz.

#### (b) Preliminary Results: 
  1. ![GO1 Navigation]([https://github.com/Sayantani-Bhattacharya/Multi-Hetero-Agent-Exploration-on-UnitreeGOs/blob/main/images/go1_high_lvl_results.mp4](https://github.com/Sayantani-Bhattacharya/Multi-Hetero-Agent-Exploration-on-UnitreeGOs/blob/main/images/GO1_Nav_first_run.mp4))
    
#### (c) Unresolved Problems:
  1. Point cloud to occupancy-grid conversion for Nav2 for GO1.

#### (d) Plans: 
  1. Develop a way to generate occupancy-grids from point clouds.
  2. Create/Understand a pipeline for saving maps after frontier based exploration (go1).
  3. Write the navigation for go2.
  4. Start developing the Distributed network’s MVP.
  5. Develop a way to convert 4d lidar data to generate maps/grid.
  6. Get GO1 and GO2  to perform a basic Navigation and map creation from slam task.


## Week4:

#### (a) Accomplishments
  1. Tested High level control for go1 with humble in jetson. [unitree_nav pkg]
  2. Tested High level control for go2 with humble docker in laptop. [unitree-ros2-sdk]

#### (b) Preliminary Results: 
  1. ![GO1 high level control](https://github.com/Sayantani-Bhattacharya/Multi-Hetero-Agent-Exploration-on-UnitreeGOs/blob/main/images/go1_high_lvl_results.mp4)
  2. ![GO2 high level control](https://github.com/Sayantani-Bhattacharya/Multi-Hetero-Agent-Exploration-on-UnitreeGOs/blob/main/images/go2_High_lvl_results.mp4)

#### (c) Unresolved Problems:
  1. Zed-camera self-calibration is failing.

#### (d) Plans: 
  1. Fix the ZED camera.
  2. Get GO1 and GO2  to perform a basic Navigation and map creation from slam task.
  
## Week3:

#### (a) Accomplishments
  1. Jetson setup with ubuntu and all the binaries done and tested.
     Suggestion: After an initial trial, install libraries and ROS2 Humble directly from source, easier way in for Jetson.
  3. Got the GO1 to work, powered by jetson in the high level mode, using wifi.
  4. Read and saw tutorials for network architecture, to develop a peer-to-peer network.

#### (b) Unresolved Problems:
  1. The Zed camera Diagnostic tool is failing in jetson. Which most likely is a GSLM driver compatibility problem.

#### (c) Plans: 
  1. Fix the camera-jetson compatibility issue.
  2. Get GO1 to perform a basic Navigation and map creation from slam task.
  3. Get familiarized with the unitree_ros2 sdk for GO2, and to get the system to work with ethernet/WiFi.
  4. Start developing the peer-to-peer network.


## Week2:

#### (a) Accomplishments
  1. Rviz simulation setup of go2 is working (ported from official ROS1 to ROS2).
  2. Got the merging map repository to work in my system.
  3. Gazebo setup for go1 and go2 working.
  4. 3D printed the Zed cam - Jetson - to go1 mount (modelling by [David Dorf](https://www.daviddorf.com/home)).
  5. Soldered the buck converter, jetson power and unitree (GO1) power ports: so the hardware setup for GO1 should be complete with this.

#### (b) Unresolved Problems:
  1. The Jetson I was using had the entire file system corrupted. Wasn't able to fix the issue, and am in the process of reconfiguring it. Waiting for the display port as well.
  2. Setting up Ethernet connection with go2 and my setup on my laptop.

#### (c) Plans: 
  1. Configure the Jetson.
  2. Get GO1 to perform a basic Navigation and map creation from slam task.
  3. Develop a simulation_navigation pkg to perform slam map creation and basic nav task, for both go1 and go2.
  4. Familiarize with the unitree_ros2 sdk for go2, and to get the system to work with ethernet.
  5. Start working on SLAM and Navigation nodes in the main repo.
  6. Get the simulation setup for GO2 done.

## Week1:

#### (a) Accomplishments
  1. Simulation setup of go1 is working.
  2. Simulation setup for go2 is ported to ROS2.
  3. Read and sorted a few research papers for map merging and collaborative SLAM. Trying to get a repository working for the same.
  4. Did basic movements in GO2 (via its App) to understand its operation.
  5. Developed an overview of the existing ROS packages for slam and navigation.

#### (b) Unresolved Problems:
  1. Simulation setup for go2 has some errors to be resolved.
  2. The map merging repository has some errors to be resolved.

#### (c) Plans: 
  1. Get the simulation setup for GO2 done.
  2. Complete GO2 hardware setup with Jetson Nano, Zed camera.
  3. Get it to perform a basic Navigation and map creation from slam task.
  4. Get the map merging repository to work.
  5. Start working on SLAM and Navigation nodes in the main repo.
