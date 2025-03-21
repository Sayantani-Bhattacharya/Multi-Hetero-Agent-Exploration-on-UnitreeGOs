# Autonomous Navigation package for Unitree GO2.
Author: Sayantani Bhattacharya

## Procedure to run:

Current(will try to reduce the steps):

### Terminal 0 [Prepping Environment]:
    ./setup.sh  # In the unitree_ros submodule after connecting to GO2, as mentioned in its ReadMe.
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash

### Terminal 1:  [unitree_go2_nav]  -> run in [main_pkg]
#### As a sanity check delete the previous map (rm ~/.ros/rtabmap.db)
ros2 launch unitree_go2_nav navigation.launch.py
The base launch file for transforms, robot-state, slam and manual navigation.
      
      ros2 launch unitree_go2_nav navigation.launch.py


### Terminal 2: [main_pkg]
This subscripes to map data by rtabmap pkg, and unitree topics, And solves for exploration goals.

<!-- Terminal 3: [unitree_go2_nav]
ros2 run unitree_go2_nav navToPose
This subscribes to nav pkg, and publishes to the high level controller of GO2. -->

###  Terminal 4: [unitree_ros]
./install/unitree_ros2_example/bin/high_level_ctrl 
This publishes to Unitree APIs, final abstaction layer.

    ./install/unitree_ros2_example/bin/high_level_ctrl 


### Terminal 5: 
Trigger for GO2 to start exploring.

    ros2 service call /explore_start go2_exploration_interfaces/srv/Empty


## Troubleshooting
1. When building unitree_go2_nav for the first time, or after colcon clean,always follow:
       
       colcon build --packages-select cyclonedds
       source /opt/ros/jazzy/setup.bash  # [dont have this in bashrc if you are following the instruction of unitree_ros2, and using cyclonedds as middleware]
       colcon build
       source install/setup.bash
