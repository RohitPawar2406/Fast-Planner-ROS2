
# ROS2 porting of Fast-Planner

This repository is ROS2 (**Foxy**) ported Fast-planner codebase from the official ROS1 Fast-Planner codebase as here - [link](https://github.com/HKUST-Aerial-Robotics/Fast-Planner)

## Demo of ROS2 ported Kinodynamic Path Searching & B-spline Optimization

<!-- add some gif here -->
 <p id="demo1" align="center">
  <img src="files/demo.gif" width = "480" height = "270"/>
 </p>

## Setup (Tested on below setup)
- ROS2 - FOXY
- ROS1 - NOETIC
- Ubuntu - 20.04 (System and Intel NUC)

## Host build

```bash
mkdir ros2_ws
cd ros2_ws
mkdir src
cd src
git clone https://github.com/RohitPawar2406/Fast-Planner-ROS2
cd ..
colcon build
```

**Note :**  UAV simulator ros nodes (as in official ROS1 repository) are not yet ported in ROS2, so we are using the rosbridge setup for it. Detailed instructions to run the setup is given as below : 

## Setup & Run ROS1 (uav_simulator code)

- Clone the ros1 package as [here]() and build it using catkin_make.
- Run the uav simulator ros node, as given below. 
```bash
source devel/setup.bash
roslaunch so3_control kino_replan.launch
```

## Setup & Run ROS1 bridge 

- Clone and build the ros1 bridge package as given in detail [here](https://github.com/ros2/ros1_bridge).
- Run the ros1 bridge node as given below.

```bash
# source noetic
source /opt/ros/noetic/setup.bash

# source foxy
source /opt/ros/foxy/setup.bash

# Then source ros1 fast planner workspace - uav simulator code in above step 
# eg(ours) : source ~/workspace/iiit-r/half_fastplanner/devel/setup.bash

# Then source ros2 fast planner workspace
# eg(ours) : source ~/workspace/iiit-r/fastplanner_ws2/install/setup.bash

source install/setup.bash
ros2 run ros1_bridge dynamic_bridge
```

## Run the Kino Replan Launch File 

```bash
cd ros2_ws # ros2 workspace build above
source install/setup.bash
ros2 launch  plan_manage  kino_replan_launch.xml
```

## Run the RViz

```bash
cd ros2_ws # ros2 workspace build above
source install/setup.bash
ros2 launch plan_manage rviz.launch.xml
```

Note : For rviz you may need to manually load the rviz file from visualizer 
- File > Open config > Select .rviz file path (Fast-Planner-ROS2 > src > planner > plan_manage > rviz2_fastplanner.rviz)


## TODO : 

- [] Port Topo replan to the ros2
- [] Port the uav simulator ros package to ros2 and remove dependency of ros2 bridge
- [] **Bug Fix :** Drone Yaw is not updating after reaching the goal position

Reach out for any further queries or discussion at : rohit.pawar@research.iiit.ac.in
