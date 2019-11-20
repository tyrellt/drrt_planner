# drrt_planner

## Steps to create workspace:
#### 1. Install ROS Melodic: http://wiki.ros.org/melodic/Installation/Ubuntu
#### 2. Create a catkin workspace: http://wiki.ros.org/catkin/Tutorials/create_a_workspace
* The rest of the commands assume you created your catkin workspace in your ~/ (home) directory and called it catkin_ws.
#### 3. Install necessary dependent packages for turtlebot3
```
sudo apt-get update
```
```
sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan ros-melodic-rosserial-arduino ros-melodic-rosserial-python ros-melodic-rosserial-server ros-melodic-rosserial-client ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro ros-melodic-compressed-image-transport ros-melodic-rqt-image-view ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers
```
- I don't know if all those packages are necessary. If I had my guess, you probably only need these ones:
```
ros-melodic-teleop-twist-keyboard ros-melodic-map-server ros-melodic-move-base ros-melodic-rqt-image-view ros-melodic-navigation
```
But to be safe, I would just install them all.
```
cd ~/catkin_ws/src/
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/catkin_ws
catkin_make
```

#### 4. Test your environment
Run through this tutorial. We already cloned some of the stuff it asks you to do. You don't have to do everything--just make sure you can run gazebo and run the teleop stuff to make sure you can control the turtlebot so you knonw everything is talking.
http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#simulation

#### 5. Clone this repository
```
cd ~/catkin_ws/src
git clone https://github.com/tyrellt/drrt_planner.git
cd ~/catkin_ws
catkin_make
```

#### 6. Set global planner for move_base
Add the following line to move_base.launch (catkin_ws/src/turtlebot3/turtlebot3_navigation/launch/move_base.launch) somewhere within the node tag:
```
<param name="base_global_planner" value="drrt_planner/DRRTPlanner"/>
```
This will set the global planner to be our planner.

