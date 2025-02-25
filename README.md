# My 10 ROS Projects for Beginners

## Introduction
Welcome to my collection of ROS (Robot Operating System) projects for beginners! Over the course of my exploration into robotics and ROS, I developed these 10 projects to deepen my understanding of various robotics concepts, from basic communication nodes to autonomous drone navigation. Each project is designed to be hands-on, educational, and beginner-friendly, with clear instructions and expected outcomes.

## Table of Contents
1. [TurtleBot3 Simulation](#turtlebot3-simulation)
2. [Simple Publisher and Subscriber](#simple-publisher-and-subscriber)
3. [Robot Arm Control](#robot-arm-control)
4. [Obstacle Avoidance Robot](#obstacle-avoidance-robot)
5. [Line Following Robot](#line-following-robot)
6. [SLAM with ROS](#slam-with-ros)
7. [Voice Controlled Robot](#voice-controlled-robot)
8. [Robot Navigation using LiDAR](#robot-navigation-using-lidar)
9. [Image Processing with ROS](#image-processing-with-ros)
10. [Autonomous Drone Navigation](#autonomous-drone-navigation)

## Prerequisites
To work with these projects, you’ll need:
- Ubuntu 20.04 or later
- ROS Noetic or ROS 2 Foxy ([ROS installation guide](http://wiki.ros.org/ROS/Installation))
- Python 3.8+
- Familiarity with Linux commands and basic ROS concepts

## Installation
1. Clone this repository:
   ```bash
   git clone https://github.com/yourusername/ros-beginner-projects.git
   cd ros-beginner-projects
   ```
2. Build the workspace:
   ```bash
   catkin_make  # For ROS 1
   colcon build # For ROS 2
   ```
3. Source the workspace:
   ```bash
   source devel/setup.bash  # For ROS 1
   source install/setup.bash # For ROS 2
   ```

## Projects

### TurtleBot3 Simulation
**Objective:** Develop a simulation environment for TurtleBot3 in Gazebo to understand robot kinematics and control.

**Setup & Usage:**
```bash
sudo apt install ros-noetic-turtlebot3-simulations
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
Control the robot using:
```bash
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
**Outcome:** Successfully navigated a simulated environment using keyboard commands.

---

### Simple Publisher and Subscriber
**Objective:** Learn the basics of ROS communication through publisher and subscriber nodes.

**Setup & Usage:**
```bash
cd ~/catkin_ws/src
catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
cd ~/catkin_ws
catkin_make
rosrun beginner_tutorials talker.py
rosrun beginner_tutorials listener.py
```
**Outcome:** Demonstrated effective communication between nodes using ROS topics.

---

### Robot Arm Control
**Objective:** Develop motion planning and control for a robot arm using MoveIt.

**Setup & Usage:**
```bash
roslaunch moveit_setup_assistant setup_assistant.launch
```
**Outcome:** Successfully controlled a simulated robotic arm, executing planned trajectories in RViz.

---

### Obstacle Avoidance Robot
**Objective:** Implement real-time obstacle detection and avoidance using LiDAR and ultrasonic sensors.

**Usage:**
```bash
rosrun obstacle_avoidance avoid.py
```
**Outcome:** The robot navigated around obstacles autonomously in both simulated and real-world environments.

---

### Line Following Robot
**Objective:** Develop a vision-based system for line following using camera inputs.

**Usage:**
```bash
rosrun line_follower follow_line.py
```
**Outcome:** Robot successfully followed a designated path marked with a black line.

---

### SLAM with ROS
**Objective:** Explore simultaneous localization and mapping (SLAM) with a mobile robot.

**Usage:**
```bash
roslaunch turtlebot3_slam turtlebot3_slam.launch
```
**Outcome:** Generated a detailed map of the environment using LiDAR data and ROS SLAM packages.

---

### Voice Controlled Robot
**Objective:** Enable voice-based robot control for hands-free operation.

**Usage:**
```bash
rosrun voice_control voice_control.py
```
**Outcome:** Successfully executed voice commands for robot movement and actions.

---

### Robot Navigation using LiDAR
**Objective:** Achieve autonomous navigation using LiDAR-based mapping and path planning.

**Usage:**
```bash
roslaunch robot_navigation navigation.launch
```
**Outcome:** The robot autonomously reached set goals while avoiding obstacles.

---

### Image Processing with ROS
**Objective:** Process real-time images for object detection using OpenCV and ROS.

**Usage:**
```bash
rosrun image_processing process_image.py
```
**Outcome:** Detected and tracked objects in real-time video feeds.

---

### Autonomous Drone Navigation
**Objective:** Develop autonomous navigation capabilities for drones using ROS.

**Usage:**
```bash
roslaunch drone_simulation drone_world.launch
```
**Outcome:** Successfully navigated through waypoints autonomously in a simulated environment.

---

## Contributing
I welcome any suggestions or improvements. Feel free to fork this repository, submit pull requests, or report issues.

## License
This repository is open-sourced under the MIT License.

## Acknowledgments
Special thanks to the ROS community and [Learn Robotics and AI](https://www.learnroboticsandai.com) for the resources and inspiration that guided me through these projects.
