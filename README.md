# AutoWN-25-Final-Mission-Controller-V9-
AutoWN '25 Final Mission Controller (V9)

This repository contains the autonomous flight logic for the AutoWN '25 competition. The system utilizes ROS2 Humble, MAVROS, and Lidar-based obstacle avoidance to navigate a complex environment with continuous repulsion fields.

## 🚀 The Approach: Step-by-Step

## Step 1: State Machine Architecture
To ensure reliability, the drone operates on a robust finite state machine (FSM). It sequences through connection, arming, takeoff, and task execution. This ensures the drone never attempts a mission task before the hardware is fully ready.

## Step 2: Perception & Lidar Clustering
The drone "sees" the world through a 3D Lidar.Filtering: We filter points relative to the drone's altitude to ignore the floor and ceiling.Clustering: Using the DBSCAN algorithm, raw points are grouped into distinct obstacle objects (Pillars vs. Walls).Visualization: Obstacles are published as MarkerArrays to Rviz for real-time monitoring.

## Step 3: Continuous Repulsion Field (Avoidance) 
Instead of a simple "stop and turn" logic, V9 uses a Potential Field approach.Attractive Force: The Mission Queue pulls the drone toward the next waypoint.Repulsive Force: Every detected obstacle within the AVOID_RADIUS exerts a "push" vector away from itself.The Math: The repulsion strength is calculated as:$$F_{repulsion} = \text{max}(0, \text{AvoidRadius} - \text{dist}) \times \text{AvoidStrength}$$This force is blended with the movement vector to "bend" the drone's path around obstacles smoothly.

## Step 4: ArUco Vision 
IntegrationUsing the front camera and OpenCV, the drone scans for ArUco markers on walls. When a marker is detected, the PerceptionSystem calculates its 3D position in the map frame and broadcasts a TF (Transform), allowing the drone to "remember" where the target was located.Step 5: Smooth InterpolationTo prevent jerky movements, the controller does not send the final waypoint immediately. It interpolates the setpoint, moving it in 5cm increments every 50ms, resulting in fluid, organic flight.


This is a sophisticated ROS2/MAVROS mission controller. To host this effectively on GitHub, you need a structured repository that explains the Autonomous Navigation, Obstacle Avoidance (Potential Fields), and Perception logic.

Here is the blueprint for your GitHub repository.

**Mission:** Autonomous Drone Navigation with Continuous Obstacle Avoidance  
**Platform:** ROS 2 Humble / Gazebo / PX4 / MAVROS

![Gazebo Simulation Environment](assets/gazebo_env.png)
*(Place a screenshot of your drone in the Gazebo arena here)*

## 📝 Project Description
This repository contains the **V9 Mission Controller** for the AutoWN'25 competition. The system is designed to autonomously navigate a quadcopter through a complex obstacle course, scan for ArUco markers on vertical walls, and perform precision landings. 

Unlike simple "stop-and-go" logic, this controller implements a **Continuous Repulsion Field** (Potential Field) algorithm, allowing the drone to smoothly curve around obstacles while maintaining forward momentum toward its goal.

### ⚙️ Sensor Configuration
* **3D LiDAR:** Used for spatial awareness and obstacle detection (Point Cloud clustering).
* **Front-Facing Camera:** Used for ArUco marker detection (ID & Pose estimation).
* **Flight Controller:** Pixhawk/PX4 (SITL in Gazebo).

---

## 📐 Mathematical Approach & Algorithms

### 1. Perception: Lidar Clustering (DBSCAN)
Raw point cloud data is noisy. We process it in three stages:
1.  **ROI Filtering:** We clip points based on height ($z$) to remove the floor and ceiling, focusing only on obstacles at flight altitude.
2.  **Self-Filtering:** Points within a 0.2m radius of the drone center are discarded to prevent self-collision detection.
3.  **Clustering:** We use **DBSCAN (Density-Based Spatial Clustering of Applications with Noise)** to group remaining points into objects.
    * *Epsilon ($\epsilon$):* 0.8m (Max distance between points in a cluster)
    * *Min Samples:* 5 points

![Rviz Visualization](assets/rviz_clustering.png)
*(Place a screenshot of Rviz showing the Red/Blue marker obstacles here)*

### 2. Navigation: Potential Field Repulsion
To avoid obstacles dynamically, we calculate a **Repulsion Vector** that "pushes" the drone away from detected clusters.

Let $P_{drone}$ be the drone's position and $O_i$ be the center of the $i$-th obstacle. The distance $d_i$ is:
$$d_i = || P_{drone} - O_i ||$$

If an obstacle is within the **Avoidance Radius ($R_{avoid} = 0.9m$)**, a repulsion force $\vec{F}_{rep}$ is generated:

$$\vec{F}_{rep} = \sum_{i} \left( \frac{P_{drone} - O_i}{d_i} \right) \cdot (R_{avoid} - d_i) \cdot k_{strength}$$

Where:
* The first term is the **Unit Vector** pointing away from the obstacle.
* The second term scales the force linearly (stronger as you get closer).
* $k_{strength} = 1.5$ is the gain factor.

This force is added to the navigation vector, bending the drone's trajectory smoothly around the hazard.

### 3. Visual Servoing: ArUco Detection
We use the `opencv-contrib-python` library to detect ArUco markers.
* **Pose Estimation:** We solve the **PnP (Perspective-n-Point)** problem using `cv2.aruco.estimatePoseSingleMarkers` to find the marker's translation vector ($\vec{t}$) relative to the camera.
* **Coordinate Transform:** The marker position is transformed from the *Camera Optical Frame* to the *Map Frame* using the drone's current odometry.

---

## 🚀 How-to-Run (Competition Guide)

### 1. System Requirements
* **OS:** Ubuntu 22.04 LTS (Jammy Jellyfish)
* **ROS Version:** ROS 2 Humble Hawksbill
* **Simulator:** Gazebo Garden / Classic
* **Middleware:** PX4-Autopilot SITL

### 2. Build Instructions
Navigate to your colcon workspace root and build the package:

```bash
cd ~/ros2_ws
colcon build --packages-select autown_mission
source install/setup.bash
