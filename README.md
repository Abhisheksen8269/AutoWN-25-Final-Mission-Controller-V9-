# AutoWN-25-Final-Mission-Controller-V9-
AutoWN '25 Final Mission Controller (V9)

## 📝 Project Description
This repository contains the **V9 Mission Controller** for the AutoWN'25 competition. The system is designed to autonomously navigate a quadcopter through a complex obstacle course, scan for ArUco markers on vertical walls, and perform precision landings. 

Unlike simple "stop-and-go" logic, this controller implements a **Continuous Repulsion Field** (Potential Field) algorithm, allowing the drone to smoothly curve around obstacles while maintaining forward momentum toward its goal

![Gazebo Simulation Environment]
<img width="1343" height="826" alt="image" src="https://github.com/user-attachments/assets/181b8814-b0ef-406f-acce-3a1e6a465dbe" />

## [Video Link]
https://drive.google.com/file/d/1fyjyz0FkORuh2HaiiOvOvaVd46t9hP-r/view?usp=sharing


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
<img width="1425" height="670" alt="image" src="https://github.com/user-attachments/assets/8412f488-b03d-43c8-9ed6-60951867d0f4" />


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

```
cd ~/ros2_ws
colcon build 
source install/setup.bash
```

### 2. Launch gazebo Simulation 

```
export GAZEBO_MODEL_PATH=$HOME/iq_sim/models:$GAZEBO_MODEL_PATH
gazebo --verbose $HOME/iq_sim/worlds/lidar.world
```


### 2. Launch SITL to connect with gazebo

```
 python3 Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map

```

### 2. Launch MAVROS With ROS2 humble

```
ros2 run mavros mavros_node --ros-args     -p fcu_url:=udp://127.0.0.1:14550@     -p use_sim_time:=true  

```

### 2. Run ROS2 node mission

```
ros2 run my_pakage mission 


```

### 2. Launch Rviz2

```
rviz2


```



