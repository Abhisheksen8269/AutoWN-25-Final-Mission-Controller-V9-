# AutoWN-25-Final-Mission-Controller-V9-
AutoWN '25 Final Mission Controller (V9)

This repository contains the autonomous flight logic for the AutoWN '25 competition. The system utilizes ROS2 Humble, MAVROS, and Lidar-based obstacle avoidance to navigate a complex environment with continuous repulsion fields.

##🚀 The Approach: Step-by-Step

##Step 1: State Machine Architecture
To ensure reliability, the drone operates on a robust finite state machine (FSM). It sequences through connection, arming, takeoff, and task execution. This ensures the drone never attempts a mission task before the hardware is fully ready.

##Step 2: Perception & Lidar ClusteringThe drone "sees" the world through a 3D Lidar.Filtering: We filter points relative to the drone's altitude to ignore the floor and ceiling.Clustering: Using the DBSCAN algorithm, raw points are grouped into distinct obstacle objects (Pillars vs. Walls).Visualization: Obstacles are published as MarkerArrays to Rviz for real-time monitoring.

##Step 3: Continuous Repulsion Field (Avoidance)Instead of a simple "stop and turn" logic, V9 uses a Potential Field approach.Attractive Force: The Mission Queue pulls the drone toward the next waypoint.Repulsive Force: Every detected obstacle within the AVOID_RADIUS exerts a "push" vector away from itself.The Math: The repulsion strength is calculated as:$$F_{repulsion} = \text{max}(0, \text{AvoidRadius} - \text{dist}) \times \text{AvoidStrength}$$This force is blended with the movement vector to "bend" the drone's path around obstacles smoothly.

##Step 4: ArUco Vision IntegrationUsing the front camera and OpenCV, the drone scans for ArUco markers on walls. When a marker is detected, the PerceptionSystem calculates its 3D position in the map frame and broadcasts a TF (Transform), allowing the drone to "remember" where the target was located.Step 5: Smooth InterpolationTo prevent jerky movements, the controller does not send the final waypoint immediately. It interpolates the setpoint, moving it in 5cm increments every 50ms, resulting in fluid, organic flight.


This is a sophisticated ROS2/MAVROS mission controller. To host this effectively on GitHub, you need a structured repository that explains the Autonomous Navigation, Obstacle Avoidance (Potential Fields), and Perception logic.

Here is the blueprint for your GitHub repository.

##1. Repository Structure
I recommend organizing your files as follows:


##autown-mission-control/
├── README.md
├── scripts/
│   └── mission_controller.py      <-- (Your V9 Code)
├── config/
│   └── params.yaml                <-- (Extracted constants)
├── assets/
│   └── repulsion_diagram.png      <-- (The logic image)
└── package.xml
└── setup.py

🚀 The Approach: Step-by-Step
Step 1: State Machine Architecture
To ensure reliability, the drone operates on a robust finite state machine (FSM). It sequences through connection, arming, takeoff, and task execution. This ensures the drone never attempts a mission task before the hardware is fully ready.

Step 2: Perception & Lidar Clustering
The drone "sees" the world through a 3D Lidar.

Filtering: We filter points relative to the drone's altitude to ignore the floor and ceiling.

Clustering: Using the DBSCAN algorithm, raw points are grouped into distinct obstacle objects (Pillars vs. Walls).

Visualization: Obstacles are published as MarkerArrays to Rviz for real-time monitoring.

Step 3: Continuous Repulsion Field (Avoidance)
Instead of a simple "stop and turn" logic, V9 uses a Potential Field approach.

Attractive Force: The Mission Queue pulls the drone toward the next waypoint.

Repulsive Force: Every detected obstacle within the AVOID_RADIUS exerts a "push" vector away from itself.

The Math: The repulsion strength is calculated as:

F 
repulsion
​
 =max(0,AvoidRadius−dist)×AvoidStrength

This force is blended with the movement vector to "bend" the drone's path around obstacles smoothly.

Step 4: ArUco Vision Integration
Using the front camera and OpenCV, the drone scans for ArUco markers on walls. When a marker is detected, the PerceptionSystem calculates its 3D position in the map frame and broadcasts a TF (Transform), allowing the drone to "remember" where the target was located.

Step 5: Smooth Interpolation
To prevent jerky movements, the controller does not send the final waypoint immediately. It interpolates the setpoint, moving it in 5cm increments every 50ms, resulting in fluid, organic flight.

🛠 Installation & Usage
Clone the repo:

Bash
git clone https://github.com/yourusername/autown-mission-control.git
Install Dependencies:

Bash
pip3 install scikit-learn opencv-python numpy
Run the Node:

Bash
python3 scripts/mission_controller.py
3. Creating the "Image" for GitHub
To make your GitHub look professional, you need a visual representation of your logic. Since I can't upload a file directly to your computer, here is how you should create it:

Open a tool like Canva or Excalidraw.

Draw:

A blue dot (Drone).

A green star (Goal).

A red circle (Obstacle).

Add Arrows:

A straight dashed line from Drone to Goal (Target Path).

A thick arrow pushing the Drone away from the Red Circle.

A curved blue line showing the drone's actual path "warped" by the obstacle.

Save as: assets/repulsion_diagram.png.

