#!/usr/bin/env python3
"""
AUTOWN'25 FINAL MISSION CONTROLLER (V9 - WITH CONTINUOUS AVOIDANCE)
- STARTUP: Exact copy of your working 'EnhancedMissionNode' logic.
- NAV: Queue-based logic + CONTINUOUS REPULSION FIELD.
- PERCEPTION: Lidar Clustering + Front Camera ArUco + Rviz Visualization.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from mavros_msgs.msg import State
from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import String, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster
import numpy as np
import math
import time
from enum import Enum
import json
import cv2
from cv_bridge import CvBridge
import sensor_msgs_py.point_cloud2 as pc2
from sklearn.cluster import DBSCAN

# ==========================================
#           MISSION CONFIGURATION
# ==========================================
TAKEOFF_ALT = 1.5        
CRUISE_SPEED = 0.4       
WAYPOINT_TOL = 0.3       
YAW_TOL = 0.1            

# Avoidance Parameters
AVOID_RADIUS = 0.9      # Distance to start avoiding
AVOID_STRENGTH = 1.5     # How hard to push away

# Relative Coordinates
RED_BEAM_LOC = [2.0, -1.0, TAKEOFF_ALT]   
GREEN_BEAM_LOC = [2.0, 1.0, TAKEOFF_ALT]  
YELLOW_PAD_LOC = [6.0, -1.0, 0.1]         

# MISSION QUEUE
MISSION_QUEUE = [
    {'type': 'GOTO', 'pos': [-3.0, -3.0, TAKEOFF_ALT], 'yaw': 0.0, 'desc': 'Approach Red Beam'},
    {'type': 'HOVER', 'dur': 10.0, 'desc': 'Hover at Red Hotspot'},
    {'type': 'GOTO', 'pos': [3.0, -3.0, TAKEOFF_ALT], 'yaw': 0.0, 'desc': 'Approach Green Beam'},
    {'type': 'HOVER', 'dur': 10.0, 'desc': 'Hover at Green Hotspot'},
    {'type': 'GOTO', 'pos': [4.0, 4.0, TAKEOFF_ALT], 'yaw': 0.0, 'desc': 'Approach Wall'},
    {'type': 'SCAN_WALL', 'dur': 5.0, 'desc': 'Scanning ArUco...'},
    {'type': 'GOTO', 'pos': [-5.0, 5.0, TAKEOFF_ALT], 'yaw': 0.0, 'desc': 'Overfly Wall'},
    {'type': 'GOTO', 'pos': [-5.0, 5.0, TAKEOFF_ALT], 'yaw': 0.0, 'desc': 'Align Yellow Pad'},
    {'type': 'LAND', 'pos': YELLOW_PAD_LOC, 'desc': 'Landing'}
]

class MissionState(Enum):
    START = 0
    CONNECTING = 1
    CHECKING_SERVICES = 2
    SETTING_MODE_GUIDED = 3
    WAITING_FOR_GUIDED = 4
    ARMING = 5
    WAITING_FOR_ARM = 6
    TAKEOFF_COMMAND = 7
    ASCENDING = 8
    EXECUTING_TASK = 9
    COMPLETE = 10
    ABORT = 99

# ==========================================
#          VISUALIZATION & PERCEPTION
# ==========================================
class PerceptionSystem:
    def __init__(self, node):
        self.marker_pub = node.create_publisher(MarkerArray, '/mission/obstacles', 10)
        self.tf_broadcaster = TransformBroadcaster(node)
        self.obstacles = []
        
    def process_lidar(self, msg, drone_pos):
        try:
            points = []
            min_z = drone_pos[2] - 0.5
            max_z = drone_pos[2] + 0.5

            for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                wx = float(p[0]) + drone_pos[0]
                wy = float(p[1]) + drone_pos[1]
                wz = float(p[2]) + drone_pos[2]
                
                if wz < min_z or wz > max_z: continue
                if (p[0]**2 + p[1]**2) < 0.2: continue # Ignore self
                points.append([wx, wy, wz])

            if len(points) < 10: 
                self.obstacles = []
                return

            pts_np = np.array(points)
            clustering = DBSCAN(eps=0.8, min_samples=5).fit(pts_np)
            
            self.obstacles = []
            ma = MarkerArray()
            d = Marker(); d.action = Marker.DELETEALL; d.header.frame_id="map"; ma.markers.append(d)

            for i, lbl in enumerate(set(clustering.labels_)):
                if lbl == -1: continue
                cluster = pts_np[clustering.labels_ == lbl]
                
                center = np.mean(cluster, axis=0)
                min_b = np.min(cluster, axis=0)
                max_b = np.max(cluster, axis=0)
                dim = max_b - min_b
                
                self.obstacles.append({'center': center, 'dim': dim})
                
                m = Marker()
                m.header.frame_id = "map"; m.ns = "obs"; m.id = i
                m.type = Marker.CUBE; m.action = Marker.ADD
                m.pose.position.x = center[0]; m.pose.position.y = center[1]; m.pose.position.z = center[2]
                m.scale.x = max(0.2, dim[0]); m.scale.y = max(0.2, dim[1]); m.scale.z = max(0.2, dim[2])
                
                if dim[0] < 1.0 and dim[1] < 1.0:
                    m.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8) # Red Pillar
                else:
                    m.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.6) # Blue Wall
                ma.markers.append(m)
            
            self.marker_pub.publish(ma)
        except: pass

    # --- NEW: CALCULATE REPULSION VECTOR ---
    def get_repulsion_vector(self, drone_pos):
        """ Calculates a 'push' vector away from all nearby obstacles """
        force = np.array([0.0, 0.0])
        if not self.obstacles: return force
        
        pos_2d = np.array(drone_pos[:2])
        
        for obs in self.obstacles:
            obs_2d = obs['center'][:2]
            dist = np.linalg.norm(pos_2d - obs_2d)
            
            # If obstacle is within danger radius
            if dist < AVOID_RADIUS:
                # Vector FROM obstacle TO drone
                vec_away = pos_2d - obs_2d
                if np.linalg.norm(vec_away) > 0:
                    vec_away = vec_away / np.linalg.norm(vec_away)
                    # Strength increases as we get closer (1/dist)
                    strength = (AVOID_RADIUS - dist) * AVOID_STRENGTH
                    force += vec_away * strength
        return force

    def publish_aruco_tf(self, id, tvec, rvec, drone_pos):
        t = TransformStamped()
        t.header.stamp = rclpy.time.Time().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = f"aruco_{id}"
        t.transform.translation.x = drone_pos[0] + tvec[2] 
        t.transform.translation.y = drone_pos[1] - tvec[0]
        t.transform.translation.z = drone_pos[2] - tvec[1]
        t.transform.rotation.w = 1.0 
        self.tf_broadcaster.sendTransform(t)

# ==========================================
#           MAIN MISSION CONTROLLER
# ==========================================
class AutonomousDrone(Node):
    def __init__(self):
        super().__init__('autonomous_drone')
        self.get_logger().info("🚀 AUTOWN V9 CONTROLLER STARTED")
        
        # QoS
        qos_rel = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_best = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
        
        # Subs
        self.state_sub = self.create_subscription(State, 'mavros/state', self.cb_state, qos_rel)
        self.pose_sub = self.create_subscription(PoseStamped, 'mavros/local_position/pose', self.cb_pose, qos_best)
        self.lidar_sub = self.create_subscription(PointCloud2, '/unilidar/cloud', self.cb_lidar, qos_best)
        self.cam_sub = self.create_subscription(Image, '/cam/image_raw', self.cb_camera, qos_best)
        
        # Pubs
        self.local_pos_pub = self.create_publisher(PoseStamped, 'mavros/setpoint_position/local', 10)
        
        # Clients
        self.cli_arm = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.cli_mode = self.create_client(SetMode, 'mavros/set_mode')
        self.cli_takeoff = self.create_client(CommandTOL, 'mavros/cmd/takeoff')
        self.cli_land = self.create_client(CommandTOL, 'mavros/cmd/land')
        
        # State
        self.state = MissionState.START
        self.drone_state = State()
        self.pose = np.array([0.0, 0.0, 0.0])
        self.current_yaw = 0.0
        
        # Modules
        self.perception = PerceptionSystem(self)
        self.cv_bridge = CvBridge()
        
        # Logic
        self.start_time = 0.0
        self.takeoff_requested = False
        self.task_idx = 0
        self.task_start_time = 0.0
        self.hovering = False
        self.detected_markers = []
        self.lidar_skip = 0
        self.initial_alt = 0.0
        self.alt_set = False
        self.current_setpoint = np.array([0.0, 0.0, 0.0])

        # Loop
        self.create_timer(0.05, self.control_loop) # 20Hz

    # --- SENSORS ---
    def cb_state(self, msg): self.drone_state = msg
    
    def cb_pose(self, msg):
        self.pose = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        # Yaw
        q = msg.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

        if not self.alt_set and abs(msg.pose.position.z) < 0.5:
            self.initial_alt = msg.pose.position.z
            self.alt_set = True

    def cb_lidar(self, msg):
        if self.state != MissionState.EXECUTING_TASK: return
        self.lidar_skip += 1
        if self.lidar_skip % 5 != 0: return
        self.perception.process_lidar(msg, self.pose)

    def cb_camera(self, msg):
        if self.lidar_skip % 10 != 0: return
        try:
            img = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            dct = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            corners, ids, _ = cv2.aruco.detectMarkers(img, dct)
            if ids is not None:
                for i, id in enumerate(ids):
                    mid = int(id[0])
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.2, np.array([[300,0,160],[0,300,120],[0,0,1]], dtype=float), np.zeros(5))
                    self.perception.publish_aruco_tf(mid, tvec[0][0], rvec[0][0], self.pose)
                    if mid not in self.detected_markers:
                        self.get_logger().info(f"📸 ARUCO DETECTED: {mid}")
                        self.detected_markers.append(mid)
        except: pass

    # --- CONTROL LOOP ---
    def control_loop(self):
        if not self.drone_state.connected: return
        now = self.get_clock().now().nanoseconds / 1e9
        dt = now - self.start_time

        # 1. STARTUP SEQUENCE
        if self.state == MissionState.START:
            if self.alt_set:
                self.get_logger().info("Connecting...")
                self.state = MissionState.CONNECTING
                self.start_time = now

        elif self.state == MissionState.CONNECTING:
            if self.cli_arm.service_is_ready():
                self.state = MissionState.SETTING_MODE_GUIDED

        elif self.state == MissionState.SETTING_MODE_GUIDED:
            if self.drone_state.mode != "GUIDED":
                self.set_mode_guided()
            else:
                self.get_logger().info("GUIDED Mode Set")
                self.state = MissionState.ARMING

        elif self.state == MissionState.ARMING:
            if not self.drone_state.armed:
                self.arm_drone()
            else:
                self.get_logger().info("Armed")
                self.state = MissionState.TAKEOFF_COMMAND

        elif self.state == MissionState.TAKEOFF_COMMAND:
            if not self.takeoff_requested:
                self.get_logger().info(f"Taking Off to {TAKEOFF_ALT}m...")
                self.takeoff_drone()
                self.takeoff_requested = True
            else:
                curr_alt = self.pose[2] - self.initial_alt
                if curr_alt >= TAKEOFF_ALT - 0.3:
                    self.get_logger().info("Takeoff Complete. Starting Mission.")
                    self.state = MissionState.ASCENDING
                    self.current_setpoint = np.array([self.pose[0], self.pose[1], TAKEOFF_ALT])

        elif self.state == MissionState.ASCENDING:
            self.send_pos(self.current_setpoint[0], self.current_setpoint[1], TAKEOFF_ALT, 0.0)
            self.state = MissionState.EXECUTING_TASK
            self.task_idx = 0
            self.hovering = False

        # 2. MISSION EXECUTION
        elif self.state == MissionState.EXECUTING_TASK:
            if self.task_idx >= len(MISSION_QUEUE):
                self.state = MissionState.COMPLETE
                return

            task = MISSION_QUEUE[self.task_idx]
            
            # TASK: LAND
            if task['type'] == 'LAND':
                self.get_logger().info(f"Executing: {task['desc']}")
                self.cli_land.call_async(CommandTOL.Request(latitude=0.0, longitude=0.0, altitude=0.0))
                self.state = MissionState.COMPLETE
                return

            # TASK: HOVER / SCAN
            if task['type'] in ['HOVER', 'SCAN_WALL']:
                if not self.hovering:
                    self.get_logger().info(f"Starting: {task['desc']}")
                    self.hovering = True
                    self.task_start_time = now
                
                self.send_pos(self.current_setpoint[0], self.current_setpoint[1], self.current_setpoint[2], 0.0)
                
                if (now - self.task_start_time) > task.get('dur', 5.0):
                    self.get_logger().info("Hover Done.")
                    self.hovering = False
                    self.task_idx += 1
                return

            # TASK: GOTO (WITH OBSTACLE AVOIDANCE)
            if task['type'] == 'GOTO':
                target = np.array(task['pos'])
                yaw_target = task.get('yaw', 0.0)
                
                # A. INTERPOLATION (Smooth Move)
                vec = target - self.current_setpoint
                dist_sp = np.linalg.norm(vec)
                
                step = 0.05
                if dist_sp > step:
                    move_vec = (vec / dist_sp) * step
                else:
                    move_vec = vec # Snap to target
                
                # B. REPULSION (Obstacle Avoidance)
                # This calculates a vector pointing AWAY from nearby obstacles
                repulsion = self.perception.get_repulsion_vector(self.pose)
                
                # C. COMBINE
                # Add repulsion to the movement vector to "bend" the path
                final_move = move_vec
                final_move[:2] += repulsion * 0.2 # Apply 20% influence of repulsion
                
                # Update current setpoint
                self.current_setpoint += final_move
                
                # Publish
                self.send_pos(self.current_setpoint[0], self.current_setpoint[1], self.current_setpoint[2], yaw_target)

                # D. CHECK ARRIVAL
                # Check actual drone position, not setpoint
                dist_drone = np.linalg.norm(self.pose[:2] - target[:2])
                z_err = abs(self.pose[2] - target[2])
                
                if dist_drone < WAYPOINT_TOL and z_err < 0.3:
                    self.get_logger().info(f"✅ Reached: {task['desc']}")
                    self.task_idx += 1

        elif self.state == MissionState.COMPLETE:
            pass 

    # --- HELPERS ---
    def send_pos(self, x, y, z, yaw):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)
        msg.pose.orientation.w = cy; msg.pose.orientation.z = sy
        self.local_pos_pub.publish(msg)

    def set_mode_guided(self):
        req = SetMode.Request(custom_mode="GUIDED")
        self.cli_mode.call_async(req)

    def arm_drone(self):
        req = CommandBool.Request(value=True)
        self.cli_arm.call_async(req)

    def takeoff_drone(self):
        req = CommandTOL.Request(altitude=float(TAKEOFF_ALT))
        self.cli_takeoff.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousDrone()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()

