#!/usr/bin/env python3

import yaml
import ctypes
import os
import sys
import math
import threading
import time
import atexit
import argparse
import signal

# ROS1 imports
try:
    import rospy
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TransformStamped, Twist, Vector3
    from geometry_msgs.msg import Quaternion as ROSQuaternion
    from sensor_msgs.msg import LaserScan
    import tf2_ros
    import rospkg
    from rosgraph_msgs.msg import Clock
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('cohan_sim')  
    lib_path = os.path.join(pkg_path, 'lib', 'libsim.so') 
    config_path = os.path.join(pkg_path, 'config') 
    env_path = os.path.join(pkg_path, 'maps') 

    HAS_ROS1 = True
except ImportError:
    HAS_ROS1 = False

# ROS2 imports
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TransformStamped, Twist, Vector3
    from geometry_msgs.msg import Quaternion as ROSQuaternion
    from sensor_msgs.msg import LaserScan
    from tf2_ros import TransformBroadcaster
    from ament_index_python.packages import get_package_share_directory
    pkg_path = get_package_share_directory('cohan_sim') 
    lib_path = os.path.join(pkg_path, 'lib', 'libsim.so')
    config_path = os.path.join(pkg_path, 'config') 
    env_path = os.path.join(pkg_path, 'maps') 

    HAS_ROS2 = True
    # Initialize ROS2 once
    if not rclpy.ok():
        rclpy.init()
    ros2_executor = MultiThreadedExecutor(num_threads=4)
except ImportError:
    HAS_ROS2 = False

def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = ROSQuaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q

class ROSWrapper:
    def __init__(self, use_ros2=False, agent_name=None, agent_idx=0):
        self.use_ros2 = use_ros2
        self.node = None
        self.tf_broadcaster = None
        self.publishers = {}
        self.subscribers = {}
        self.running = True
        self.agent_name = agent_name
        self.agent_idx = agent_idx
        
        # Create topic prefix for non-first agents
        self.prefix = f"{agent_name}/" if agent_name else ""
        self.frame_prefix = f"{agent_name}/" if agent_name else ""

        if use_ros2:
            if not HAS_ROS2:
                print("ROS2 not available")
                return
            
            # Configure different QoS settings for different types of messages
            tf_qos = QoSProfile(
                depth=10,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST
            )
            
            sensor_qos = QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST
            )
            
            state_qos = QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST
            )
            
            node_name = f'cohan_sim_{agent_name}' if agent_name else 'cohan_sim'
            self.node = Node(node_name)
            
            # Create callback group
            self.callback_group = MutuallyExclusiveCallbackGroup()
            
            self.tf_broadcaster = TransformBroadcaster(self.node, qos=tf_qos)
            # Create publishers with appropriate QoS for each type
            self.publishers['odom'] = self.node.create_publisher(
                Odometry, f'{self.prefix}odom', state_qos)
            self.publishers['ground_truth'] = self.node.create_publisher(
                Odometry, f'{self.prefix}base_pose_ground_truth', state_qos)
            self.publishers['scan'] = self.node.create_publisher(
                LaserScan, f'{self.prefix}scan', sensor_qos)
            self.subscribers['cmd_vel'] = self.node.create_subscription(
                Twist, f'{self.prefix}cmd_vel', self.cmd_vel_callback, state_qos,
                callback_group=self.callback_group)
            self.subscribers['head_rotation'] = self.node.create_subscription(
                Vector3, f'{self.prefix}head_rotation', self.head_rotation_callback, state_qos,
                callback_group=self.callback_group)

            
            # Add node to executor
            ros2_executor.add_node(self.node)
        else:
            if not HAS_ROS1:
                print("ROS1 not available")
                return
            rospy.init_node('cohan_sim', anonymous=True)
            self.tf_broadcaster = tf2_ros.TransformBroadcaster()

            # Create publishers with increased queue sizes
            self.publishers['odom'] = rospy.Publisher(
                f'{self.prefix}odom', Odometry, queue_size=50)
            self.publishers['ground_truth'] = rospy.Publisher(
                f'{self.prefix}base_pose_ground_truth', Odometry, queue_size=50)
            self.publishers['scan'] = rospy.Publisher(
                f'{self.prefix}scan', LaserScan, queue_size=50)
            self.subscribers['cmd_vel'] = rospy.Subscriber(
                f'{self.prefix}cmd_vel', Twist, self.cmd_vel_callback)
            self.subscribers['head_rotation'] = rospy.Subscriber(
                f'{self.prefix}head_rotation', Vector3, self.head_rotation_callback)

    def cmd_vel_callback(self, msg):
        # Call the C function to set robot velocities
        lib.set_robot_velocity(self.agent_idx, msg.linear.x, msg.linear.y, msg.angular.z)

    def head_rotation_callback(self, msg):
        # Call the C function to set head rotation
        lib.set_head_rotation(self.agent_idx, msg.z)

    def publish_tf(self, x, y, theta, head_yaw, timestamp=None):
        if timestamp is None:
            timestamp = rospy.Time.now() if not self.use_ros2 else self.node.get_clock().now().to_msg()

        transforms = []
        # odom -> base_footprint
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = f"{self.frame_prefix}odom"
        t.child_frame_id = f"{self.frame_prefix}base_footprint"
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = 0.0
        q = quaternion_from_euler(0, 0, float(theta))
        t.transform.rotation = q
        transforms.append(t)

        # base_footprint -> base_link
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = f"{self.frame_prefix}base_footprint"
        t.child_frame_id = f"{self.frame_prefix}base_link"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        q = quaternion_from_euler(0, 0, 0)
        t.transform.rotation = q
        transforms.append(t)

        # base_link -> base_laser_link
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = f"{self.frame_prefix}base_link"
        t.child_frame_id = f"{self.frame_prefix}base_laser_link"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.1  # 10cm above base_link
        q = quaternion_from_euler(0, 0, 0)
        t.transform.rotation = q
        transforms.append(t)

        # base_link -> head_rotation_frame
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = f"{self.frame_prefix}base_link"
        t.child_frame_id = f"{self.frame_prefix}head_rotation_frame"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 1.0  # 1m above base_link (adjust as needed)
        q = quaternion_from_euler(0, 0, head_yaw)  # or (roll, pitch, yaw)
        t.transform.rotation = q
        transforms.append(t)
        
        # Send all transforms at once
        if self.use_ros2:
            self.tf_broadcaster.sendTransform(transforms)
        else:
            for transform in transforms:
                self.tf_broadcaster.sendTransform(transform)

    def publish_odom(self, x, y, theta, vx, vy, vth, timestamp=None):
        if timestamp is None:
            timestamp = rospy.Time.now() if not self.use_ros2 else self.node.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = timestamp
        odom.header.frame_id = f"{self.frame_prefix}odom"
        odom.child_frame_id = ""
        
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        
        q = quaternion_from_euler(0, 0, theta)
        odom.pose.pose.orientation = q
        
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth
        
        self.publishers['odom'].publish(odom)
        self.publishers['ground_truth'].publish(odom)  # Same as odom for simulation

    def publish_scan(self, ranges, max_range, angle_min, angle_max, angle_increment, timestamp=None):
        if timestamp is None:
            timestamp = rospy.Time.now() if not self.use_ros2 else self.node.get_clock().now().to_msg()

        scan = LaserScan()
        scan.header.stamp = timestamp
        scan.header.frame_id = f"{self.frame_prefix}base_laser_link"
        scan.angle_min = float(angle_min)
        scan.angle_max = float(angle_max)
        scan.angle_increment = float(angle_increment)
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.01  # 1cm minimum range
        scan.range_max = float(max_range)
        
        # Convert ranges to list of floats and filter invalid values
        scan.ranges = []
        for r in ranges:
            try:
                val = float(r)
                if val < scan.range_min or val > scan.range_max or not math.isfinite(val):
                    val = float('inf')
                scan.ranges.append(val)
            except (ValueError, TypeError):
                scan.ranges.append(float('inf'))
        
        self.publishers['scan'].publish(scan)
    
    def get_time(self):
        if self.use_ros2:
            return self.node.get_clock().now().to_msg()
        else:
            return rospy.Time.now()
    
    def shutdown(self):
        self.running = False
        if self.use_ros2 and self.node:
            ros2_executor.remove_node(self.node)
            self.node.destroy_node()

parser = argparse.ArgumentParser()
parser.add_argument('--render', action='store_true', help='Enable rendering if this flag is set')
parser.add_argument('--env', type=str, default='laas.yaml', help='Path to environment YAML file (e.g. map.yaml)')
args, unknown = parser.parse_known_args()


# Load the shared library (make sure libsim.so is built and in the same directory)
lib = ctypes.CDLL(os.path.abspath(lib_path))

render_flag = ctypes.c_bool.in_dll(lib, "RENDER_SIM")
render_flag.value = 1 if args.render else 0


# Define argument types for set_map_params
lib.set_map_params.argtypes = [
    ctypes.c_char_p,
    ctypes.c_double,
    ctypes.POINTER(ctypes.c_double),
    ctypes.c_int,
    ctypes.c_double,
    ctypes.c_double,
    ctypes.c_double,  # speed
    ctypes.c_double,  # angular_speed
]
lib.set_map_params.restype = None

# Define argument types for robot state and control functions
lib.get_robot_state.argtypes = [ctypes.c_int]
lib.get_robot_state.restype = ctypes.POINTER(ctypes.c_double) 

lib.set_robot_velocity.argtypes = [ctypes.c_int, ctypes.c_double, ctypes.c_double, ctypes.c_double]  
lib.set_robot_velocity.restype = None

lib.get_laser_data.argtypes = [ctypes.c_int]
lib.get_laser_data.restype = ctypes.POINTER(ctypes.c_float)  # Changed to float to match C code

lib.set_head_rotation.argtypes = [ctypes.c_int, ctypes.c_double]
lib.set_head_rotation.restype = None

# Define run_simulation (no args)
lib.run_simulation.argtypes = []
lib.run_simulation.restype = None

# Define start_render_thread and stop_render_thread (no args)
lib.start_render_thread.argtypes = []
lib.start_render_thread.restype = None
lib.stop_render_thread.argtypes = []
lib.stop_render_thread.restype = None

yaml_path = config_path + "/" + args.env

# Parse YAML
with open(yaml_path) as f:
    data = yaml.safe_load(f)

image = (env_path+"/"+data['image']).encode('utf-8')
resolution = float(data['resolution'])
origin = (ctypes.c_double * 3)(*data['origin'])
negate = int(data.get('negate', 0))
occupied_thresh = float(data.get('occupied_thresh', 0.65))
free_thresh = float(data.get('free_thresh', 0.196))
speed = float(data.get('speed', 2.0))
angular_speed = float(data.get('angular_speed', 3.141592653589793 / 2))  # Default pi/2
entities = data.get('entities', [])

# Add this after loading the map config
entities = data.get('entities', [])
lib.add_entity.argtypes = [
    ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double,
    ctypes.POINTER(ctypes.c_int), ctypes.c_bool, ctypes.c_bool, ctypes.c_bool, ctypes.c_char_p, ctypes.c_double, ctypes.c_int, ctypes.c_double]
used_types = set()
for ent in entities:
    ent_name = ent.get('name', 'unknown')
    if ent_name in used_types:
        print(f"Warning: Entity name '{ent_name}' is already used. Skipping this entity.")
        continue
    used_types.add(ent_name)
    color = (ctypes.c_int * 4)(*ent['color'])
    lib.add_entity(
        float(ent['start_x']), float(ent['start_y']), float(ent['start_theta']), float(ent['radius']), color, bool(ent['use_keyboard']), bool(ent['use_differential']), bool(ent.get('use_laser', False)), ent_name.encode('utf-8'), float(ent.get('laser_range', 10.0)), int(ent.get('laser_resolution', 1081)), float(ent.get('laser_angle', 6.283185307179586)))

# Call the C function to set parameters
lib.set_map_params(image, resolution, origin, negate, occupied_thresh, free_thresh, speed, angular_speed)

# Start rendering thread if rendering is enabled
if args.render:
    lib.start_render_thread()

# Register cleanup for rendering thread
atexit.register(lib.stop_render_thread)

# Create ROS wrapper instances for each agent
ros1_wrappers = []
ros2_wrappers = []

if HAS_ROS1:
    for idx, ent in enumerate(entities):
        agent_name = ent.get('name', 'unknown')
        if idx == 0:
            ros1_wrappers.append(ROSWrapper(use_ros2=False, agent_idx=idx))
        else:
            ros1_wrappers.append(ROSWrapper(use_ros2=False, agent_name=agent_name, agent_idx=idx))

if HAS_ROS2:
    for idx, ent in enumerate(entities):
        agent_name = ent.get('name', 'unknown')
        if idx == 0:
            ros2_wrappers.append(ROSWrapper(use_ros2=True, agent_idx=idx))
        else:
            ros2_wrappers.append(ROSWrapper(use_ros2=True, agent_name=agent_name, agent_idx=idx))

# Set use_sim_time for ROS1
if HAS_ROS1:
    try:
        import rosparam
        rosparam.set_param('/use_sim_time', True)
    except Exception:
        rospy.set_param('/use_sim_time', True)

# Prepare /clock publisher for ROS1
if HAS_ROS1:
    clock_pub = rospy.Publisher('/clock', Clock, queue_size=10)

SIM_FPS = 60
SIM_STEP = 1.0 / SIM_FPS
sim_time = 0.0

# Shared state for all agents (for TF timer)
latest_agent_states = [{} for _ in entities]

# Global running flag for clean shutdown
running = True

def signal_handler(sig, frame):
    global running
    print("\nReceived Ctrl+C, shutting down...")
    running = False

signal.signal(signal.SIGINT, signal_handler)

# Define the simulation loop as a separate function
def simulation_loop():
    global running, sim_time
    next_step_time = time.time() + SIM_STEP
    while running:
        now = time.time()
        if now < next_step_time:
            time.sleep(next_step_time - now)
            now = time.time()
        elif now > next_step_time + SIM_STEP:
            next_step_time = now + SIM_STEP
        else:
            next_step_time += SIM_STEP

        lib.step_simulation()
        sim_time += SIM_STEP

        # Publish /clock for ROS1
        if HAS_ROS1:
            clock_msg = Clock()
            secs = int(sim_time)
            nsecs = int((sim_time - secs) * 1e9)
            clock_msg.clock.secs = secs
            clock_msg.clock.nsecs = nsecs
            clock_pub.publish(clock_msg)

        for idx, ent in enumerate(entities):
            state = lib.get_robot_state(idx)
            if state:
                laser_data = lib.get_laser_data(idx)
                ranges = []
                if laser_data:
                    laser_array = (ctypes.c_float * int(ent.get('laser_resolution'))).from_address(
                        ctypes.cast(laser_data, ctypes.c_void_p).value)
                    ranges = [float(laser_array[i]) for i in range(int(ent.get('laser_resolution')))]
                x, y, theta, vx, vy, vth, head_yaw = state[0], state[1], state[2], state[3], state[4], state[5], state[6]
                timestamp = None
                max_range = float(ent.get('laser_range'))
                if HAS_ROS1 and idx < len(ros1_wrappers):
                    wrapper = ros1_wrappers[idx]
                    timestamp = rospy.Time.from_sec(sim_time)
                    wrapper.publish_tf(x, y, theta, head_yaw, timestamp)
                    wrapper.publish_odom(x, y, theta, vx, vy, vth, timestamp)
                    if ranges:
                        wrapper.publish_scan(ranges, max_range, -float(ent.get('laser_angle'))/2, float(ent.get('laser_angle'))/2, float(ent.get('laser_angle'))/(int(ent.get('laser_resolution'))-1), timestamp)
                if HAS_ROS2 and idx < len(ros2_wrappers):
                    wrapper = ros2_wrappers[idx]
                    timestamp = wrapper.get_time()
                    wrapper.publish_tf(x, y, theta, head_yaw, timestamp)
                    wrapper.publish_odom(x, y, theta, vx, vy, vth, timestamp)
                    if ranges:
                        wrapper.publish_scan(ranges, max_range, -float(ent.get('laser_angle'))/2, float(ent.get('laser_angle'))/2, float(ent.get('laser_angle'))/(float(ent.get('laser_resolution'))-1), timestamp)
        if HAS_ROS2:
            ros2_executor.spin_once(timeout_sec=0.0)

# Start the simulation loop in a separate thread
simulation_thread = threading.Thread(target=simulation_loop)
simulation_thread.start()

try:
    running = True
    # Wait for the simulation thread to finish
    simulation_thread.join()
except KeyboardInterrupt:
    print("KeyboardInterrupt received, exiting...")
finally:
    # Cleanup ROS nodes
    for wrapper in ros1_wrappers:
        wrapper.shutdown()
    for wrapper in ros2_wrappers:
        wrapper.shutdown()
    if HAS_ROS2:
        rclpy.shutdown()