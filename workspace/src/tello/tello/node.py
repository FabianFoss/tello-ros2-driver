#!/usr/bin/env python3

import pprint
import math
import rclpy
import threading
import numpy
import time
import av
import tf2_ros
import cv2
import time
import yaml

from djitellopy import Tello

from rclpy.node import Node
from tello_msg.msg import TelloStatus, TelloID, TelloWifiConfig
from std_msgs.msg import Empty, String
from sensor_msgs.msg import Image, Imu, BatteryState, Temperature, CameraInfo
from geometry_msgs.msg import Twist, TransformStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import ament_index_python
from rclpy.action import ActionServer
from takeoff_and_land_interface.action import TakeoffAndLand

# Tello ROS node class, inherits from the Tello controller object.
#
# Can be configured to be used by multiple drones, publishes, all data collected from the drone and provides control using ROS messages.
class TelloNode():
    def __init__(self, node):
        # ROS node
        self.node = node

        # Declare parameters
        self.node.declare_parameter('connect_timeout', 100.0)
        self.node.declare_parameter('tello_ip', '192.168.10.1')
        self.node.declare_parameter('tf_base', 'map')
        self.node.declare_parameter('tf_drone', 'drone')
        self.node.declare_parameter('tf_pub', True)
        self.node.declare_parameter('camera_info_file', '')

        # Get parameters
        self.connect_timeout = float(self.node.get_parameter('connect_timeout').value)
        self.tello_ip = str(self.node.get_parameter('tello_ip').value)
        self.tf_base = str(self.node.get_parameter('tf_base').value)
        self.tf_drone = str(self.node.get_parameter('tf_drone').value)
        self.tf_pub = bool(self.node.get_parameter('tf_pub').value)
        self.camera_info_file = str(self.node.get_parameter('camera_info_file').value)

        # Camera information loaded from calibration yaml
        self.camera_info = None
        
        # Check if camera info file was received as argument
        if len(self.camera_info_file) == 0:
            share_directory = ament_index_python.get_package_share_directory('tello')
            self.camera_info_file = share_directory + '/ost.yaml'

        # Read camera info from YAML file
        with open(self.camera_info_file, 'r') as file:
            self.camera_info = yaml.load(file, Loader=yaml.FullLoader)
            self.node.get_logger().info('Tello: Camera information YAML' + self.camera_info.__str__()) # Not actually correct for tello, but modified for depth image approach

        # Configure drone connectionFalse
        Tello.TELLO_IP = self.tello_ip
        Tello.RESPONSE_TIMEOUT = int(self.connect_timeout)

        self.node.action_server = ActionServer(self.node, TakeoffAndLand, "takeoff_and_land_command", self.auto_tkf_lnd)



        self.last_update_time = self.node.get_clock().now()
        # Initialize position attributes
        self.position_x = 0.0
        self.position_y = 0.0
        self.position_z = 0.0
        self.initial_orientation = [0.0, 0.0, 0.0, 0.0]

        # Connect to drone
        self.node.get_logger().info('Tello: Connecting to drone')

        self.tello = Tello()
        self.tello.connect()

        self.node.get_logger().info('Tello: Connected to drone')

        # Publishers and subscribers
        self.setup_publishers()
        self.setup_subscribers()

        # Processing threads
        self.start_video_capture()
        self.start_tello_status()
        self.start_tello_odom()

        self.node.get_logger().info('Tello: Driver node ready')

    # Setup ROS publishers of the node.
    def setup_publishers(self):
        self.pub_image_raw = self.node.create_publisher(Image, 'image_raw', 1)
        self.pub_camera_info = self.node.create_publisher(CameraInfo, 'camera_info', 1) # Modified for depth image approach
        self.pub_status = self.node.create_publisher(TelloStatus, 'status', 1)
        self.pub_id = self.node.create_publisher(TelloID, 'id', 1)
        self.pub_imu = self.node.create_publisher(Imu, 'imu', 1)
        self.pub_battery = self.node.create_publisher(BatteryState, 'battery', 1)
        self.pub_temperature = self.node.create_publisher(Temperature, 'temperature', 100)
        self.pub_odom = self.node.create_publisher(Odometry, 'odom', 1)

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self.node)
    
    # Setup the topic subscribers of the node.
    def setup_subscribers(self):
        self.sub_emergency = self.node.create_subscription(Empty, 'emergency', self.cb_emergency, 1)
        self.sub_takeoff = self.node.create_subscription(Empty, 'takeoff', self.cb_takeoff, 1)
        self.sub_land = self.node.create_subscription(Empty, 'land', self.cb_land, 1)
        self.sub_control = self.node.create_subscription(Twist, 'control', self.cb_control, 1)
        self.cmd_vel = self.node.create_subscription(Twist, 'cmd_vel', self.cb_cmd_vel, 10)
        self.sub_flip = self.node.create_subscription(String, 'flip', self.cb_flip, 1)
        self.sub_wifi_config = self.node.create_subscription(TelloWifiConfig, 'wifi_config', self.cb_wifi_config, 1)
        self.sub_initial_pose = self.node.create_subscription(PoseWithCovarianceStamped,'tello_initialpose', self.cb_initial_pose, 10)
    
    # Get the orientation of the drone as a quaternion
    def get_orientation_quaternion(self):
        deg_to_rad = math.pi / 180.0
        return euler_to_quaternion([
            self.tello.get_yaw() * deg_to_rad*-1,
            self.tello.get_pitch() * deg_to_rad*-1,
            self.tello.get_roll() * deg_to_rad
        ])

    # Start drone info thread
    def start_tello_odom(self, rate=1.0/30.0):
        def status_odom():
            while True:
                current_time = self.node.get_clock().now()
                dt = (current_time - self.last_update_time).nanoseconds / 1e9  # Convert from nanoseconds to seconds
                self.last_update_time = current_time

                
                if self.tf_pub:
                    speed_x = float(self.tello.get_speed_x()) / 10.0
                    speed_y = float(self.tello.get_speed_y()) / 10.0*-1 # Flip axis
                    speed_z = float(self.tello.get_speed_z()) / 10.0*-1
                    
                    # Get orientation quaternion
                    q = self.get_orientation_quaternion()
                    # self.node.get_logger().info(str(q))
                    # Update position based on speed and delta time
                    
                    self.position_x += speed_x * dt
                    self.position_y += speed_y * dt
                    self.position_z += speed_z * dt
                
                    # Transform odom -> drone
                    t_odom_base = TransformStamped()
                    t_odom_base.header.stamp = self.node.get_clock().now().to_msg()
                    t_odom_base.header.frame_id = 'odom'
                    t_odom_base.child_frame_id = self.tf_drone

                    t_odom_base.transform.rotation.x = q[0] + self.initial_orientation[0]
                    t_odom_base.transform.rotation.y = q[1] + self.initial_orientation[1]
                    t_odom_base.transform.rotation.z = q[2] + self.initial_orientation[2]
                    t_odom_base.transform.rotation.w = q[3] + self.initial_orientation[3]

                    t_odom_base.transform.translation.x = self.position_x
                    t_odom_base.transform.translation.y = self.position_y
                    t_odom_base.transform.translation.z = self.position_z

                    self.tf_broadcaster.sendTransform(t_odom_base)

                if True:
                    
                    # Get orientation quaternion
                    q = self.get_orientation_quaternion()

                    # Publish IMU
                    msg = Imu()
                    msg.header.stamp = self.node.get_clock().now().to_msg()
                    msg.header.frame_id = self.tf_drone
                    
                    # Calculate acceleration
                    acc_x = self.tello.get_acceleration_x() / 10.0
                    acc_y = self.tello.get_acceleration_y() / 10.0
                    acc_z = self.tello.get_acceleration_z() / 10.0
                    
                    msg.linear_acceleration.x = acc_x
                    msg.linear_acceleration.y = acc_y
                    msg.linear_acceleration.z = acc_z
                    
                    msg.orientation.x = q[0]
                    msg.orientation.y = q[1]
                    msg.orientation.z = q[2]
                    msg.orientation.w = q[3]

                    # Publish odometry
                    odom_msg = Odometry()
                    odom_msg.header.stamp = self.node.get_clock().now().to_msg()
                    odom_msg.header.frame_id = "odom"

                    speed_x = float(self.tello.get_speed_x()) / 10.0
                    speed_y = float(self.tello.get_speed_y()) / 10.0*-1 # Flip axis
                    speed_z = float(self.tello.get_speed_z()) / 10.0*-1
                    
                    odom_msg.twist.twist.linear.x = speed_x
                    odom_msg.twist.twist.linear.y = speed_y
                    odom_msg.twist.twist.linear.z = speed_z
                    
                    odom_msg.pose.pose.orientation.x = q[0] + self.initial_orientation[0]
                    odom_msg.pose.pose.orientation.y = q[1] + self.initial_orientation[1]
                    odom_msg.pose.pose.orientation.z = q[2] + self.initial_orientation[2]
                    odom_msg.pose.pose.orientation.w = q[3] + self.initial_orientation[3]
                    
                    odom_msg.pose.pose.position.x = self.position_x
                    odom_msg.pose.pose.position.y = self.position_y
                    odom_msg.pose.pose.position.z = self.position_z

                    self.pub_odom.publish(odom_msg)
                    self.pub_imu.publish(msg)

                time.sleep(rate)

        thread = threading.Thread(target=status_odom)
        thread.start()
        return thread

    # Start drone info thread
    def start_tello_status(self, rate=1.0/30.0):
        def status_loop():
            while True:
                # Battery
                if self.pub_battery.get_subscription_count() > 0:
                    msg = BatteryState()
                    msg.header.stamp = self.node.get_clock().now().to_msg()
                    msg.header.frame_id = self.tf_drone
                    msg.percentage = float(self.tello.get_battery())
                    msg.voltage = 3.8
                    msg.design_capacity = 1.1
                    msg.present = True
                    msg.power_supply_technology = 2 # POWER_SUPPLY_TECHNOLOGY_LION
                    msg.power_supply_status = 2 # POWER_SUPPLY_STATUS_DISCHARGING
                    self.pub_battery.publish(msg)

                # Temperature
                if self.pub_temperature.get_subscription_count() > 0:
                    msg = Temperature()
                    msg.header.stamp = self.node.get_clock().now().to_msg()
                    msg.header.frame_id = self.tf_drone
                    msg.temperature = self.tello.get_temperature()
                    msg.variance = 0.0
                    self.pub_temperature.publish(msg)

                # Tello Statuscf_initial_pose
                if self.pub_status.get_subscription_count() > 0:
                    msg = TelloStatus()
                    msg.acceleration.x = self.tello.get_acceleration_x()
                    msg.acceleration.y = self.tello.get_acceleration_y()
                    msg.acceleration.z = self.tello.get_acceleration_z()

                    msg.speed.x = float(self.tello.get_speed_x())
                    msg.speed.y = float(self.tello.get_speed_y())
                    msg.speed.z = float(self.tello.get_speed_z())

                    msg.pitch = self.tello.get_pitch()
                    msg.roll = self.tello.get_roll()
                    msg.yaw = self.tello.get_yaw()

                    msg.barometer = int(self.tello.get_barometer())
                    msg.distance_tof = self.tello.get_distance_tof()

                    msg.fligth_time = self.tello.get_flight_time()

                    msg.battery = self.tello.get_battery()

                    msg.highest_temperature = self.tello.get_highest_temperature()
                    msg.lowest_temperature = self.tello.get_lowest_temperature()
                    msg.temperature = self.tello.get_temperature()

                    msg.wifi_snr = self.tello.query_wifi_signal_noise_ratio()

                    self.pub_status.publish(msg)

                # Tello ID
                if self.pub_id.get_subscription_count() > 0:
                    msg = TelloID()
                    
                    msg.sdk_version = self.tello.query_sdk_version()
                    msg.serial_number = self.tello.query_serial_number()
                    self.pub_id.publish(msg)

                # Camera info
                if self.pub_camera_info.get_subscription_count() > 0:
                    msg = CameraInfo()
                    
                    msg.header.stamp = self.node.get_clock().now().to_msg()
                    msg.header.frame_id = self.camera_info['camera_name']
                    msg.height = self.camera_info['image_height']
                    msg.width = self.camera_info['image_width']
                    msg.distortion_model = self.camera_info['distortion_model']
                    
                    msg.k = self.camera_info['camera_matrix']['data']
                    msg.d = self.camera_info['distortion_coefficients']['data']  # Directly use the list
                    msg.r = self.camera_info['rectification_matrix']['data']   
                    msg.p = self.camera_info['projection_matrix']['data'] 
                    
                    self.pub_camera_info.publish(msg)
                
                # Sleep
                time.sleep(rate)

        thread = threading.Thread(target=status_loop)
        thread.start()
        return thread


    # Start video capture thread.
    def start_video_capture(self, rate=1.0/30.0):
        # Enable tello stream
        self.tello.streamon()

        # OpenCV bridge
        self.bridge = CvBridge()

        def video_capture_thread():
            frame_read = self.tello.get_frame_read()

            while True:
                # Get frame from drone
                frame = frame_read.frame

                # Publish opencv frame using CV bridge
                msg = self.bridge.cv2_to_imgmsg(numpy.array(frame), 'bgr8')
                msg.header.stamp = self.node.get_clock().now().to_msg()
                #msg.header.frame_id = "depth_camera_frame"
                self.pub_image_raw.publish(msg)

                time.sleep(rate)
                

        # We need to run the recorder in a seperate thread, otherwise blocking options would prevent frames from getting added to the video
        thread = threading.Thread(target=video_capture_thread)
        thread.start()
        return thread

    # Terminate the code and shutdown node.
    def terminate(self, err):
        self.node.get_logger().error(str(err))
        self.tello.end()
        rclpy.shutdown()

    # Stop all movement in the drone
    def cb_emergency(self, msg):
        self.tello.emergency()

    # Drone takeoff message control
    def cb_takeoff(self, msg):
        self.tello.takeoff()

    # Land the drone message callback
    def cb_land(self, msg):
        self.tello.land()


    def auto_tkf_lnd(self, goal_handle):
        self.node.get_logger().info(f"Received request: {goal_handle.request.command}")
        result = TakeoffAndLand.Result()
        feedback = TakeoffAndLand.Feedback()

        command = goal_handle.request.command
        if command == 'takeoff':
            feedback.status = 'Taking off'
            goal_handle.publish_feedback(feedback)
            self.cb_takeoff("takeoff")
            self.node.get_clock().sleep_for(rclpy.duration.Duration(seconds=1))            
            feedback.status = 'At altitude'
        elif command == 'land':
            feedback.status = 'Landing'
            goal_handle.publish_feedback(feedback)
            self.cb_land("land")
            self.node.get_clock().sleep_for(rclpy.duration.Duration(seconds=5))        
            feedback.status = 'On ground'
        
        goal_handle.publish_feedback(feedback)
        result.success = True
        return result

    # Control messages received use to control the drone "analogically"
    #
    # This method of controls allow for more precision in the drone control.
    #
    # Receives the linear and angular velocities to be applied from -100 to 100.
    def cb_control(self, msg):
        self.tello.send_rc_control(int(msg.linear.x), int(msg.linear.y), int(msg.linear.z), int(msg.angular.z))

    def cb_cmd_vel(self, msg):
        x = msg.linear.y*100  # Now x is aligned with drone's forward (-y is now x)
        y = msg.linear.x*100   # y takes the value of x
        z = msg.linear.z*100   # z remains unchanged
        yaw = -msg.angular.z*50  # yaw remains unchanged

        self.tello.send_rc_control(int(x), int(y), int(z), int(yaw))
        
    # Configure the wifi credential that should be used by the drone.
    #
    # The drone will be restarted after the credentials are changed.
    def cb_wifi_config(self, msg):
        self.tello.set_wifi_credentials(msg.ssid, msg.password)
    
    # Perform a drone flip in a direction specified.
    # 
    # Directions can be "r" for right, "l" for left, "f" for forward or "b" for backward.
    def cb_flip(self, msg):
        self.tello.flip(msg.data)

    def cb_initial_pose(self, msg):
        # Extract the position from the initial pose message
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y
        self.position_z = msg.pose.pose.position.z

        # Extract the orientation (quaternion) from the initial pose message
        self.initial_orientation[0] = msg.pose.pose.orientation.x
        self.initial_orientation[1] = msg.pose.pose.orientation.y
        self.initial_orientation[2] = msg.pose.pose.orientation.z
        self.initial_orientation[3] = msg.pose.pose.orientation.w

        # Log the received initial pose
        self.node.get_logger().info(
            f'Received initial pose: x={self.position_x}, y={self.position_y}, z={self.position_z}, '
            f'orientation=[{self.initial_orientation[0]}, {self.initial_orientation[1]}, {self.initial_orientation[2]}, {self.initial_orientation[3]}]'
        )

        # Create a TransformStamped message
        t_initial_pose = TransformStamped()
        t_initial_pose.header.stamp = self.node.get_clock().now().to_msg()
        t_initial_pose.header.frame_id = 'odom'
        t_initial_pose.child_frame_id = self.tf_drone

        t_initial_pose.transform.rotation.x = self.initial_orientation[0]
        t_initial_pose.transform.rotation.y = self.initial_orientation[1]
        t_initial_pose.transform.rotation.z = self.initial_orientation[2]
        t_initial_pose.transform.rotation.w = self.initial_orientation[3]

        t_initial_pose.transform.translation.x = self.position_x
        t_initial_pose.transform.translation.y = self.position_y
        t_initial_pose.transform.translation.z = self.position_z

        # Send the transform
        self.tf_broadcaster.sendTransform(t_initial_pose)

# Convert a rotation from euler to quaternion.
def euler_to_quaternion(r):
    (yaw, pitch, roll) = (r[0], r[1], r[2])
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

# Convert rotation from quaternion to euler.
def quaternion_to_euler(q):
    (x, y, z, w) = (q[0], q[1], q[2], q[3])
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [yaw, pitch, roll]

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('tello')
    drone = TelloNode(node)

    rclpy.spin(node)

    drone.cb_shutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
