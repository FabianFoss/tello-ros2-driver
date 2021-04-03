import math
import rclpy
import av
import threading
import numpy
import tf
import tellopy

from tello.msg import FlightStatus
from std_msgs.msg import Empty, UInt8, UInt8, Bool, UInt8MultiArray
from sensor_msgs.msg import Image, Imu, BatteryState, Temperature, CameraInfo
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError

# Video rates possible for the tello drone
VIDEO_AUTO = 0
VIDEO_1MBS = 1
VIDEO_1_5MBS = 2
VIDEO_2MBS = 3
VIDEO_2_5MBS = 4

# Tello ROS node class, inherits from the tellopy.Tello controller object.
#
# Can be configured to be used by multiple drones, publishes, all data collected from the drone and provides control using ROS messages.
class TelloNode(tellopy.Tello):
    def __init__(self):
        # Connection parameters
        self.connect_timeout_sec = float(rclpy.get_param('~connect_timeout_sec', 10.0))
        self.tello_ip = rclpy.get_param('~tello_ip', '192.168.10.1')

        # TF parameters
        self.tf_base = rclpy.get_param('~tf_base', 'map')
        self.tf_drone = rclpy.get_param('~tf_drone', 'drone')
        self.tf_drone_body = rclpy.get_param('~tf_drone_body', 'body')

        # OpenCV bridge
        self.bridge = CvBridge()
        self.frame_thread = None

        super(TelloNode, self).__init__(self.tello_ip)

        # Connect to drone
        rclpy.loginfo('Tello: Connecting to drone %s', self.tello_addr)
        self.connect()

        try:
            self.wait_for_connection(timeout=self.connect_timeout_sec)
        except Exception as err:
            self.terminate(err)
            return

        rclpy.loginfo('Tello: Connected to drone')
        rclpy.on_shutdown(self.cb_shutdown)

        # Max position delta without applying correction
        self.pos_max_delta = 2.0

        # Correction applied to drone positioning
        self.pos_delta_x = 0.0
        self.pos_delta_y = 0.0
        self.pos_delta_z = 0.0

        # Position of the drone
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0

        # Setup dynamic reconfigure
        self.cfg = None

        # Setup ROS publishers
        self.pub_image_raw = rclpy.Publisher('image_raw', Image, queue_size=1)
        self.pub_camera_info = rclpy.Publisher('camera_info', CameraInfo, queue_size=1, latch=True)
        self.pub_status = rclpy.Publisher('status', FlightStatus, queue_size=1, latch=True)
        self.pub_imu = rclpy.Publisher('imu', Imu, queue_size=1, latch=True)
        self.pub_battery = rclpy.Publisher('battery', BatteryState, queue_size=1, latch=True)
        self.pub_temperature = rclpy.Publisher('temperature', Temperature, queue_size=1, latch=True)
        self.pub_odom = rclpy.Publisher('odom', Odometry, queue_size=1, latch=True)

        # Setup TF broadcaster
        self.tf_br = tf.TransformBroadcaster()

        # Setup ROS subscribers
        def cb_stop(msg):
            self.right(0)
            self.left(0)
            self.up(0)
            self.down(0)
            self.forward(0)
            self.backward(0)
            self.clockwise(0)
            self.counter_clockwise(0)
        self.sub_stop = rclpy.Subscriber('stop', Empty, cb_stop, queue_size=10)

        def cb_takeoff(msg):
            self.takeoff()
        self.sub_takeoff = rclpy.Subscriber('takeoff', Empty, cb_takeoff, queue_size=10)

        def cb_land(msg):
            self.land()
        self.sub_land = rclpy.Subscriber('land', Empty, cb_land, queue_size=10)

        def cb_left(msg):
            self.left(msg.data)
        self.sub_left = rclpy.Subscriber('left', UInt8, cb_left, queue_size=10)

        def cb_right(msg):
            self.right(msg.data)
        self.sub_right = rclpy.Subscriber('right', UInt8, cb_right, queue_size=10)

        def cb_up(msg):
            self.up(msg.data)
        self.sub_up = rclpy.Subscriber('up', UInt8, cb_up, queue_size=10)

        def cb_down(msg):
            self.down(msg.data)
        self.sub_down = rclpy.Subscriber('down', UInt8, cb_down, queue_size=10)

        def cb_forward(msg):
            self.forward(msg.data)
        self.sub_forward = rclpy.Subscriber('forward', UInt8, cb_forward, queue_size=10)

        def cb_backward(msg):
            self.backward(msg.data)
        self.sub_backward = rclpy.Subscriber('backward', UInt8, cb_backward, queue_size=10)

        def cb_counter_clockwise(msg):
            self.clockwise(msg.data)
        self.sub_counter_clockwise = rclpy.Subscriber('counter_clockwise', UInt8, cb_counter_clockwise, queue_size=10)

        def cb_clockwise(msg):
            self.clockwise(msg.data)
        self.sub_clockwise = rclpy.Subscriber('clockwise', UInt8, cb_clockwise, queue_size=10)

        self.sub_cmd_vel = rclpy.Subscriber('cmd_vel', Twist, self.cb_cmd_vel, queue_size=10)
        self.sub_fast_mode = rclpy.Subscriber('fast_mode', Bool, self.cb_fast_mode)
        self.sub_throw_takeoff = rclpy.Subscriber('throw_takeoff', Empty, self.cb_throw_takeoff)
        self.sub_palm_land = rclpy.Subscriber('palm_land', Empty, self.cb_palm_land)

        # Subscribe data from drone
        self.subscribe(self.EVENT_FLIGHT_DATA, self.cb_drone_flight_data)
        self.subscribe(self.EVENT_LOG_DATA, self.cb_drone_odom_log_data)
        # self.subscribe(self.EVENT_LIGHT, self.cb_drone_light_data)

        # Frame grabber thread
        self.frame_thread = threading.Thread(target=self.camera_loop)
        self.frame_thread.start()

        # Configure video encoder rate
        self.set_video_encoder_rate(VIDEO_1MBS)
        self.set_video_mode(False)

        rclpy.loginfo('Tello: Driver node ready')

    # Camera processing thread method should be called passed to a Thread object
    def camera_loop(self):
        # Configure node loop rate
        rate = rclpy.Rate(30)
        frame_id = self.tf_drone

        # Drone video capture
        self.start_video()
        drone_stream = self.get_video_stream()
        container = None
        video_stream = None
        frame_dropped = 0

        # Drone processing cycle
        while self.state != self.STATE_QUIT:
            # Try to connect video
            if container is None:
                try:
                    container = av.open(drone_stream)
                    video_stream = container.streams.video[0]
                except Exception as err:
                    container = None
                    rclpy.logerr('Tello: Failed to connect video stream (pyav) - %s' % str(err))
                    self.terminate(err)

            # Process frames from drone camera
            else:
                try:
                    for packet in container.demux(video_stream):
                        for frame in packet.decode():
                            img = frame.to_image()
                            arr = numpy.array(img)
                            img_msg = self.bridge.cv2_to_imgmsg(arr, 'rgb8')
                            img_msg.header.frame_id = frame_id
                            self.pub_image_raw.publish(img_msg)

                    # Reset frame dropped counter
                    frame_dropped = 0

                    # Camera info message
                    camera_info_msg = CameraInfo()
                    camera_info_msg.header.stamp = rclpy.Time.now()
                    camera_info_msg.header.frame_id = self.tf_drone
                    camera_info_msg.width = 960
                    camera_info_msg.height = 720
                    camera_info_msg.D = [-0.101373, 0.179355, -0.015361, -0.005923, 0.000000]
                    camera_info_msg.K = [889.998695, 0.0, 459.681039, 0.0, 896.051429, 328.962342, 0.0, 0.0, 1.0]
                    camera_info_msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
                    camera_info_msg.P = [889.546814, 0.0, 455.164911, 0.0, 0.0, 888.916870, 319.444672, 0.0, 0.0, 0.0, 1.0, 1.0]
                    self.pub_camera_info.publish(camera_info_msg)
                except Exception as err:
                    frame_dropped += 1
                    # If more than 100 frames dropped assume drone connection was lost
                    if frame_dropped > 100:
                        rclpy.logerr('Tello: Connection to the drone was lost - %s' % str(err))
                        self.terminate(err)

            rate.sleep()

    def terminate(self, err):
        rclpy.logerr(str(err))
        rclpy.signal_shutdown(str(err))
        self.quit()

    def cb_shutdown(self):
        self.quit()

        if self.frame_thread is not None:
            self.frame_thread.join()

    # Callback method called when the drone sends light data
    def cb_drone_light_data(self, event, sender, data, **args):
        return
        #print "------------LDATA--------------"
        #print data.__dict__
        #print "----------- MVO DATA-----------"
        #print data.mvo.__dict__
        #print "----------- IMU DATA-----------"
        #print data.imu.__dict__
        #print "-------------------------------"

    # Callback method called when the drone sends odom info
    def cb_drone_odom_log_data(self, event, sender, data, **args):
        #print "----------- MVO DATA-----------"
        #print data.mvo.__dict__
        #print "Pos X" + str(data.mvo.pos_x) + " Y" + str(data.mvo.pos_y) + " Z" + str(data.mvo.pos_z)
        #print "Vel X" + str(data.mvo.vel_x) + " Y" + str(data.mvo.vel_y) + " Z" + str(data.mvo.vel_z)
        #print "----------- IMU DATA-----------"
        #print data.imu.__dict__
        #print "----------LOG DATA-------------"
        #print data.imu.log.__dict__
        #print "-------------------------------"

        # Calculate delta between positions
        delta = math.sqrt(math.pow(self.pos_x - data.mvo.pos_x, 2) + math.pow(self.pos_y - data.mvo.pos_y, 2) + math.pow(self.pos_z - data.mvo.pos_z, 2))

        # Apply correction to position
        if delta > self.pos_max_delta:
            self.pos_delta_x += data.mvo.pos_x - self.pos_x
            self.pos_delta_y += data.mvo.pos_y - self.pos_y
            self.pos_delta_z += data.mvo.pos_z - self.pos_z


        # Position of the drone
        self.pos_x = data.mvo.pos_x
        self.pos_y = data.mvo.pos_y
        self.pos_z = data.mvo.pos_z

        pos_x = self.pos_x - self.pos_delta_x
        pos_y = self.pos_y - self.pos_delta_y
        pos_z = self.pos_z - self.pos_delta_z


        quaternion = (data.imu.q0, data.imu.q1, data.imu.q2, data.imu.q3)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]

        quaternion_roll = tf.transformations.quaternion_from_euler(0.0, 0.0, -roll)

        #print "----------CALCULATED POSITION-------------"
        #print "Euler Roll:" + str(euler[0]) + " Pitch: " + str(euler[1]) + "Yaw: " + str(euler[2])
        #print "Delta X" + str(self.pos_delta_x) + " Y" + str(self.pos_delta_y) + " Z" + str(self.pos_delta_z)
        #print "MVO Position X" + str(data.mvo.pos_x) + " Y" + str(data.mvo.pos_y) + " Z" + str(data.mvo.pos_z)
        #print "Calculated Position X" + str(pos_x) + " Y" + str(pos_y) + " Z" + str(pos_z)
        #print "------------------------------------------"

        # Publish drone transform
        self.tf_br.sendTransform((pos_x, pos_y, pos_z), (0.0, 0.0, 0.0, 1.0), rclpy.Time.now(), self.tf_base, self.tf_drone)
        self.tf_br.sendTransform((0.0, 0.0, 0.0), (quaternion_roll[0], quaternion_roll[1], quaternion_roll[2], quaternion_roll[3]), rclpy.Time.now(), self.tf_drone, self.tf_drone_body)

        # Publish odom data
        odom_msg = Odometry()
        odom_msg.header.stamp = rclpy.Time.now()
        odom_msg.header.frame_id = self.tf_base
        odom_msg.pose.pose.position.x = pos_x
        odom_msg.pose.pose.position.y = pos_y
        odom_msg.pose.pose.position.z = pos_z
        odom_msg.twist.twist.linear.x = data.mvo.vel_x
        odom_msg.twist.twist.linear.y = data.mvo.vel_y
        odom_msg.twist.twist.linear.z = data.mvo.vel_z
        self.pub_odom.publish(odom_msg)

        # Publish IMU data
        imu_msg = Imu()
        imu_msg.header.stamp = rclpy.Time.now()
        imu_msg.header.frame_id = self.tf_drone
        imu_msg.header.seq = count = data.count
        imu_msg.linear_acceleration.x = data.imu.acc_x
        imu_msg.linear_acceleration.y = data.imu.acc_y
        imu_msg.linear_acceleration.z = data.imu.acc_z
        imu_msg.angular_velocity.x = data.imu.gyro_x
        imu_msg.angular_velocity.y = data.imu.gyro_y
        imu_msg.angular_velocity.z = data.imu.gyro_z
        imu_msg.orientation.x = quaternion_roll[0]
        imu_msg.orientation.y = quaternion_roll[1]
        imu_msg.orientation.z = quaternion_roll[2]
        imu_msg.orientation.w = quaternion_roll[3]
        self.pub_imu.publish(imu_msg)

    # Callback called every time the drone sends information
    def cb_drone_flight_data(self, event, sender, data, **args):
        # Publish battery message
        battery_msg = BatteryState()
        battery_msg.percentage = data.battery_percentage
        battery_msg.voltage = 3.8
        battery_msg.design_capacity = 1.1
        battery_msg.present = True
        battery_msg.power_supply_technology = 2 # POWER_SUPPLY_TECHNOLOGY_LION
        battery_msg.power_supply_status = 2 # POWER_SUPPLY_STATUS_DISCHARGING
        self.pub_battery.publish(battery_msg)

        # Publish temperature data
        temperature_msg = Temperature()
        temperature_msg.header.frame_id = self.tf_drone
        temperature_msg.temperature = data.temperature_height
        self.pub_temperature.publish(temperature_msg)

        # Publish flight data
        msg = FlightStatus()
        msg.imu_state = data.imu_state
        msg.pressure_state = data.pressure_state
        msg.down_visual_state = data.down_visual_state
        msg.gravity_state = data.gravity_state
        msg.wind_state = data.wind_state
        msg.drone_hover = data.drone_hover
        msg.factory_mode = data.factory_mode
        msg.imu_calibration_state = data.imu_calibration_state
        msg.fly_mode = data.fly_mode #1 Landed | 6 Flying
        msg.camera_state = data.camera_state
        msg.electrical_machinery_state = data.electrical_machinery_state

        msg.front_in = data.front_in
        msg.front_out = data.front_out
        msg.front_lsc = data.front_lsc

        msg.power_state = data.power_state
        msg.battery_state = data.battery_state
        msg.battery_low = data.battery_low
        msg.battery_lower = data.battery_lower
        msg.battery_percentage = data.battery_percentage
        msg.drone_battery_left = data.drone_battery_left

        msg.light_strength = data.light_strength

        msg.fly_speed = data.fly_speed
        msg.east_speed = data.east_speed
        msg.north_speed = data.north_speed
        msg.ground_speed = data.ground_speed
        msg.fly_time = data.fly_time
        msg.drone_fly_time_left = data.drone_fly_time_left

        msg.wifi_strength = data.wifi_strength
        msg.wifi_disturb = data.wifi_disturb

        msg.height = data.height
        msg.temperature_height = data.temperature_height

        self.pub_status.publish(msg)

    # Callback for the palm landing functionality subscriber
    def cb_palm_land(self, msg):
        self.palm_land()

    # Call used to set the fast mode value, should be used carefully to avoid damaging the drone.
    def cb_fast_mode(self, msg):
        self.fast_mode = msg.data

    # Callback for cmd_vel messages received use to control the drone "analogically"
    #
    # This method of controls allow for more precision in the drone control.
    def cb_cmd_vel(self, msg):
        self.set_pitch(msg.linear.x)
        self.set_roll(-msg.linear.y)
        self.set_throttle(msg.linear.z)
        self.set_yaw(-msg.angular.z)

    # Callback for the throw takeoff functionality
    def cb_throw_takeoff(self, msg):
        self.throw_and_go()


def main(args=None):
    rclpy.init(args=args)

    # drone = TelloNode()

    while True: # drone.state != drone.STATE_QUIT:
        print('running')
        rclpy.spin()

if __name__ == '__main__':
    main()