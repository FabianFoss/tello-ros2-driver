import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
import numpy as np
import tf2_ros

class EKFLocalization(Node):
    def __init__(self):
        super().__init__('ekf_localization_node')

        # Initialize subscribers
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Twist, '/control', self.control_callback, 10)

        # Initialize publisher
        self.ekf_pub = self.create_publisher(Odometry, '/ekf_odom', 10)

        # Initialize TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Initialize EKF state
        self.dt = 1/100  # Time step
        self.state_dim = 6
        self.meas_dim = 6
        self.control_dim = 3

        self.x = np.zeros((self.state_dim, 1))  # Initial state [x, y, z, vx, vy, vz]
        self.P = np.eye(self.state_dim)         # Initial covariance matrix
        self.Q = np.eye(self.state_dim) * 0.1   # Process noise covariance matrix
        self.R = np.eye(self.meas_dim) * 0.1    # Measurement noise covariance matrix
        self.I = np.eye(self.state_dim)         # Identity matrix

        self.u = np.zeros((self.control_dim, 1))
        self.z = np.zeros((self.meas_dim, 1))
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0])

        # Timer to run the EKF update loop
        self.timer = self.create_timer(self.dt, self.ekf_update_loop)

        # Flag to indicate if control input has been received
        self.control_received = False

    def predict(self):
        # State transition model (F)
        F = np.eye(self.state_dim)
        F[0, 3] = self.dt
        F[1, 4] = self.dt
        F[2, 5] = self.dt

        # Control input model (B)
        B = np.zeros((self.state_dim, self.control_dim))
        B[0, 0] = 0.5 * self.dt**2
        B[1, 1] = 0.5 * self.dt**2
        B[2, 2] = 0.5 * self.dt**2
        B[3, 0] = self.dt
        B[4, 1] = self.dt
        B[5, 2] = self.dt

        # If control input is not received, use last known control input
        if not self.control_received:
            self.u = np.zeros((self.control_dim, 1))

        # Predicted state estimate
        self.x = F @ self.x + B @ self.u

        # Predicted covariance estimate
        self.P = F @ self.P @ F.T + self.Q

    def update(self):
        # Measurement model (H)
        H = np.eye(self.meas_dim, self.state_dim)

        # Measurement prediction
        z_pred = H @ self.x

        # Innovation or measurement residual
        y = self.z - z_pred

        # Innovation covariance
        S = H @ self.P @ H.T + self.R

        # Optimal Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)

        # Updated state estimate
        self.x = self.x + K @ y

        # Updated covariance estimate
        self.P = (self.I - K @ H) @ self.P

    def imu_callback(self, msg):
        # Since we are ignoring acceleration, we only handle orientation from IMU
        pass

    def odom_callback(self, msg):
        # Update the measurement vector with odometry data
        self.z[0] = msg.pose.pose.position.x
        self.z[1] = msg.pose.pose.position.y
        self.z[2] = msg.pose.pose.position.z
        self.z[3] = msg.twist.twist.linear.x
        self.z[4] = msg.twist.twist.linear.y
        self.z[5] = msg.twist.twist.linear.z

        # Update orientation from odometry message
        self.orientation[0] = msg.pose.pose.orientation.x
        self.orientation[1] = msg.pose.pose.orientation.y
        self.orientation[2] = msg.pose.pose.orientation.z
        self.orientation[3] = msg.pose.pose.orientation.w

    def control_callback(self, msg):
        # Update the control input vector with commanded speed
        self.u[0] = msg.linear.x
        self.u[1] = msg.linear.y
        self.u[2] = msg.linear.z
        self.control_received = True

    def ekf_update_loop(self):
        # Run the EKF predict and update steps
        self.predict()
        self.update()

        # Publish the updated state
        ekf_msg = Odometry()
        ekf_msg.header.stamp = self.get_clock().now().to_msg()
        ekf_msg.header.frame_id = "odom"
        ekf_msg.child_frame_id = "drone"
        ekf_msg.pose.pose.position.x = self.x[0, 0]
        ekf_msg.pose.pose.position.y = self.x[1, 0]
        ekf_msg.pose.pose.position.z = self.x[2, 0]
        ekf_msg.pose.pose.orientation.x = self.orientation[0]
        ekf_msg.pose.pose.orientation.y = self.orientation[1]
        ekf_msg.pose.pose.orientation.z = self.orientation[2]
        ekf_msg.pose.pose.orientation.w = self.orientation[3]
        ekf_msg.twist.twist.linear.x = self.x[3, 0]
        ekf_msg.twist.twist.linear.y = self.x[4, 0]
        ekf_msg.twist.twist.linear.z = self.x[5, 0]

        self.ekf_pub.publish(ekf_msg)

        # Publish the transform between odom and base_link
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'drone'
        transform.transform.translation.x = self.x[0, 0]
        transform.transform.translation.y = self.x[1, 0]
        transform.transform.translation.z = self.x[2, 0]

        transform.transform.rotation.x = self.orientation[0]
        transform.transform.rotation.y = self.orientation[1]
        transform.transform.rotation.z = self.orientation[2]
        transform.transform.rotation.w = self.orientation[3]

        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = EKFLocalization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Imu
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Twist, TransformStamped
# import numpy as np
# import tf2_ros

# class EKFLocalization(Node):
#     def __init__(self):
#         super().__init__('ekf_localization_node')

#         # Initialize subscribers
#         self.create_subscription(Imu, '/imu', self.imu_callback, 10)
#         self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
#         self.create_subscription(Twist, '/control', self.control_callback, 10)

#         # Initialize publisher
#         self.ekf_pub = self.create_publisher(Odometry, '/ekf_odom', 10)

#         # Initialize TF broadcaster
#         self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

#         # Initialize EKF state
#         self.dt = 1.0/30.0  # Time step
#         self.state_dim = 6
#         self.meas_dim = 6
#         self.control_dim = 3

#         self.x = np.zeros((self.state_dim, 1))  # Initial state [x, y, z, vx, vy, vz]
#         self.P = np.eye(self.state_dim)         # Initial covariance matrix
#         self.Q = np.eye(self.state_dim) * 0.1   # Process noise covariance matrix
#         self.R = np.eye(self.meas_dim) * 0.1    # Measurement noise covariance matrix
#         self.I = np.eye(self.state_dim)         # Identity matrix

#         self.u = np.zeros((self.control_dim, 1))
#         self.z = np.zeros((self.meas_dim, 1))
#         self.orientation = np.array([0.0, 0.0, 0.0, 1.0])

#         # Timer to run the EKF update loop
#         self.timer = self.create_timer(self.dt, self.ekf_update_loop)

#     def predict(self):
#         # State transition model (F)
#         F = np.eye(self.state_dim)
#         F[0, 3] = self.dt
#         F[1, 4] = self.dt
#         F[2, 5] = self.dt

#         # Control input model (B)
#         B = np.zeros((self.state_dim, self.control_dim))
#         B[3, 0] = self.dt
#         B[4, 1] = self.dt
#         B[5, 2] = self.dt

#         # Predicted state estimate
#         self.x = F @ self.x + B @ self.u

#         # Predicted covariance estimate
#         self.P = F @ self.P @ F.T + self.Q

#     def update(self):
#         # Measurement model (H)
#         H = np.eye(self.meas_dim, self.state_dim)

#         # Measurement prediction
#         z_pred = H @ self.x

#         # Innovation or measurement residual
#         y = self.z - z_pred

#         # Innovation covariance
#         S = H @ self.P @ H.T + self.R

#         # Optimal Kalman gain
#         K = self.P @ H.T @ np.linalg.inv(S)

#         # Updated state estimate
#         self.x = self.x + K @ y

#         # Updated covariance estimate
#         self.P = (self.I - K @ H) @ self.P

#     def imu_callback(self, msg):
#         # Update the measurement vector with IMU data
#         self.z[0] = msg.linear_acceleration.x
#         self.z[1] = msg.linear_acceleration.y
#         self.z[2] = msg.linear_acceleration.z

#     def odom_callback(self, msg):
#         # Update the measurement vector with odometry data
#         self.z[3] = msg.twist.twist.linear.x
#         self.z[4] = msg.twist.twist.linear.y
#         self.z[5] = msg.twist.twist.linear.z

#         # Update orientation from odometry message
#         self.orientation[0] = msg.pose.pose.orientation.x
#         self.orientation[1] = msg.pose.pose.orientation.y
#         self.orientation[2] = msg.pose.pose.orientation.z
#         self.orientation[3] = msg.pose.pose.orientation.w

#     def control_callback(self, msg):
#         # Update the control input vector with commanded speed
#         self.u[0] = msg.linear.x
#         self.u[1] = msg.linear.y
#         self.u[2] = msg.linear.z

#     def ekf_update_loop(self):
#         # Run the EKF predict and update steps
#         self.predict()
#         self.update()

#         # Publish the updated state
#         ekf_msg = Odometry()
#         ekf_msg.header.stamp = self.get_clock().now().to_msg()
#         ekf_msg.header.frame_id = "odom"
#         ekf_msg.child_frame_id = "drone"
#         ekf_msg.pose.pose.position.x = self.x[0, 0]
#         ekf_msg.pose.pose.position.y = self.x[1, 0]
#         ekf_msg.pose.pose.position.z = self.x[2, 0]
#         ekf_msg.pose.pose.orientation.x = self.orientation[0]
#         ekf_msg.pose.pose.orientation.y = self.orientation[1]
#         ekf_msg.pose.pose.orientation.z = self.orientation[2]
#         ekf_msg.pose.pose.orientation.w = self.orientation[3]
#         ekf_msg.twist.twist.linear.x = self.x[3, 0]
#         ekf_msg.twist.twist.linear.y = self.x[4, 0]
#         ekf_msg.twist.twist.linear.z = self.x[5, 0]

#         self.ekf_pub.publish(ekf_msg)

#         # Publish the transform between odom and base_link
#         transform = TransformStamped()
#         transform.header.stamp = self.get_clock().now().to_msg()
#         transform.header.frame_id = 'odom'
#         transform.child_frame_id = 'drone'
#         transform.transform.translation.x = self.x[0, 0]
#         transform.transform.translation.y = self.x[1, 0]
#         transform.transform.translation.z = self.x[2, 0]
#         print(self.x[2,0])
#         transform.transform.rotation.x = self.orientation[0]
#         transform.transform.rotation.y = self.orientation[1]
#         transform.transform.rotation.z = self.orientation[2]
#         transform.transform.rotation.w = self.orientation[3]

#         self.tf_broadcaster.sendTransform(transform)

# def main(args=None):
#     rclpy.init(args=args)
#     node = EKFLocalization()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
