import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, PoseWithCovarianceStamped
from std_msgs.msg import Header
import time

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')
        self.publisher = self.create_publisher(PoseArray, 'new_mission', 10)
        self.initial_pos_pub = self.create_publisher(PoseWithCovarianceStamped, "tello_initialpose", 1)
        self.timer = self.create_timer(2.0, self.publish_waypoints)
        
        self.published = False

        #self.publish_waypoints()
    def publish_waypoints(self):
        if not self.published:
            initial_pose = PoseWithCovarianceStamped()
            initial_pose.pose.pose.position.x = 0.0
            initial_pose.pose.pose.position.y = 0.0
            initial_pose.pose.pose.position.z = 0.0

            self.initial_pos_pub.publish(initial_pose)

            time.sleep(10)

            pose_array = PoseArray()
            pose_array.header = Header(frame_id='map', stamp=self.get_clock().now().to_msg())
            
            #((8.1, 8.5, 4.0), (0.0, 0.0, 0.0, 1.0)),
            #((12.7, 19.0, 4.0), (0.0, 0.0, 0.0, 1.0))
            # Example waypoints: only positions are set, orientations are default
            
            waypoints = [
                # ((21.8, 0.0, 4.0), (0.0, 0.0, 0.0, 1.0)), 
                # ((12.5, 0.0, 4.0), (0.0, 0.0, 0.0, 1.0)),
                # ((12.5, 3.0, 4.0), (0.0, 0.0, 0.0, 1.0)), 
                #((2.0, 0.0, 4.0), (0.0, 0.0, 0.0, 1.0)),
                #((10.13, 0.0, 4.0), (0.0, 0.0, 0.0, 1.0))

                ((4.5, 0.0, 4.0), (0.0, 0.0, 0.0, 1.0)),
                ((4.5, -4.0, 4.0), (0.0, 0.0, 0.0, 1.0)),
                ((4.5, 0.0, 4.0), (0.0, 0.0, 0.0, 1.0)),
                ((0.0, 0.0, 4.0), (0.0, 0.0, 0.0, 1.0)),
            ]
            
            for position, orientation in waypoints:
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = position
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = orientation
                pose_array.poses.append(pose)

            self.publisher.publish(pose_array)
            self.get_logger().info('Publishing waypoints...')
            self.published = True
        else:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = WaypointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
