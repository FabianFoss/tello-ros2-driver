import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from takeoff_and_land_interface.action import TakeoffAndLand

class WaypointFollowerClient(Node):
    def __init__(self):
        super().__init__('waypoint_follower_client')
        self.takeoff_land_client = ActionClient(self, TakeoffAndLand, '/takeoff_and_land_command')
        self.waypoint_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        self.waypoints_subscriber = self.create_subscription(
            PoseArray,
            'new_mission',
            self.cb_mission,
            10)
        self.waypoints = None
        self.get_logger().info("Waypoint follower client has been started.")
    
    def cb_mission(self, msg):
        self.get_logger().info("Got mission message!")
        if self.waypoints is None:
            poses = msg.poses  # Assuming z=0 for all waypoints
            waypoints = [create_pose(pt.position.x, pt.position.y, 0) for pt in poses]  # Assuming z=0 for all waypoints for simplicity purposes           self.send_takeoff_or_land("takeoff")
            self.waypoints = waypoints
            self.send_takeoff_or_land("takeoff")
        else:
            self.get_logger().info("Already on a mission")

    def send_takeoff_or_land(self, command):
        goal_msg = TakeoffAndLand.Goal()
        goal_msg.command = command

        self.takeoff_land_client.wait_for_server()
        self.future = self.takeoff_land_client.send_goal_async(goal_msg)
        self.future.add_done_callback(self.handle_land_response if command == "land" else self.after_takeoff)

    def send_goal(self, waypoints):
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints

        self.waypoint_client.wait_for_server()
        self._send_goal_future = self.waypoint_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.waypoints_done_callback)

    def after_takeoff(self, future):
        goal_handle = future.result()
        if goal_handle.accepted and (not (self.waypoints is None)):
            self.get_logger().info('Takeoff accepted, navigating waypoints')
            self.send_goal(self.waypoints)
        else:
            self.get_logger().info('Takeoff rejected')
            self.waypoints = None

    def waypoints_done_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Navigation goal rejected')
            self.waypoints = None
            return

        self.get_logger().info('Navigation goal accepted, waiting for the result')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.handle_navigation_result)

    def handle_navigation_result(self, future):
        result = future.result().result
        if result:
            self.get_logger().info("Navigation successfully completed, sending land command")
            self.send_takeoff_or_land("land")
            self.waypoints = None
        else:
            self.get_logger().info("Navigation failed")
            self.waypoints = None

    def handle_land_response(self, future):
        result = future.result()
        if result.accepted:
            self.get_logger().info("Landing completed successfully")
            self.waypoints = None
        else:
            self.get_logger().info("Landing failed")
            self.waypoints = None

def create_pose(x, y, z):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = x
    pose.pose.position.y = y
    return pose

def main(args=None):
    rclpy.init(args=args)
    waypoint_follower_client = WaypointFollowerClient()
    rclpy.spin(waypoint_follower_client)
    waypoint_follower_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()