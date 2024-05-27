
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nodes = [
        # TELLO
        Node(
            package='tello',
            executable='tello',
            output='screen',
            namespace='/',
            name='tello',
            parameters=[
                {'connect_timeout': 10.0},
                {'tello_ip': '192.168.10.1'},
                {'tf_base': 'map'},
                {'tf_drone': 'drone'}
            ],
            remappings=[
                ('/image_raw', '/camera')
            ],
            respawn=True
        ),

        Node(
            package='tello_control',
            executable='tello_control',
            namespace='/',
            name='control',
            output='screen',
            respawn=False
        ),

        # TRANSFORMERS
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_camera',
            output='screen',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '1.0', 'drone', 'camera_depth_frame'],        
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_imu',
            output='screen',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '1.0', 'drone', 'imu'],        
        ),
        

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_drone_to_base_link',
            output='screen',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '1.0', 'drone', 'base_link'],        
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_odom_to_map',
            output='screen',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '1.0', 'map', 'odom'],        
        ),
        
        # RVIZ
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            namespace='/',
            name='rviz2',
            respawn=True,
            arguments=['-d', '/home/fabianfossbudal/repos/master-thesis-mono-repo/tello-ros2-driver/workspace/src/nav2_rviz.rviz']
        ),

        # # Localization node
        # Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='ekf_filter_node',
        #     output='screen',
        #     parameters=["/home/fabianfossbudal/repos/master-thesis-mono-repo/nav2_ws/src/config/ekf2.yaml", {'use_sim_time': False}]
        # ),

        # Waypoint action server
        Node(
            package='nav_through_waypoints',
            executable='nav_through_points',
            name='nav_through_points',
            output='screen',
        ),

        Node(
            package='video_saver',
            executable='capture_video_node',
            name='capture_video_node',
            output='screen',
        ),
    ]

    return LaunchDescription(nodes)