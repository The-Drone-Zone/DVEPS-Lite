from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='camera_scan_pkg',
        #     namespace='camera_scan_pkg', #namespace does not matter i believe
        #     executable='img_pub_node',
        #     name='camera_pub',
        #     prefix='gnome-terminal --'
        # ),
        # Node(
        #     package='camera_scan_pkg',
        #     namespace='camera_scan_pkg',
        #     executable='img_analysis_node',
        #     name='camera_analysis',
        #     prefix='gnome-terminal --',
        #     output='log',
        #     arguments=['--ros-args', '--log-level', 'info']
        # ),
        Node(
            package='drone_commander_pkg',
            namespace='drone_commander_pkg',
            executable='drone_commander',
            name='drone_commander',
            prefix='gnome-terminal --',
            output='log',
            arguments=['--ros-args', '--log-level', 'info'],
            remappings=[
                ('DecisionController/command', '/decision_pkg/DecisionController/command')
            ]
        ),
        Node(
            package='decision_pkg',
            namespace='decision_pkg',
            executable='decision_node',
            name='decision_node',
            prefix='gnome-terminal --',
            output='log',
            arguments=['--ros-args', '--log-level', 'info'],
            remappings=[
                ('output_obstacles', '/camera_scan_pkg/output_obstacles'),
                ('Lidar/analysis', '/lidar_scan_pkg/Lidar/analysis'),
                ('DecisionController/command_ack', '/drone_commander_pkg/DecisionController/command_ack')
            ]
        ),
        Node(
            package='lidar_scan_pkg',
            namespace='lidar_scan_pkg',
            executable='lidar_analysis_node',
            name='lidar_analysis_node',
            prefix='gnome-terminal --',
            output='log',
            arguments=['--ros-args', '--log-level', 'info']
        ),
        Node(
            package='lidar_scan_pkg',
            namespace='lidar_scan_pkg',
            executable='lidar_pub_node',
            name='lidar_pub_node',
            prefix='gnome-terminal --'
        ),
        Node(
            package='lidar_scan_pkg',
            namespace='lidar_scan_pkg',
            executable='obstacle_distance_node',
            name='obstacle_distance_node',
            prefix='gnome-terminal --'
        )
        # Node(
        #     package='rqt_image_view',
        #     namespace='rqt_image_view',
        #     executable='rqt_image_view',
        #     name='rqt_image_view',
        #     prefix='gnome-terminal --'
        # )
        # TimerAction(
        #     period=3.0,  # wait 3 seconds
        #     actions=[
        #         Node(
        #             package='decision_pkg',
        #             namespace='decision_pkg',
        #             executable='decision_node',
        #             name='decision_node',
        #             prefix='gnome-terminal --',
        #             output='screen',
        #             remappings=[
        #                 ('output_obstacles', '/camera_scan_pkg/output_obstacles'),
        #                 ('Lidar/analysis', '/custom_msg_pkg/Lidar/analysis'),
        #             ]
        #         )
        #     ]
        # )
    ])