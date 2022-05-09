from launch import LaunchDescription
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    return LaunchDescription([
        LifecycleNode(
            package='mir_object_recognition',
            executable='mmor',
            name='mmor',
            namespace='',
            output='screen',
            remappings=[
                ('input_image_topic', '/camera/color/image_raw'),
                ('input_cloud_topic', '/camera/depth/color/points'),
            ]
        ),
    ])