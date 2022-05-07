from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.LifecycleNode(
            package='mir_object_recognition',
            node_executable='multimodal_object_recognition',
            node_name='mmor',
            output='screen',
            remappings=[
                ('input_image_topic', '/camera/color/image_raw'),
                ('input_cloud_topic', '/camera/depth/color/points'),
            ]
        ),
    ])