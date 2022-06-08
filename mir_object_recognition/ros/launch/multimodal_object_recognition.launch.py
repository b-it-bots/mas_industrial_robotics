from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('mir_object_recognition'),
        'config',
        'scene_segmentation_constraints.yaml'
        )
        
    node = LifecycleNode(
            package='mir_object_recognition', 
            executable='mmor',
            namespace='',
            name='mmor', 
            parameters = [config],
            output='screen',
            remappings=[
                ('input_image_topic', '/camera/color/image_raw'),
                ('input_cloud_topic', '/camera/depth/color/points'),
            ])
    
    ld.add_action(node)
    return ld
