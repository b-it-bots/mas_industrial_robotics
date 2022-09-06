from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    model_config = os.path.join(
    get_package_share_directory('mir_rgb_object_recognition_models'),
    'common',
    'models',
    'yolov5',
    'atwork_realdata_combined/'
    )

    rgb_config = os.path.join(
    get_package_share_directory('mir_object_recognition'),
    'ros',
    'config',
    'rgb_classifier_config.yaml'
    )

    yolov5_config = os.path.join(
    get_package_share_directory('mir_recognizer_scripts'),
    'config',
    'data_for_detect.yaml'
    )

    return LaunchDescription([
        Node(
            package='mir_recognizer_scripts',
            namespace='',
            executable='rgb_recognizer',
            parameters=[
                {'~model_dir': model_config},
                {'~rgb_config_file': rgb_config},
                {'~yolo_data_config_file': yolov5_config}
            ],
            name='rgb_recognizer',
        )
    ])