# launch file to view the youbot in rviz

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition
import os

def generate_launch_description():

    # joint state gui publisher argument
    declare_joint_state_gui = DeclareLaunchArgument(
        'joint_state_gui',
        default_value='true',
        description='Launch joint state gui publisher')
    
    robot_name = os.environ['ROBOT']

    # planning_context
    youbot_xacro_file = os.path.join(get_package_share_directory('mir_hardware_config'), robot_name, 'urdf',
                                     'robot.urdf.xacro')
    robot_description_config = Command(
        [FindExecutable(name='xacro'), ' ', youbot_xacro_file])

    robot_description = {'robot_description': ParameterValue(robot_description_config, value_type=str)}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    # launch joint state publisher with gui based on argument
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='both',
        condition=IfCondition(LaunchConfiguration('joint_state_gui'))
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='both',
        condition=UnlessCondition(LaunchConfiguration('joint_state_gui'))
    )

    return LaunchDescription([
        declare_joint_state_gui,
        robot_state_publisher,
        joint_state_publisher,
        joint_state_publisher_gui
    ])