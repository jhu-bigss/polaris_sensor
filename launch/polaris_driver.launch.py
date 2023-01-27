from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    ld = LaunchDescription()
    polaris_package_path = get_package_share_path('polaris_sensor')
    
    polaris_ros_node = Node(
        package = 'polaris_sensor',
        executable = 'polaris_sensor_node',
        parameters = [
            {"roms": os.path.join(polaris_package_path, 'rom/LCSR-optical-tracker-body.rom')},
            {"port": "/dev/ttyUSB0"}
        ],
        
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    ld.add_action(polaris_ros_node)
    ld.add_action(rviz_node)

    return ld
