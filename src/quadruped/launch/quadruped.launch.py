from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node  
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # 手柄驱动节点（修正包名）
        Node(
            package='joy',
            executable='joy_node',
            name='quadruped_joy',
            parameters=[{
                    'dev': '/dev/input/js0',
                    'deadzone': 0.12,
                    'autorepeat_rate': 15.0,  # 从30Hz降为15Hz
                    'coalesce_interval': 0.03
            }]
        ),
        
        Node(
            package='quadruped',
            executable='main_controller',
            name='main_controller',
            parameters=[{
                'control_frequency': 50.0,
                'port': '/dev/ttyROBOT',  # 保持符号链接路径
                'baud': 921600
            }]
        )
    ])