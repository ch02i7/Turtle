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
            name='joy_node',
            parameters=[{
                'dev': '/dev/input/jsROBOT',  # 使用udev固定后的路径
                'deadzone': 0.05
            }]
        ),
        
        # 下位机通信节点（更新参数传递方式）
        Node(
            package='quadruped',
            executable='robot_comm',
            name='robot_comm',
            parameters=[{
                'port': '/dev/ttyROBOT',
                'baud': 921600,
            }]
        ),
        
        # 主控制节点（保持原有配置）
        Node(
            package='quadruped',
            executable='main_controller',
            name='main_controller',
            output='screen',
            parameters=[{
                'control_frequency': 50.0
            }]
        )
    ])