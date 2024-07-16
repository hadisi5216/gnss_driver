from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    gnss_data_publisher = Node(
        package="gnss_driver",
        executable="gnss_driver_node",
        name='gnss_driver_node',
        output='screen',
        parameters=[
            {}
        ]
    )
    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription(
        [gnss_data_publisher])
    # 返回让ROS2根据launch描述执行节点
    return launch_description
