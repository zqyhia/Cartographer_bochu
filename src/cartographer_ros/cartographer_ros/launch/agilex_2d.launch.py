import os
from  ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='False') # 隐式声明启动参数，LaunchConfiguration 的第二个参数 default 允许你为参数提供默认值，而不需要显式使用 DeclareLaunchArgument
    
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', # Cartographer设置文件目录：cartographer_ros/configuration_files
        default=os.path.join(get_package_share_directory('cartographer_ros') , 'configuration_files'))
    configuration_basename = LaunchConfiguration('configuration_basename', default='agilex.lua') # Cartographer设置文件名：wheeltec_2d.lua
    
    resolution = LaunchConfiguration('resolution', default='0.05') # 地图分辨率默认为0.05m
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='0.5') # 地图发布周期默认为0.5s

    return LaunchDescription([

        # 声明先前的参数
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),
        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),

        # 运行cartographer_ros包下的cartographer_node节点（默认不使用仿真时间）。通过arguments传入设置文件。将/odom和/imu重映射。
        # 负责执行 SLAM 的主要功能，发布机器人的位姿（/tf 和 /tf_static）、子地图（/submap_list）、轨迹（/trajectory_list）
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename],
            remappings=[
            ('odom','/odometry/filtered'),
            ('imu', 'imu_data'), # 需要订阅/imu话题，将/imu/data内容传递给/imu
            ]),
        
        # 运行cartographer_ros包下的cartographer_occupancy_grid_node节点（默认不使用仿真时间），通过arguments传入地图分辨率（默认0.05m）和地图发布周期（默认0.5s）
        # 负责将 cartographer_node 生成的地图转换为 ROS 的标准地图格式（OccupancyGrid），发布 OccupancyGrid 格式的地图（/map）
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',

            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec]),
 
    ])
