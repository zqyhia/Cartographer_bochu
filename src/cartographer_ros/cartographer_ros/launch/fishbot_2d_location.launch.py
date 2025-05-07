"""
  Copyright 2018 The Cartographer Authors
  Copyright 2022 Wyca Robotics (for the ros2 conversion)

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
"""

from launch import LaunchDescription # 定义Launch文件的主类 generate_launch_description()来源
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription # 声明参数 包含其他launch文件
from launch.conditions import IfCondition, UnlessCondition # 导入条件类
from launch.substitutions import LaunchConfiguration # 用launch参数替换节点参数
import launch_ros.actions
from launch_ros.actions import Node, SetRemap # 用于启动ROS2节点 用于重命名话题 
from launch_ros.substitutions import FindPackageShare # 用于查找ROS2包的共享目录路径
from launch.launch_description_sources import PythonLaunchDescriptionSource # 加载Python Launch文件

def generate_launch_description():

    ## ***** 声明参数 *****
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value = 'False') # 声明'use_sim_time'参数为False
    configuration_basename_arg = DeclareLaunchArgument('configuration_basename', default_value = 'fishbot_2d_location.lua') # 
    map_yaml_arg = DeclareLaunchArgument('map_yaml', default_value='/home/zqyhia/maps/fishbot0410.yaml', description='Map: .yaml location')

    # cartographer节点启动：功能包cartographer_ros，节点cartographer_node
    # 负责执行 SLAM 的主要功能，发布机器人的位姿（/tf 和 /tf_static）、子地图（/submap_list）、轨迹（/trajectory_list）
    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}], # cartographer_node需要的运行参数
        arguments = [ # 直接传给节点可执行文件的命令行参数，包括 参数设置文件夹 和 参数设置文件
            '-configuration_directory', FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files',
            '-configuration_basename', LaunchConfiguration('configuration_basename')],
        output = 'screen'
        )

    # 运行nav2_map_server包下的map_server节点，用于 加载静态地图   
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map_yaml'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
    )

    # 生命周期管理，cartographer_node默认不支持 Nav2 的生命周期管理，因此只对map_server做生命周期管理
    lifecycle_manager_node = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            emulate_tty=True, 
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server']}],
    )


    return LaunchDescription([
        use_sim_time_arg,
        configuration_basename_arg,
        map_yaml_arg,
        cartographer_node,
        map_server_node,
        lifecycle_manager_node
    ])
