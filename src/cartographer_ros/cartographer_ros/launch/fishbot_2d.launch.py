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
from launch_ros.actions import Node, SetRemap # 用于启动ROS2节点 用于重命名话题 
from launch_ros.substitutions import FindPackageShare # 用于查找ROS2包的共享目录路径
from launch.launch_description_sources import PythonLaunchDescriptionSource # 加载Python Launch文件
import os # 文件、目录、路径、环境变量操作

def generate_launch_description():

    ## ***** 声明参数 *****
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value = 'False') # 声明'use_sim_time'参数为False

    # cartographer节点启动：功能包cartographer_ros，节点cartographer_node
    # 负责执行 SLAM 的主要功能，发布机器人的位姿（/tf 和 /tf_static）、子地图（/submap_list）、轨迹（/trajectory_list）
    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}], # cartographer_node需要的运行参数
        arguments = [ # 直接传给节点可执行文件的命令行参数，包括 参数设置文件夹 和 参数设置文件
            '-configuration_directory', FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files',
            '-configuration_basename', 'fishbot_2d.lua'],
        output = 'screen'
        )

    # cartographer栅格地图节点：功能包cartographer_ros，节点cartographer_occupancy_grid_node
    # 负责将 cartographer_node 生成的地图转换为 ROS 的标准地图格式（OccupancyGrid），发布 OccupancyGrid 格式的地图（/map）
    cartographer_occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_occupancy_grid_node',
        parameters = [
            {'use_sim_time': True}, # 是否使用仿真时间
            {'resolution': 0.05}], # 栅格地图分辨率
        )

    return LaunchDescription([
        use_sim_time_arg,
        cartographer_node,
        cartographer_occupancy_grid_node,
    ])
