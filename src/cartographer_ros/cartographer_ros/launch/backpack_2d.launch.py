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

    ## ***** 文件路径 ******
    pkg_share = FindPackageShare('cartographer_ros').find('cartographer_ros') # cartographer_ros包的共享目录路径
    urdf_dir = os.path.join(pkg_share, 'urdf') # urdf文件夹的路径（在share下）
    urdf_file = os.path.join(urdf_dir, 'backpack_2d.urdf') # urdf文件的路径（在share下）
    with open(urdf_file, 'r') as infp: # with上下文管理器，确保文件读取结束后自动关闭，'r'为读模式
        robot_desc = infp.read() # 打开backpack_2d.urdf文件并读取内容，存储在robot_desc变量中

    ## ***** 节点 *****
    # 机器人状态发布节点
    robot_state_publisher_node = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        parameters=[
            {'robot_description': robot_desc}, # 机器人描述传给状态发布节点
            {'use_sim_time': LaunchConfiguration('use_sim_time')}], # 是否使用模拟时间传给状态发布节点
        output = 'screen'
        )

    # cartographer节点启动：功能包cartographer_ros，节点cartographer_node
    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}], # cartographer_node需要的运行参数
        arguments = [ # 直接传给节点可执行文件的命令行参数，包括 参数设置文件夹 和 参数设置文件
            '-configuration_directory', FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files',
            '-configuration_basename', 'backpack_2d.lua'],
        remappings = [
            ('echoes', 'horizontal_laser_2d')], # 话题重映射，将节点内部的'echoes'话题重映射为'horizontal_laser_2d'
        output = 'screen'
        )

    # cartographer栅格地图节点：功能包cartographer_ros，节点cartographer_occupancy_grid_node
    cartographer_occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_occupancy_grid_node',
        parameters = [
            {'use_sim_time': True}, # 是否使用仿真时间
            {'resolution': 0.05}, # 栅格地图分辨率
            {'publish_period_sec': 1}, # 发布周期
            ],
        )

    return LaunchDescription([
        use_sim_time_arg,
        # Nodes
        robot_state_publisher_node,
        cartographer_node,
        cartographer_occupancy_grid_node,
    ])
