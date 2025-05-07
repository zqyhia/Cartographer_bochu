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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess # 声明参数 包含其他launch文件 执行外部命令
from launch.conditions import IfCondition, UnlessCondition # 导入条件类
from launch.substitutions import LaunchConfiguration # 用launch参数替换指定参数
from launch_ros.actions import Node, SetRemap # 用于启动ROS2节点 用于重命名话题 
from launch_ros.substitutions import FindPackageShare # 用于查找ROS2包的共享目录路径
from launch.launch_description_sources import PythonLaunchDescriptionSource # 加载Python Launch文件
from launch.actions import Shutdown # 在节点退出时关闭整个Launch文件

def generate_launch_description():

    ## ***** 节点 *****
    # 启动其他launch文件：cartographer_ros包的共享目录路径/launch/fishbot_2d.launch.py
    fishbot_2d_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(FindPackageShare('cartographer_ros').find('cartographer_ros') + '/launch/fishbot_2d_location.launch.py'),
        launch_arguments = {'use_sim_time': 'True'}.items() # 传递给包含launch文件的参数（那个文件use_sim_time默认为False）
        )

    # 启动RViz2可视化工具
    rviz_node = Node(
        package = 'rviz2',
        executable = 'rviz2',
        on_exit = Shutdown(), # 当RViz2退出时，关闭整个Launch文件
        arguments = ['-d', FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files/fishbot_location.rviz'], # '-d'用于指定RViz2的配置文件，使用cartographer_ros为RViz2 2d demo写好的配置文件
        parameters = [{'use_sim_time': True}], # 传递给节点的参数，表示 使用仿真时间
    )

    return LaunchDescription([
        # 节点
        fishbot_2d_launch,
        rviz_node,
    ])
