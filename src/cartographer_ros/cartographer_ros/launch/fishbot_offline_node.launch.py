from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import Shutdown

def generate_launch_description():

    ## ***** Launch arguments *****
    bag_filenames_arg = DeclareLaunchArgument('bag_filenames')
    no_rviz_arg = DeclareLaunchArgument('no_rviz')
    rviz_config_arg = DeclareLaunchArgument('rviz_config')
    configuration_directory_arg = DeclareLaunchArgument('configuration_directory')
    configuration_basenames_arg = DeclareLaunchArgument('configuration_basenames')
    # urdf_filenames_arg = DeclareLaunchArgument('urdf_filenames')

    ## ***** Nodes *****
    rviz_node = Node(
        package = 'rviz2',
        executable = 'rviz2',
        on_exit = Shutdown(),
        arguments = ['-d', LaunchConfiguration('rviz_config')],
        parameters = [{'use_sim_time': True}],
        condition = UnlessCondition(LaunchConfiguration('no_rviz')) # no_rviz=false时执行
    )

    cartographer_occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_occupancy_grid_node',
        parameters = [
            {'use_sim_time': True},
            {'resolution': 0.05}],
        )

    cartographer_offline_node_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_offline_node',
        parameters = [{'use_sim_time': True}],
        arguments = [
            '-configuration_directory', LaunchConfiguration('configuration_directory'),
            '-configuration_basenames', LaunchConfiguration('configuration_basenames'),
            #'-urdf_filenames', LaunchConfiguration('urdf_filenames'),
            '-bag_filenames', LaunchConfiguration('bag_filenames')],
        output = 'screen'
        )


    return LaunchDescription([
        # Launch arguments
        bag_filenames_arg,
        no_rviz_arg,
        rviz_config_arg,
        configuration_directory_arg,
        configuration_basenames_arg,
        # urdf_filenames_arg,

        # Nodes
        rviz_node,
        cartographer_occupancy_grid_node,
        cartographer_offline_node_node,
    ])
