from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch_ros.event_handlers import OnStateTransition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart, OnProcessExit
import os

# 暂时是废案废案废案废案废案废案废案废案废案废案废案废案废案废案废案废案废案废案废案废案废案废案废案废案废案废案废案
# 因为Cartographer本身就不支持生命周期管理

def generate_launch_description():
    ## ***** 声明参数 *****
    # 是否启用仿真时间
    use_sim_time_arg = DeclareLaunchArgument( 
        'use_sim_time', 
        default_value='False',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # 是否启用间歇性建图
    mapping_active_arg = DeclareLaunchArgument( 
        'mapping_active',
        default_value='True',
        description='Whether to start with mapping active'
    )
    
    # 建图阶段持续时间(秒)
    mapping_duration_arg = DeclareLaunchArgument(
        'mapping_duration',
        default_value='10.0',
        description='Duration in seconds for each mapping phase'
    )
    
    # 暂停阶段持续时间(秒)
    pause_duration_arg = DeclareLaunchArgument(
        'pause_duration',
        default_value='5.0',
        description='Duration in seconds for each pause phase'
    )

    ## ***** 节点配置 *****
    # 修改为LifecycleNode
    cartographer_node = LifecycleNode(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        namespace='',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[
            '-configuration_directory', 
            FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files',
            '-configuration_basename', 
            'fishbot_2d.lua'
        ],
    )

    # 保留原有occupancy grid节点(非生命周期节点)
    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'resolution': 0.05}
        ],
        condition=IfCondition(LaunchConfiguration('mapping_active')) # 条件启动
    )

    ## ***** 生命周期控制逻辑 *****
    def get_cycle_control_script():
        # 生成bash控制脚本(使用f-string动态插入参数)
        return f"""
        while true; do # 无限循环
            # 激活建图阶段
            echo "[$(date)] Activating mapping for {LaunchConfiguration('mapping_duration')}s";
            ros2 lifecycle set /cartographer_node activate; # 激活节点
            sleep {LaunchConfiguration('mapping_duration')}; # 保持激活状态
            
            # 暂停阶段
            echo "[$(date)] Deactivating mapping for {LaunchConfiguration('pause_duration')}s";
            ros2 lifecycle set /cartographer_node deactivate; # 停用节点
            sleep {LaunchConfiguration('pause_duration')}; # 保持停用状态
        done
        """

    # 创建控制进程
    cycle_control = ExecuteProcess(
        cmd=['bash', '-c', get_cycle_control_script()], # 执行bash脚本
        shell=True, # 使用shell解释器
        output='screen',
        condition=IfCondition(LaunchConfiguration('mapping_active')) # 条件执行
    )

    ## ***** 自动配置处理 *****
    # 节点启动后自动配置(OnProcessStart事件)
    auto_configure = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=cartographer_node, # 目标节点
            on_start=[ # 节点启动后执行的动作序列：节点进程启动完成 → 触发 OnProcessStart 事件处理器
                TimerAction( # 延迟1秒执行
                    period=10.0,
                    actions=[
                        ExecuteProcess(
                            cmd=['ros2', 'lifecycle', 'set', '/cartographer_node', 'configure'],
                            shell=True,
                            output='screen'
                        )
                    ]
                )
            ]
        )
    )

    # 配置完成后延迟启动循环控制(OnStateTransition事件)
    auto_start_cycling = RegisterEventHandler(
        event_handler=OnStateTransition( # 监听状态转换事件
            target_lifecycle_node=cartographer_node, # 监控的目标节点
            start_state='configuring', # 起始状态（节点正在配置中）
            goal_state='inactive', # 目标状态（节点已完成配置）
            entities=[ # 状态转换成功后执行
                TimerAction( # 延迟2秒执行
                    period=2.0,
                    actions=[cycle_control] # 启动循环控制
                )
            ]
        )
    )

    return LaunchDescription([
        use_sim_time_arg,
        mapping_active_arg,
        mapping_duration_arg,
        pause_duration_arg,
        
        cartographer_node,
        cartographer_occupancy_grid_node,
        
        auto_configure,
        auto_start_cycling,
    ])