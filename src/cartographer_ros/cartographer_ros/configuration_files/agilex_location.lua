include "map_builder.lua"
include "trajectory_builder.lua"

-- 基本配置
options = {
    map_builder = MAP_BUILDER,
    trajectory_builder = TRAJECTORY_BUILDER,
    map_frame = "map",
    tracking_frame = "base_link", 	-- "imu_link",gyro_link
    published_frame = "base_link", 	-- "odom",
    odom_frame = "odom_carto", 		-- "odom",
    provide_odom_frame = true, -- 提供里程计坐标系
    publish_frame_projected_to_2d = true, -- 是否将发布的坐标系投影到 2D 平面
    use_odometry = true,
    use_nav_sat = false,
    use_landmarks = false,
    num_laser_scans = 1,
    num_multi_echo_laser_scans = 0,
    num_subdivisions_per_laser_scan = 1, -- 每帧激光雷达扫描的细分数量为 1
    num_point_clouds = 0,
    lookup_transform_timeout_sec = 0.2, -- 查找坐标变换的超时时间为 0.2 秒
    submap_publish_period_sec = 0.3, -- 子地图发布的周期为 0.3 秒
    pose_publish_period_sec = 5e-3, -- 位姿发布的周期为 5e-3 秒
    trajectory_publish_period_sec = 30e-3, -- 轨迹发布的周期为 30e-3 秒
    rangefinder_sampling_ratio = 1., -- 激光雷达数据的采样比例为 1
    odometry_sampling_ratio = 0.1, -- 里程计数据的采样比例为 0.1
    fixed_frame_pose_sampling_ratio = 1., -- 固定坐标系位姿数据的采样比例为 1
    imu_sampling_ratio = 1., -- IMU 数据的采样比例为 1
    landmarks_sampling_ratio = 1., -- 地标数据的采样比例为 1
    use_pose_extrapolator = true, -- 姿态外推
}

-- 地图构建器配置
MAP_BUILDER.use_trajectory_builder_2d = true -- 使用 2D 轨迹构建器
MAP_BUILDER.num_background_threads = 4 -- 地图构建器的后台线程数量为 4

-- 2D 轨迹构建器配置
TRAJECTORY_BUILDER_2D.use_imu_data = true -- 是否使用 IMU 数据（false）
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true -- 是否使用在线相关扫描匹配

TRAJECTORY_BUILDER_2D.min_range = 0.2 -- 激光雷达的最小有效距离为 0.15 米
TRAJECTORY_BUILDER_2D.max_range = 20.0 -- 激光雷达的最大有效距离为 5.0 米
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 19.0 -- 缺失数据的射线长度为 3 米
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1 -- 累积的激光雷达数据帧数为 1
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 50 -- 每个子地图的激光雷达数据帧数为 45

TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(20) -- 如果如果两次激光雷达扫描之间的角度变化小于该值，则可能丢弃当前扫描。
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.5 -- 如果两次扫描之间的平移距离小于该值，则可能丢弃当前扫描。
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.1 -- 如果两次扫描的时间间隔小于该值，则可能丢弃当前扫描。
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025 -- 体素滤波器的大小为 0.05 米

TRAJECTORY_BUILDER_2D.pose_extrapolator.constant_velocity.imu_gravity_time_constant = 5. -- 姿态外推的IMU重力时间常数（数值越小能让系统更快适应重力变化，适用于动态高、IMU精度高的场景） 10.

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 50 -- Ceres 求解器的最大迭代次数为 20
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 10.0 -- 控制 ​激光点云中的占据点（如墙壁、障碍物）在匹配时的权重。 1.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 21. -- Ceres 扫描匹配器的平移权重为 10
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 10. -- 旋转惩罚权重。越高说明越相信运动模型。40

TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.) -- 扫描匹配时允许的​​最大角度搜索范围，设置太小出错了可能不能校正回来，默认20
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.2 -- 扫描匹配时允许的​​最大位移搜索范围​​，设置太小出错了可能不能校正回来，默认0.1

-- 位姿图优化配置
POSE_GRAPH.optimize_every_n_nodes = 15 -- 每n个节点优化一次位姿图（纯定位模式需要修改） 90

POSE_GRAPH.constraint_builder.min_score = 0.6 -- 约束构建器的最小得分 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.65 -- 全局定位的最小得分 0.7

POSE_GRAPH.constraint_builder.sampling_ratio = 0.05 -- 约束构建器的采样比例，每隔多少节点会尝试与子地图或其他节点建立约束 0.3
POSE_GRAPH.global_sampling_ratio = 0.05 -- 全局采样的比例，每隔多少节点会尝试与历史所有子地图进行回环匹配 0.003

POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 2.5e5 -- 控制​​回环检测​​时对​​平移​​​​误差的约束强度 1e5
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 2.5e5 -- 控制​​回环检测​​时对​​旋转​​​​误差的约束强度 1e5

POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 8e4 -- 控制​​局部SLAM位姿​在全局优化中的位移权重 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 8e4 -- 控制​​局部SLAM位姿​在全局优化中的旋转权重 1e5
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e4 -- 控制​​里程计数据​​在全局优化中的位移权重 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight= 1e4 -- 控制​​里程计数据​​在全局优化中的旋转权重 1e5

POSE_GRAPH.optimization_problem.acceleration_weight = 1.5e2 -- 约束相邻位姿间的​​加速度连续性，用于平滑轨迹并抑制不合理的瞬时加速度变化（数值越大，轨迹越平滑） 1.1e2
POSE_GRAPH.optimization_problem.rotation_weight = 2e4 -- 约束相邻位姿间的​​旋转角速度连续性​​（单位：rad²/s²），防止不合理的旋转突变（数值越大，对旋转漂移抑制越强） 1.6e4

POSE_GRAPH.optimization_problem.huber_scale = 6 -- Huber 损失函数的尺度参数,控制在图优化过程中何时从平方误差切换为线性误差（越大越适合高精度传感器，越低鲁棒性越强） 1e1

POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 80 -- 全局优化迭代次数 50

POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 8. -- 快速相关性扫描匹配器的匹配平移范围（越大回环匹配的范围也越大） 7.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(25.) -- 快速相关性扫描匹配器的匹配旋转范围（越大回环匹配的范围也越大） math.rad(30.)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7 -- 扫描匹配时使用的分支限界搜索深度 7.

-- POSE_GRAPH.max_num_final_iterations = 500 -- 最终的全局优化数 200

-- 新的纯定位模式配置（见map_builder.cc: MaybeAddPureLocalizationTrimmer），仅定位不建图
TRAJECTORY_BUILDER.pure_localization_trimmer = {
    max_submaps_to_keep = 3 -- 在定位过程中保留多少个子地图。越小，内存占用越低；越大，定位可能更稳定
}

return options
