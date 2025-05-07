include "map_builder.lua"
include "trajectory_builder.lua"

-- 基本配置
options = {
    map_builder = MAP_BUILDER,
    trajectory_builder = TRAJECTORY_BUILDER,
    map_frame = "map",
    tracking_frame = "base_footprint", 	-- "imu_link",gyro_link
    published_frame = "base_footprint", 	-- "odom",
    odom_frame = "odom_combined", 		-- "odom",
    provide_odom_frame = false, -- 不提供里程计坐标系
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
}

-- 地图构建器配置
MAP_BUILDER.use_trajectory_builder_2d = true -- 使用 2D 轨迹构建器
MAP_BUILDER.num_background_threads = 4 -- 地图构建器的后台线程数量为 4

-- 2D 轨迹构建器配置
TRAJECTORY_BUILDER_2D.min_range = 0.15 -- 激光雷达的最小有效距离为 0.15 米
TRAJECTORY_BUILDER_2D.max_range = 5.0 -- 激光雷达的最大有效距离为 5.0 米
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3. -- 缺失数据的射线长度为 3 米
TRAJECTORY_BUILDER_2D.use_imu_data = true -- 是否使用 IMU 数据（false）
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true -- 是否使用在线相关扫描匹配
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1) -- 运动滤波器的最大角度为 0.1 弧度
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 6e2 -- Ceres 扫描匹配器的平移权重为 2e2 10
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 40 -- Ceres 求解器的最大迭代次数为 20
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1 -- 累积的激光雷达数据帧数为 1
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05 -- 体素滤波器的大小为 0.05 米
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 45 -- 每个子地图的激光雷达数据帧数为 45
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 2.0 -- 控制 ​激光点云中的占据点（如墙壁、障碍物）在匹配时的权重。 1.
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 4e2 -- 旋转惩罚权重。越高说明越相信运动模型。40

-- 位姿图优化配置
POSE_GRAPH.constraint_builder.min_score = 0.65 -- 约束构建器的最小得分为 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7 -- 全局定位的最小得分为 0.7
POSE_GRAPH.global_sampling_ratio = 0.001 -- 全局采样的比例为 0.001
POSE_GRAPH.constraint_builder.sampling_ratio = 0.001 -- 约束构建器的采样比例为 0.001
POSE_GRAPH.optimize_every_n_nodes = 50 -- 每 50 个节点优化一次位姿图
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight= 1e1
-- POSE_GRAPH.optimization_problem.rotation_weight = 3e4 -- 旋转权重，越大越相信imu等传感器 1.6e4
-- POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 100 -- 全局优化迭代次数 50
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 8 -- 全局优化搜索深度 7
-- POSE_GRAPH.max_num_final_iterations = 500 -- 最终的全局优化数 200


return options
