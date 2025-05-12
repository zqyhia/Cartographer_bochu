include "map_builder.lua" -- 其中include：pose_graph.lua
include "trajectory_builder.lua" -- 其中include：trajectory_builder_2d.lua 和 trajectory_builder_3d.lua

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
  publish_tracked_pose = true -- 发布全局坐标系下的位姿
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 3.5
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.
-- TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true -- 在 ​前端（Local SLAM）​ 使用 ​在线相关性扫描匹配.显著提高 ​短时间内的位姿精度，尤其适合 ​高速运动 或 ​低精度里程计 的场景。
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

POSE_GRAPH.constraint_builder.min_score = 0.65 -- 控制一般回环检测的阈值（普通子图匹配） 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.85 -- 控制全局重定位（global localization）时的匹配阈值 0.7

POSE_GRAPH.constraint_builder.sampling_ratio = 0.05 -- 约束构建器的采样比例，每隔多少节点会尝试与子地图或其他节点建立约束 0.3
POSE_GRAPH.global_sampling_ratio = 0.01 -- 全局采样的比例，每隔多少节点会尝试与历史所有子地图进行回环匹配 0.003

POSE_GRAPH.optimize_every_n_nodes = 30 -- 90
-- TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90 -- 90
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 8 -- 回环检测搜索深度 7
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 100 -- 控制 ​全局优化（后端）​ 时 ​Ceres Solver 的最大迭代次数 50
POSE_GRAPH.optimization_problem.rotation_weight = 3e4 -- 控制 ​旋转约束 在优化中的权重。值越大：优化结果更倾向于保持旋转平滑（减少突变）。​值越小：旋转更自由，可能更适合高频旋转场景（如AGV）。1.6e4
-- POSE_GRAPH.optimization_problem.acceleration_weight = 1e1 -- 控制 ​IMU线性加速度 在优化中的权重。​值越大：优化结果更信任IMU的加速度测量（可能降低激光匹配的权重）。1.1e2
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight= 1e2

-- 新的纯定位模式配置（见map_builder.cc: MaybeAddPureLocalizationTrimmer），仅定位不建图
TRAJECTORY_BUILDER.pure_localization_trimmer = {
    max_submaps_to_keep = 3 -- 在定位过程中保留多少个子地图。越小，内存占用越低；越大，定位可能更稳定
}

return options
