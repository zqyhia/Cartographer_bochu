/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/mapping/internal/2d/local_trajectory_builder_2d.h"

#include <limits>
#include <memory>

#include "absl/memory/memory.h"
#include "cartographer/metrics/family_factory.h"
#include "cartographer/sensor/range_data.h"

namespace cartographer {
namespace mapping {

static auto* kLocalSlamLatencyMetric = metrics::Gauge::Null();
static auto* kLocalSlamRealTimeRatio = metrics::Gauge::Null();
static auto* kLocalSlamCpuRealTimeRatio = metrics::Gauge::Null();
static auto* kRealTimeCorrelativeScanMatcherScoreMetric =
    metrics::Histogram::Null();
static auto* kCeresScanMatcherCostMetric = metrics::Histogram::Null();
static auto* kScanMatcherResidualDistanceMetric = metrics::Histogram::Null();
static auto* kScanMatcherResidualAngleMetric = metrics::Histogram::Null();

// 构造函数，初始化成员变量
LocalTrajectoryBuilder2D::LocalTrajectoryBuilder2D(
    const proto::LocalTrajectoryBuilderOptions2D& options,
    const std::vector<std::string>& expected_range_sensor_ids)
    : options_(options),
      active_submaps_(options.submaps_options()),
      motion_filter_(options_.motion_filter_options()),
      real_time_correlative_scan_matcher_(
          options_.real_time_correlative_scan_matcher_options()),
      ceres_scan_matcher_(options_.ceres_scan_matcher_options()),
      range_data_collator_(expected_range_sensor_ids) {}

LocalTrajectoryBuilder2D::~LocalTrajectoryBuilder2D() {}

// TransformToGravityAlignedFrameAndFilter：处理激光雷达数据的关键预处理步骤，主要完成 ​​坐标系转换​​ 和 ​​数据滤波​​
sensor::RangeData
LocalTrajectoryBuilder2D::TransformToGravityAlignedFrameAndFilter(
    const transform::Rigid3f& transform_to_gravity_aligned_frame, // 输入1：transform_to_gravity_aligned_frame: 局部坐标系到重力对齐坐标系的变换矩阵
    const sensor::RangeData& range_data) const { // 输入2：range_data: 局部坐标系下的原始激光雷达数据（包含origin, returns, misses）   返回处理后的RangeData
  const sensor::RangeData cropped =
      sensor::CropRangeData(sensor::TransformRangeData(
                                range_data, transform_to_gravity_aligned_frame), // （1）坐标系变换
                            options_.min_z(), options_.max_z()); // （2）高度裁剪
  return sensor::RangeData{
      cropped.origin,
      sensor::VoxelFilter(cropped.returns, options_.voxel_filter_size()), // （3）体素滤波
      sensor::VoxelFilter(cropped.misses, options_.voxel_filter_size())};
}

// ScanMatch：扫描匹配实现，它采用 ​​两级匹配策略​​ 来优化位姿估计
std::unique_ptr<transform::Rigid2d> LocalTrajectoryBuilder2D::ScanMatch(
    const common::Time time, const transform::Rigid2d& pose_prediction, // 当前时间戳   预测位姿：重力对齐坐标系在局部坐标系下的位姿，并且还做了2D投影处理
    const sensor::PointCloud& filtered_gravity_aligned_point_cloud) { // 滤波后的点云
  // (1) 子图检查​：如果没有可匹配的子图，直接返回预测位姿
  if (active_submaps_.submaps().empty()) {
    return absl::make_unique<transform::Rigid2d>(pose_prediction);
  }
  std::shared_ptr<const Submap2D> matching_submap =
      active_submaps_.submaps().front(); // 匹配子图为子图管理中前一张子图
  // The online correlative scan matcher will refine the initial estimate for
  // the Ceres scan matcher.
  // (2) 在线相关匹配：会提高初始位姿估计的精度
  transform::Rigid2d initial_ceres_pose = pose_prediction;
  // 暴力搜索在预测位姿周围的候选位姿；计算每个候选位姿的点云与子图的相关性得分；选择得分最高的位姿作为Ceres优化的初始值
  if (options_.use_online_correlative_scan_matching()) {
    const double score = real_time_correlative_scan_matcher_.Match(
        pose_prediction, filtered_gravity_aligned_point_cloud, // 初始位姿   滤波后的点云
        *matching_submap->grid(), &initial_ceres_pose); // 匹配子图的概率栅格   输出优化后的初始位姿
    kRealTimeCorrelativeScanMatcherScoreMetric->Observe(score);
  }
  // (3) Ceres优化匹配
  auto pose_observation = absl::make_unique<transform::Rigid2d>();
  ceres::Solver::Summary summary;
  ceres_scan_matcher_.Match(pose_prediction.translation(), initial_ceres_pose,  // 预测的平移   Ceres用的初始位姿（来自最开始的预测 或者 在线相关匹配）
                            filtered_gravity_aligned_point_cloud, // 滤波后的点云
                            *matching_submap->grid(), pose_observation.get(), // 匹配子图的概率栅格   输出最终位姿
                            &summary); // 优化过程摘要
  // (4) 指标记录
  if (pose_observation) {
    kCeresScanMatcherCostMetric->Observe(summary.final_cost); // 最终代价
    const double residual_distance = // 平移残差
        (pose_observation->translation() - pose_prediction.translation())
            .norm();
    kScanMatcherResidualDistanceMetric->Observe(residual_distance);
    const double residual_angle = // 旋转残差
        std::abs(pose_observation->rotation().angle() -
                 pose_prediction.rotation().angle());
    kScanMatcherResidualAngleMetric->Observe(residual_angle);
  }
  return pose_observation; // 输出最终的位姿
}

// AddRangeData：局部 SLAM 流水线的核心入口，负责处理原始激光雷达数据并触发扫描匹配。
std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult>
LocalTrajectoryBuilder2D::AddRangeData(
    const std::string& sensor_id, // 输入：传感器ID
    const sensor::TimedPointCloudData& unsynchronized_data) { // 输入：带时间戳的原始点云数据    输出：匹配后的位姿和点云（累积足够数据时）
  // ​(1) 数据时间同步​
  auto synchronized_data =
      range_data_collator_.AddRangeData(sensor_id, unsynchronized_data); // 将当前帧与其它传感器数据（如IMU）时间对齐
  if (synchronized_data.ranges.empty()) {
    LOG(INFO) << "Range data collator filling buffer.";
    return nullptr;
  }

  // (2) 姿态外推器检初始化
  const common::Time& time = synchronized_data.time;
  // Initialize extrapolator now if we do not ever use an IMU.
  if (!options_.use_imu_data()) {
    InitializeExtrapolator(time);
  }

  if (extrapolator_ == nullptr) {
    // Until we've initialized the extrapolator with our first IMU message, we
    // cannot compute the orientation of the rangefinder.
    LOG(INFO) << "Extrapolator not yet initialized.";
    return nullptr;
  }

  // (3) 时间有效性验证
  CHECK(!synchronized_data.ranges.empty()); // 数据非空断言（CHECK）
  // TODO(gaschler): Check if this can strictly be 0.
  CHECK_LE(synchronized_data.ranges.back().point_time.time, 0.f); // 点云时间戳合理性（最后一个点时间 ≤ 0，相对时间表示）
  const common::Time time_first_point =
      time +
      common::FromSeconds(synchronized_data.ranges.front().point_time.time);
  if (time_first_point < extrapolator_->GetLastPoseTime()) { // 避免时间回退（首点时间应 ≥ 上次推测位姿时间）
    LOG(INFO) << "Extrapolator is still initializing.";
    return nullptr;
  }

  // (4) 逐点位姿推算
  std::vector<transform::Rigid3f> range_data_poses;
  range_data_poses.reserve(synchronized_data.ranges.size());
  bool warned = false;
  for (const auto& range : synchronized_data.ranges) {
    common::Time time_point = time + common::FromSeconds(range.point_time.time); // 计算每个点对应的绝对时间戳
    if (time_point < extrapolator_->GetLastExtrapolatedTime()) {
      if (!warned) {
        LOG(ERROR)
            << "Timestamp of individual range data point jumps backwards from "
            << extrapolator_->GetLastExtrapolatedTime() << " to " << time_point;
        warned = true;
      }
      time_point = extrapolator_->GetLastExtrapolatedTime();
    }
    range_data_poses.push_back(
        extrapolator_->ExtrapolatePose(time_point).cast<float>()); // 调用 ExtrapolatePose 获取该时刻的传感器在局部坐标系下的位姿
  }

  if (num_accumulated_ == 0) {
    // 'accumulated_range_data_.origin' is uninitialized until the last
    // accumulation.
    accumulated_range_data_ = sensor::RangeData{{}, {}, {}}; // 重置累积器
  }

  // Drop any returns below the minimum range and convert returns beyond the
  // maximum range into misses.
  for (size_t i = 0; i < synchronized_data.ranges.size(); ++i) { // 遍历时间同步后的激光雷达点云中的每一个点
    const sensor::TimedRangefinderPoint& hit = // 激光雷达在某个时刻测量到的单个点的原始数据
        synchronized_data.ranges[i].point_time;
    const Eigen::Vector3f origin_in_local = // 将原点坐标从 传感器坐标系 转换到 ​​局部地图坐标系​​（通过姿态外推后的位姿）
        range_data_poses[i] * // 上面得到的通过姿态外推后的位姿，左乘能够将传感器坐标系的点转换到局部坐标系下
        synchronized_data.origins.at(synchronized_data.ranges[i].origin_index); // 获取当前点对应的传感器原点索引，再取出该索引对应的原始传感器原点坐标（传感器坐标系）
    sensor::RangefinderPoint hit_in_local = // 将hit点坐标从 传感器坐标系 转换到 ​​局部地图坐标系
        range_data_poses[i] * sensor::ToRangefinderPoint(hit); // ToRangefinderPoint：将带时间戳的点转换为普通3D点（丢弃时间信息）
    const Eigen::Vector3f delta = hit_in_local.position - origin_in_local;
    const float range = delta.norm(); // 在局部坐标系下计算欧几里德距离
    // ​(5) 有效距离过滤​
    if (range >= options_.min_range()) {
      if (range <= options_.max_range()) {
        accumulated_range_data_.returns.push_back(hit_in_local); // 有效点
      } else { // 超出最大距离转为miss
        hit_in_local.position =
            origin_in_local +
            options_.missing_data_ray_length() / range * delta;
        accumulated_range_data_.misses.push_back(hit_in_local);
      }
    }
  }
  ++num_accumulated_;

  // (6) 累积触发条件​：积累N帧数据后统一处理
  if (num_accumulated_ >= options_.num_accumulated_range_data()) { // 检查是否累积了足够帧数的激光数据
    const common::Time current_sensor_time = synchronized_data.time;
    absl::optional<common::Duration> sensor_duration;
    if (last_sensor_time_.has_value()) {
      sensor_duration = current_sensor_time - last_sensor_time_.value(); // 计算当前帧与上一帧的时间差（sensor_duration）
    }
    last_sensor_time_ = current_sensor_time; // 更新last_sensor_time_为当前帧时间戳
    num_accumulated_ = 0; // 重置累积状态​
    const transform::Rigid3d gravity_alignment = transform::Rigid3d::Rotation( // 调用extrapolator_->EstimateGravityOrientation(time)，获取传感器坐标到重力对齐坐标系的旋转变换
        extrapolator_->EstimateGravityOrientation(time));
    // TODO(gaschler): This assumes that 'range_data_poses.back()' is at time
    // 'time'.
    accumulated_range_data_.origin = range_data_poses.back().translation(); // 设置累积数据的原点​。range_data_poses.back()：最后一帧激光数据的传感器位姿
    return AddAccumulatedRangeData( // 触发后续处理​，调用 AddAccumulatedRangeData
        time, // 当前激光帧的时间戳
        TransformToGravityAlignedFrameAndFilter( // 坐标系转换与滤波：最终输出
            gravity_alignment.cast<float>() * range_data_poses.back().inverse(), // 本行总体：局部坐标系到重力对齐坐标系的变换矩阵计算。gravity_alignment：从传感器坐标系到重力对齐坐标系的纯旋转矩阵​
            accumulated_range_data_), // range_data_poses.back()：最后一帧激光数据对应的传感器位姿（局部坐标系到传感器坐标系），inverse求逆得到 传感器坐标系到局部坐标系的变换。局部->传感器->重力对齐
        gravity_alignment, sensor_duration);
  }
  return nullptr;
}

// AddAccumulatedRangeData：处理重力对齐后的（积累了一定量的）激光雷达数据，进行扫描匹配并更新子地图
std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult>
LocalTrajectoryBuilder2D::AddAccumulatedRangeData(
    const common::Time time, // 时间戳
    const sensor::RangeData& gravity_aligned_range_data, // 重力对齐坐标系下的激光雷达数据（滤波后）
    const transform::Rigid3d& gravity_alignment, // 传感器坐标系到重力对齐坐标系的变换
    const absl::optional<common::Duration>& sensor_duration) { // 传感器持续时间
  // 1. 空数据检查​
  if (gravity_aligned_range_data.returns.empty()) {
    LOG(WARNING) << "Dropped empty horizontal range data.";
    return nullptr;
  }

  // Computes a gravity aligned pose prediction.
  // 2. 位姿预测计算
  const transform::Rigid3d non_gravity_aligned_pose_prediction =
      extrapolator_->ExtrapolatePose(time); // 局部坐标系下的位姿预测——传感器坐标系在局部坐标系下的位姿（传感器坐标系->局部坐标系的变换）
  const transform::Rigid2d pose_prediction = transform::Project2D( // 将3D位姿预测转换到重力对齐坐标系，然后投影到2D平面(因为这是2D SLAM，可以简化扫描匹配计算)
      non_gravity_aligned_pose_prediction * gravity_alignment.inverse()); // gravity_alignment：传感器坐标系->重力对齐坐标系。整体为：重力对齐坐标系->传感器坐标系->局部坐标系的变换

  // 3. 点云滤波处理
  const sensor::PointCloud& filtered_gravity_aligned_point_cloud = // 按照自适应体素滤波器设置对重力对齐点云数据进行滤波
      sensor::AdaptiveVoxelFilter(gravity_aligned_range_data.returns,
                                  options_.adaptive_voxel_filter_options());
  if (filtered_gravity_aligned_point_cloud.empty()) {
    return nullptr;
  }

  // local map frame <- gravity-aligned frame
  // 4. 扫描匹配
  std::unique_ptr<transform::Rigid2d> pose_estimate_2d = // pose_estimate_2d：2D位姿预测pose_prediction经扫描匹配后得到的更准确2D位姿估计
      ScanMatch(time, pose_prediction, filtered_gravity_aligned_point_cloud); // 时间 刚刚预测的位姿（重力对齐->局部） 过滤后的重力对齐系点云
  if (pose_estimate_2d == nullptr) {
    LOG(WARNING) << "Scan matching failed.";
    return nullptr;
  }
  const transform::Rigid3d pose_estimate = // 将2D位姿估计转换回3D位姿：乘以重力对齐变换，得到最终的局部坐标系下的位姿估计
      transform::Embed3D(*pose_estimate_2d) * gravity_alignment; // Embed3D：将2D位姿"嵌入"到3D空间，Z轴位移依然是0，为了保持与3D传感器数据坐标系的一致性
  extrapolator_->AddPose(time, pose_estimate); // 通过姿态外推的AddPose函数来更新姿态外推器的估计，用于改进未来的位姿预测（这里竟然也在维护姿态外推器！）

  // 5. 数据转换和子地图插入
  sensor::RangeData range_data_in_local = // 将重力对齐的激光数据转换到局部地图坐标系
      TransformRangeData(gravity_aligned_range_data,
                         transform::Embed3D(pose_estimate_2d->cast<float>())); // 这里实际上是一个 重力对齐->局部 的变换，只是经过了一系列优化
  std::unique_ptr<InsertionResult> insertion_result = InsertIntoSubmap( // 将处理后的数据插入到子地图中
      time, range_data_in_local, filtered_gravity_aligned_point_cloud,
      pose_estimate, gravity_alignment.rotation());

  // 6. 性能统计
  const auto wall_time = std::chrono::steady_clock::now();
  if (last_wall_time_.has_value()) {
    const auto wall_time_duration = wall_time - last_wall_time_.value();
    kLocalSlamLatencyMetric->Set(common::ToSeconds(wall_time_duration));
    if (sensor_duration.has_value()) {
      kLocalSlamRealTimeRatio->Set(common::ToSeconds(sensor_duration.value()) /
                                   common::ToSeconds(wall_time_duration));
    }
  }
  const double thread_cpu_time_seconds = common::GetThreadCpuTimeSeconds();
  if (last_thread_cpu_time_seconds_.has_value()) {
    const double thread_cpu_duration_seconds =
        thread_cpu_time_seconds - last_thread_cpu_time_seconds_.value();
    if (sensor_duration.has_value()) {
      kLocalSlamCpuRealTimeRatio->Set(
          common::ToSeconds(sensor_duration.value()) /
          thread_cpu_duration_seconds);
    }
  }
  last_wall_time_ = wall_time;
  last_thread_cpu_time_seconds_ = thread_cpu_time_seconds;
  // 7. 返回结果
  return absl::make_unique<MatchingResult>( // 构造MatchingResult并返回匹配结果
      MatchingResult{time, pose_estimate, std::move(range_data_in_local),
                     std::move(insertion_result)});
}

// InsertIntoSubmap：激光雷达数据（处理后）插入子地图
std::unique_ptr<LocalTrajectoryBuilder2D::InsertionResult> // 返回InsertionResult的智能指针（包含节点数据和子地图指针）
LocalTrajectoryBuilder2D::InsertIntoSubmap(
    const common::Time time, const sensor::RangeData& range_data_in_local,
    const sensor::PointCloud& filtered_gravity_aligned_point_cloud,
    const transform::Rigid3d& pose_estimate,
    const Eigen::Quaterniond& gravity_alignment) {
  // 1. 运动滤波检查​
  if (motion_filter_.IsSimilar(time, pose_estimate)) {
    return nullptr;
  }
  // 2. 子地图数据插入
  std::vector<std::shared_ptr<const Submap2D>> insertion_submaps =
      active_submaps_.InsertRangeData(range_data_in_local); // 使用了之前在submap2d附近看到的插入数据的函数InsertRangeData，后面会调用单张图的InsertRangeData，然后是插入器的Insert，如果是概率网格就会再调用CastRays
  // 3. 节点数据封装​
  return absl::make_unique<InsertionResult>(InsertionResult{
      std::make_shared<const TrajectoryNode::Data>(TrajectoryNode::Data{
          time,
          gravity_alignment,
          filtered_gravity_aligned_point_cloud,
          {},  // 'high_resolution_point_cloud' is only used in 3D.
          {},  // 'low_resolution_point_cloud' is only used in 3D.
          {},  // 'rotational_scan_matcher_histogram' is only used in 3D.
          pose_estimate}),
      std::move(insertion_submaps)});
}

// AddImuData：添加IMU数据，实际上是使用姿态外推器的AddImuData
void LocalTrajectoryBuilder2D::AddImuData(const sensor::ImuData& imu_data) {
  CHECK(options_.use_imu_data()) << "An unexpected IMU packet was added.";
  InitializeExtrapolator(imu_data.time);
  extrapolator_->AddImuData(imu_data);
}

// AddOdometryData：添加里程计数据，实际上是使用姿态外推器的AddOdometryData
void LocalTrajectoryBuilder2D::AddOdometryData(
    const sensor::OdometryData& odometry_data) {
  if (extrapolator_ == nullptr) {
    // Until we've initialized the extrapolator we cannot add odometry data.
    LOG(INFO) << "Extrapolator not yet initialized.";
    return;
  }
  extrapolator_->AddOdometryData(odometry_data);
}

// InitializeExtrapolator：初始化姿态外推器，都是姿态外推器那边的函数
void LocalTrajectoryBuilder2D::InitializeExtrapolator(const common::Time time) {
  if (extrapolator_ != nullptr) {
    return;
  }
  CHECK(!options_.pose_extrapolator_options().use_imu_based());
  // TODO(gaschler): Consider using InitializeWithImu as 3D does.
  extrapolator_ = absl::make_unique<PoseExtrapolator>(
      ::cartographer::common::FromSeconds(options_.pose_extrapolator_options()
                                              .constant_velocity()
                                              .pose_queue_duration()),
      options_.pose_extrapolator_options()
          .constant_velocity()
          .imu_gravity_time_constant());
  extrapolator_->AddPose(time, transform::Rigid3d::Identity());
}

// RegisterMetrics：全面的监控体系
void LocalTrajectoryBuilder2D::RegisterMetrics(
    metrics::FamilyFactory* family_factory) {
  auto* latency = family_factory->NewGaugeFamily(
      "mapping_2d_local_trajectory_builder_latency",
      "Duration from first incoming point cloud in accumulation to local slam "
      "result");
  kLocalSlamLatencyMetric = latency->Add({});
  auto* real_time_ratio = family_factory->NewGaugeFamily(
      "mapping_2d_local_trajectory_builder_real_time_ratio",
      "sensor duration / wall clock duration.");
  kLocalSlamRealTimeRatio = real_time_ratio->Add({});

  auto* cpu_real_time_ratio = family_factory->NewGaugeFamily(
      "mapping_2d_local_trajectory_builder_cpu_real_time_ratio",
      "sensor duration / cpu duration.");
  kLocalSlamCpuRealTimeRatio = cpu_real_time_ratio->Add({});
  auto score_boundaries = metrics::Histogram::FixedWidth(0.05, 20);
  auto* scores = family_factory->NewHistogramFamily(
      "mapping_2d_local_trajectory_builder_scores", "Local scan matcher scores",
      score_boundaries);
  kRealTimeCorrelativeScanMatcherScoreMetric =
      scores->Add({{"scan_matcher", "real_time_correlative"}});
  auto cost_boundaries = metrics::Histogram::ScaledPowersOf(2, 0.01, 100);
  auto* costs = family_factory->NewHistogramFamily(
      "mapping_2d_local_trajectory_builder_costs", "Local scan matcher costs",
      cost_boundaries);
  kCeresScanMatcherCostMetric = costs->Add({{"scan_matcher", "ceres"}});
  auto distance_boundaries = metrics::Histogram::ScaledPowersOf(2, 0.01, 10);
  auto* residuals = family_factory->NewHistogramFamily(
      "mapping_2d_local_trajectory_builder_residuals",
      "Local scan matcher residuals", distance_boundaries);
  kScanMatcherResidualDistanceMetric =
      residuals->Add({{"component", "distance"}});
  kScanMatcherResidualAngleMetric = residuals->Add({{"component", "angle"}});
}

}  // namespace mapping
}  // namespace cartographer
