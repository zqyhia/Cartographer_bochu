/*
 * Copyright 2017 The Cartographer Authors
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

#include "cartographer/mapping/pose_extrapolator.h"

#include <algorithm>

#include "absl/memory/memory.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

// PoseExtrapolator构造函数：初始化三个参数：位姿队列的最大持续时间、IMU重力时间常数、时间戳和位姿的结构体（默认值）
PoseExtrapolator::PoseExtrapolator(const common::Duration pose_queue_duration, // 参数1：pose_queue_duration：位姿队列的最大持续时间
                                   double imu_gravity_time_constant) // 参数2：imu_gravity_time_constant：IMU重力时间常数
    : pose_queue_duration_(pose_queue_duration),
      gravity_time_constant_(imu_gravity_time_constant),
      cached_extrapolated_pose_{common::Time::min(), // cached_extrapolated_pose_：时间戳和位姿的结构体。初始化为最小时间戳和单位位姿。
                                transform::Rigid3d::Identity()} {}

// InitializeWithImu：创建一个基于初始 IMU 数据的位姿外推器实例
std::unique_ptr<PoseExtrapolator> PoseExtrapolator::InitializeWithImu(
    const common::Duration pose_queue_duration,
    const double imu_gravity_time_constant, const sensor::ImuData& imu_data) {
  // 构建一个姿态外推器（智能指针）
  auto extrapolator = absl::make_unique<PoseExtrapolator>(
      pose_queue_duration, imu_gravity_time_constant);
  // 将初始IMU数据加入时间序列队列
  extrapolator->AddImuData(imu_data);
  extrapolator->imu_tracker_ =
      absl::make_unique<ImuTracker>(imu_gravity_time_constant, imu_data.time); // ImuTracker作用：估计重力方向和IMU姿态
  // 更新IMU跟踪器状态
  extrapolator->imu_tracker_->AddImuLinearAccelerationObservation( // 线加速度观测
      imu_data.linear_acceleration);
  extrapolator->imu_tracker_->AddImuAngularVelocityObservation( // 角速度观测
      imu_data.angular_velocity);
  extrapolator->imu_tracker_->Advance(imu_data.time); // 使用没有用过的观测，更新跟踪器到指定时间
  // 添加初始位姿
  extrapolator->AddPose(
      imu_data.time,
      transform::Rigid3d::Rotation(extrapolator->imu_tracker_->orientation()));
  return extrapolator; // 返回姿态外推器指针
}

// GetLastPoseTime：获取位姿外推器中最新位姿时间戳。该方法返回当前存储在 timed_pose_queue_ 队列中最后一个位姿的时间戳。如果队列为空，则返回最小时间戳。
common::Time PoseExtrapolator::GetLastPoseTime() const {
  if (timed_pose_queue_.empty()) {
    return common::Time::min();
  }
  return timed_pose_queue_.back().time;
}

// GetLastExtrapolatedTime：获取最后外推时间的方法。来源于extrapolation_imu_tracker_
common::Time PoseExtrapolator::GetLastExtrapolatedTime() const {
  if (!extrapolation_imu_tracker_) { // AddPose函数里创建了extrapolation_imu_tracker_
    return common::Time::min();
  }
  return extrapolation_imu_tracker_->time();
}

// AddPose：处理新输入的位姿数据并更新整个外推系统的状态
void PoseExtrapolator::AddPose(const common::Time time,
                               const transform::Rigid3d& pose) {
  if (imu_tracker_ == nullptr) { // 只在第一次收到位姿时创建IMU跟踪器
    common::Time tracker_start = time;
    if (!imu_data_.empty()) {
      tracker_start = std::min(tracker_start, imu_data_.front().time); // 优先使用最早的IMU数据时间（如果存在），不一定是创建IMU追踪器的时间
    }
    imu_tracker_ =
        absl::make_unique<ImuTracker>(gravity_time_constant_, tracker_start); // 使用 重力时间常数 和 最早的IMU数据时间 创建 imu_tracker_
  }
  timed_pose_queue_.push_back(TimedPose{time, pose}); // 插入新位姿到位姿队列timed_pose_queue_
  while (timed_pose_queue_.size() > 2 && // 保持至少2个位姿，如果3个及以上就把最早的位姿信息弹出（速度计算需要）
         timed_pose_queue_[1].time <= time - pose_queue_duration_) { // 确保后读取的第二个位姿存在的时间 大于等于 pose_queue_duration_
    timed_pose_queue_.pop_front(); // 弹出最早的位姿
  }
  UpdateVelocitiesFromPoses(); // 速度估计更新
  AdvanceImuTracker(time, imu_tracker_.get()); // 更新IMU跟踪器状态，将没有使用的IMU数据观测添加到IMU跟踪器中 .get是std::unique_ptr的函数，用于获取指针但不转移所有权
  TrimImuData(); // IMU数据修剪，移除早于(最新位姿时间 - pose_queue_duration_)的数据
  TrimOdometryData(); // 里程计数据修剪
  // 跟踪器副本创建
  odometry_imu_tracker_ = absl::make_unique<ImuTracker>(*imu_tracker_); // odometry_imu_tracker_：专门处理里程计数据
  extrapolation_imu_tracker_ = absl::make_unique<ImuTracker>(*imu_tracker_); // extrapolation_imu_tracker_：用于位姿外推
}

// AddImuData：添加IMU数据并修剪数据
void PoseExtrapolator::AddImuData(const sensor::ImuData& imu_data) {
  CHECK(timed_pose_queue_.empty() ||
        imu_data.time >= timed_pose_queue_.back().time);
  imu_data_.push_back(imu_data);
  TrimImuData();
}

// AddOdometryData：添加里程计数据。添加后，修剪数据，一旦有充足的数据就会用于计算跟踪坐标系的角速度和线速度
void PoseExtrapolator::AddOdometryData(
    const sensor::OdometryData& odometry_data) {
  CHECK(timed_pose_queue_.empty() ||
        odometry_data.time >= timed_pose_queue_.back().time);
  odometry_data_.push_back(odometry_data); // 将新里程计数据加入队列
  TrimOdometryData(); // 修剪过期的里程计数据
  if (odometry_data_.size() < 2) { // 至少需要2个里程计数据才能计算速度
    return;
  }
  // TODO(whess): Improve by using more than just the last two odometry poses.
  // Compute extrapolation in the tracking frame.
  // {基于 里程计数据 计算 跟踪坐标系的角速度和线速度}
  const sensor::OdometryData& odometry_data_oldest = odometry_data_.front();
  const sensor::OdometryData& odometry_data_newest = odometry_data_.back();
  const double odometry_time_delta =
      common::ToSeconds(odometry_data_oldest.time - odometry_data_newest.time);
  const transform::Rigid3d odometry_pose_delta =
      odometry_data_newest.pose.inverse() * odometry_data_oldest.pose;
  // 【跟踪坐标系角速度计算】（角速度无所谓坐标系）
  angular_velocity_from_odometry_ =
      transform::RotationQuaternionToAngleAxisVector(
          odometry_pose_delta.rotation()) /
      odometry_time_delta;
  if (timed_pose_queue_.empty()) {
    return;
  }
  // 【跟踪坐标系线速度计算】
  // 1.初始线速度计算（里程计坐标系内）
  const Eigen::Vector3d
      linear_velocity_in_tracking_frame_at_newest_odometry_time = // 这个命名有点误导
          odometry_pose_delta.translation() / odometry_time_delta;
  // 2.获取当前姿态（跟踪坐标系）
  const Eigen::Quaterniond orientation_at_newest_odometry_time = 
      timed_pose_queue_.back().pose.rotation() *
      ExtrapolateRotation(odometry_data_newest.time, // ExtrapolateRotation(...)：使用IMU跟踪器估计这段时间内的姿态变化
                          odometry_imu_tracker_.get()); // AddPose中创建的IMU跟踪器副本
  // 3.将线速度从里程计坐标系转换到跟踪坐标系
  linear_velocity_from_odometry_ =
      orientation_at_newest_odometry_time *
      linear_velocity_in_tracking_frame_at_newest_odometry_time;
}

// ExtrapolatePose：姿态外推函数（最核心）。根据历史数据预测未来某一时刻的机器人位姿
transform::Rigid3d PoseExtrapolator::ExtrapolatePose(const common::Time time) {
  const TimedPose& newest_timed_pose = timed_pose_queue_.back(); // 最新已知位姿
  CHECK_GE(time, newest_timed_pose.time); // 确保预测时间不早于最新位姿时间
  if (cached_extrapolated_pose_.time != time) { // 避免对同一时间戳重复进行位姿外推计算
    // 外推平移信息
    const Eigen::Vector3d translation =
        ExtrapolateTranslation(time) + newest_timed_pose.pose.translation(); // 外推估计平移变化量+最新平移
    // 外推旋转信息
    const Eigen::Quaterniond rotation =
        newest_timed_pose.pose.rotation() *
        ExtrapolateRotation(time, extrapolation_imu_tracker_.get()); // 外推估计旋转变化量+最新旋转
    cached_extrapolated_pose_ =
        TimedPose{time, transform::Rigid3d{translation, rotation}}; // cached_extrapolated_pose_ 存储最后一次计算结果 TimedPose结构体里一个time，一个pose
  }
  return cached_extrapolated_pose_.pose; // 返回外推的位姿
}

// EstimateGravityOrientation：估计重力方向
Eigen::Quaterniond PoseExtrapolator::EstimateGravityOrientation(
    const common::Time time) {
  ImuTracker imu_tracker = *imu_tracker_; // 复制当前的 IMU 跟踪器状态
  AdvanceImuTracker(time, &imu_tracker); // 将跟踪器状态推进到指定时间
  return imu_tracker.orientation(); // 返回该时间点的重力方向估计（通过四元数表示），应该表示传感器坐标系->世界坐标系的变换
}

// UpdateVelocitiesFromPoses：基于位姿信息计算运动速度。基于位姿队列中的历史位姿（跟踪坐标系）来估计当前的线速度和角速度（跟踪坐标系）
void PoseExtrapolator::UpdateVelocitiesFromPoses() {
  if (timed_pose_queue_.size() < 2) {
    // 需要2个位姿来更新速度
    return;
  }
  CHECK(!timed_pose_queue_.empty()); // 确保队列不是空的
  const TimedPose& newest_timed_pose = timed_pose_queue_.back(); // newest_timed_pose：最新位姿
  const auto newest_time = newest_timed_pose.time; // newest_time：最新位姿的时间
  const TimedPose& oldest_timed_pose = timed_pose_queue_.front(); // oldest_timed_pose：最早位姿
  const auto oldest_time = oldest_timed_pose.time; // oldest_time：最早位姿时间
  const double queue_delta = common::ToSeconds(newest_time - oldest_time); // queue_delta：最大位姿时间差（单位：秒）
  if (queue_delta < common::ToSeconds(pose_queue_duration_)) { // 如果时间差<pose_queue_duration_，不进行速度估计，WARN并返回
    LOG(WARNING) << "Queue too short for velocity estimation. Queue duration: "
                 << queue_delta << " s";
    return;
  }
  const transform::Rigid3d& newest_pose = newest_timed_pose.pose; // newest_pose：最新位姿
  const transform::Rigid3d& oldest_pose = oldest_timed_pose.pose; // oldest_pose：最早位姿
  // 计算平移速度，通过translation转换，再使用Eigen库计算
  linear_velocity_from_poses_ = 
      (newest_pose.translation() - oldest_pose.translation()) / queue_delta;
  // 计算旋转角速度，通过rotation转换，再使用Eigen库计算四元数
  angular_velocity_from_poses_ = 
      transform::RotationQuaternionToAngleAxisVector(
          oldest_pose.rotation().inverse() * newest_pose.rotation()) /
      queue_delta;
}

// TrimImuData：修剪IMU数据，如果第二个IMU数据早于最新的位姿，丢弃最早的IMU数据，至少保存1个IMU数据
void PoseExtrapolator::TrimImuData() {
  while (imu_data_.size() > 1 && !timed_pose_queue_.empty() &&
         imu_data_[1].time <= timed_pose_queue_.back().time) {
    imu_data_.pop_front();
  }
}

// TrimOdometryData：修剪里程计数据，和IMU差不多，但是至少保存2个里程计数据
void PoseExtrapolator::TrimOdometryData() {
  while (odometry_data_.size() > 2 && !timed_pose_queue_.empty() &&
         odometry_data_[1].time <= timed_pose_queue_.back().time) {
    odometry_data_.pop_front();
  }
}

// AdvanceImuTracker：利用IMU数据将 指定的IMU跟踪器 更新到 指定时间。按照时间顺序将没有用过的IMU观测，应用到IMU跟踪器更新中。 输入：时间、IMU跟踪器指针。
void PoseExtrapolator::AdvanceImuTracker(const common::Time time,
                                         ImuTracker* const imu_tracker) const {
  CHECK_GE(time, imu_tracker->time()); // 确保目标时间不早于跟踪器当前时间
  // 无有效IMU数据 的情况
  if (imu_data_.empty() || time < imu_data_.front().time) { // IMU 数据为空 或 设定目标时间早于第一个 IMU 数据
    // There is no IMU data until 'time', so we advance the ImuTracker and use
    // the angular velocities from poses and fake gravity to help 2D stability.
    imu_tracker->Advance(time); // 直接推进到目标时间
    imu_tracker->AddImuLinearAccelerationObservation(Eigen::Vector3d::UnitZ()); // 添加默认重力观测（Z 轴方向），在没有IMU数据的时候做的特殊处理
    imu_tracker->AddImuAngularVelocityObservation( // 添加角速度观测
        odometry_data_.size() < 2 ? angular_velocity_from_poses_ // 其次使用位姿估计的角速度（数据少的时候）
                                  : angular_velocity_from_odometry_); // 优先使用里程计数据（数据充足的时候）
    return;
  }
  // 跟踪器滞后于IMU数据 的情况——有新的IMU数据可以用于更新IMU跟踪器
  if (imu_tracker->time() < imu_data_.front().time) { // IMU跟踪器比最早的IMU数据还要早，即IMU跟踪器没有及时处理IMU数据
    // Advance to the beginning of 'imu_data_'.
    imu_tracker->Advance(imu_data_.front().time); // 更新IMU跟踪器时间为最早IMU数据时间
  }
  // 按时间顺序应用IMU观测
  auto it = std::lower_bound( // 使用 std::lower_bound 二分查找第一个不早于跟踪器当前时间的 IMU 数据
      imu_data_.begin(), imu_data_.end(), imu_tracker->time(), // imu_tracker->time()：查询跟踪器当前时间
      [](const sensor::ImuData& imu_data, const common::Time& time) {
        return imu_data.time < time;
      });
  while (it != imu_data_.end() && it->time < time) { // 遍历从当前位置到目标时间之前的所有 IMU 数据
    imu_tracker->Advance(it->time); // 推进跟踪器到数据时间点
    imu_tracker->AddImuLinearAccelerationObservation(it->linear_acceleration); // 添加线加速度观测
    imu_tracker->AddImuAngularVelocityObservation(it->angular_velocity); // 添加角速度观测
    ++it; // 下一个IMU数据
  }
  imu_tracker->Advance(time); // 最终推进到目标时间，处理可能存在的最后一段没有 IMU 数据的时间间隔
}

// ExtrapolateRotation：计算两个时间点之间相对旋转。输入：时间 IMU跟踪器   输出：主跟踪器（imu_tracker_）到参数跟踪器（推进到未来/当前时间）的旋转变化量。
// 由于旋转的叠加特性，需要时刻分析最新的姿态变化，因此用参数跟踪器临时推进到目标时间，再与主跟踪器最新状态比较，获得旋转变化量。
Eigen::Quaterniond PoseExtrapolator::ExtrapolateRotation(
    const common::Time time, ImuTracker* const imu_tracker) const {
  CHECK_GE(time, imu_tracker->time()); // 确保目标时间不早于参数跟踪器当前时间
  AdvanceImuTracker(time, imu_tracker); // 更新参数跟踪器，处理从当前时间到目标时间的所有IMU数据
  const Eigen::Quaterniond last_orientation = imu_tracker_->orientation(); // 读取主跟踪器​imu_tracker_​最新姿态角，主跟踪器能够维护最新位姿时刻的全局旋转状态
  return last_orientation.inverse() * imu_tracker->orientation(); // 返回 主跟踪器最新姿态 到 参数跟踪器推进后姿态 的 旋转变化量 ΔR = R_initial⁻¹ × R_final
}

// ExtrapolateTranslation：计算位置平移量。输入：时间   输出：平移量（速度*时间）
Eigen::Vector3d PoseExtrapolator::ExtrapolateTranslation(common::Time time) {
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  const double extrapolation_delta =
      common::ToSeconds(time - newest_timed_pose.time); // 目标时间 - 最新存储位姿时间
  if (odometry_data_.size() < 2) { // 速度源选择逻辑：根据里程计数据是否充足选择数据源
    return extrapolation_delta * linear_velocity_from_poses_; // 如果里程计数据不足，使用基于位姿变化计算的速度。linear_velocity_from_poses_通过UpdateVelocitiesFromPoses()计算
  }
  return extrapolation_delta * linear_velocity_from_odometry_; // 如果里程计数据充足，优先使用基于里程计计算的速度。linear_velocity_from_odometry_通过AddOdometryData()计算
}

// ExtrapolatePosesWithGravity：批量预测位姿（包括重力）并返回完整外推结果。输入：多个时间点   输出：打包好的位姿结果
PoseExtrapolator::ExtrapolationResult
PoseExtrapolator::ExtrapolatePosesWithGravity(
    const std::vector<common::Time>& times) {
  std::vector<transform::Rigid3f> poses; // 姿态数组poses，还是float类型
  for (auto it = times.begin(); it != std::prev(times.end()); ++it) {
    poses.push_back(ExtrapolatePose(*it).cast<float>()); // 对 除最后一个时间 进行遍历、预测，将预测结果转换为 Rigid3f（单精度浮点数）并存入向量
  }

  const Eigen::Vector3d current_velocity = odometry_data_.size() < 2 // 速度源选择逻辑：根据里程计数据是否充足选择数据源
                                               ? linear_velocity_from_poses_ // 位姿估计速度
                                               : linear_velocity_from_odometry_; // 里程计估计速度
  return ExtrapolationResult{poses, ExtrapolatePose(times.back()), // 返回打包后的位姿结果，包括：位姿、最后一个时间点的位姿、当前速度估计、最后一个时间点的重力方向估计
                             current_velocity,
                             EstimateGravityOrientation(times.back())};
}

}  // namespace mapping
}  // namespace cartographer
