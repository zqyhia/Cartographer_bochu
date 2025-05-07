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

#include "cartographer/mapping/imu_tracker.h"

#include <cmath>
#include <limits>

#include "cartographer/common/math.h"
#include "cartographer/mapping/internal/eigen_quaterniond_from_two_vectors.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

// ImuTracker：构造函数，初始化所有关键状态变量：
ImuTracker::ImuTracker(const double imu_gravity_time_constant, // imu_gravity_time_constant：重力方向估计的时间常数（单位：秒），控制重力估计的平滑度。
                       const common::Time time) // time：初始化时间戳，表示跟踪器的起始时间
    : imu_gravity_time_constant_(imu_gravity_time_constant), // 重力估计时间常数
      time_(time), // 时间戳
      last_linear_acceleration_time_(common::Time::min()), // 最后加速度更新时间​：使用最小时间常量，表示没有收到加速度数据
      orientation_(Eigen::Quaterniond::Identity()), // 无旋转的初始姿态：单位四元数 [1, 0, 0, 0]，表示无旋转。orientation_代表从传感器坐标系到世界坐标系的旋转
      gravity_vector_(Eigen::Vector3d::UnitZ()), // ​​初始重力向量：​[0, 0, 1]，初始化为 +Z 以表示“重力反方向”
      imu_angular_velocity_(Eigen::Vector3d::Zero()) {} // 初始角速度​：[0, 0, 0]，表示无旋转。

// Advance：推进时间并更新姿态
void ImuTracker::Advance(const common::Time time) {
  CHECK_LE(time_, time);
  const double delta_t = common::ToSeconds(time - time_);
  const Eigen::Quaterniond rotation =
      transform::AngleAxisVectorToRotationQuaternion( // 从角轴到四元数的转换
          Eigen::Vector3d(imu_angular_velocity_ * delta_t)); // 角速度积分公式：Δθ = ω * Δt
  orientation_ = (orientation_ * rotation).normalized(); // 更新当前姿态
  gravity_vector_ = rotation.conjugate() * gravity_vector_; // 更新重力方向 原理：v_local = R⁻¹ * v_world，四元数共轭 rotation.conjugate() 等价于逆旋转 R⁻¹
  time_ = time;
}

// AddImuLinearAccelerationObservation：通过指数移动平均更新重力方向估计，并同步修正姿态四元数。输入​​：加速度计测量的线性加速度（含重力）
void ImuTracker::AddImuLinearAccelerationObservation(
    const Eigen::Vector3d& imu_linear_acceleration) {
  // Update the 'gravity_vector_' with an exponential moving average using the
  // 'imu_gravity_time_constant'.
  const double delta_t = // delta_t：时间差
      last_linear_acceleration_time_ > common::Time::min()
          ? common::ToSeconds(time_ - last_linear_acceleration_time_) // 如果不是第一次收到加速度数据，计算与上一次加速度更新的时间差
          : std::numeric_limits<double>::infinity(); // 如果是第一次收到加速度数据（last_linear_acceleration_time_未初始化），设 delta_t 为无穷大
  last_linear_acceleration_time_ = time_; // 更新时间戳time_
  // 1.计算平滑因子 alpha = 1 - e^(-Δt/τ) （τ 是构造函数传入的 imu_gravity_time_constant_）
  // Δt/τ 越大 → alpha 越接近 1（新观测权重高） → 适合初始化场景
  // Δt/τ 越小 → alpha 接近 0（历史数据权重高） → 避免高频噪声干扰
  const double alpha = 1. - std::exp(-delta_t / imu_gravity_time_constant_);
  // 2.更新重力方向（指数移动平均）g_new = (1-alpha)*g_old + alpha*a_measured
  gravity_vector_ =
      (1. - alpha) * gravity_vector_ + alpha * imu_linear_acceleration;
  // Change the 'orientation_' so that it agrees with the current
  // 'gravity_vector_'.
  // 3.姿态对齐重力​：​通过加速度计观测的重力方向，修正陀螺仪积分得到的姿态四元数​
  // 计算当前重力方向 (gravity_vector_) 与理论重力方向（[0,0,1] 在当前姿态坐标系下的投影）之间的旋转
  const Eigen::Quaterniond rotation = FromTwoVectors( // FromTwoVectors(a,b)：返回从向量 a 旋转到 b 的四元数。
      gravity_vector_, orientation_.conjugate() * Eigen::Vector3d::UnitZ()); // orientation_.conjugate() * Eigen::Vector3d::UnitZ()：将世界坐标系的 +Z 轴转换到局部坐标系
  orientation_ = (orientation_ * rotation).normalized(); // 将旋转应用到当前姿态，使估计的重力方向与理论方向对齐。
  CHECK_GT((orientation_ * gravity_vector_).z(), 0.);
  CHECK_GT((orientation_ * gravity_vector_).normalized().z(), 0.99);
}

// AddImuAngularVelocityObservation：引入IMU角速度观测，直接将角速度赋值给成员变量imu_angular_velocity_
void ImuTracker::AddImuAngularVelocityObservation(
    const Eigen::Vector3d& imu_angular_velocity) {
  imu_angular_velocity_ = imu_angular_velocity;
}

}  // namespace mapping
}  // namespace cartographer
