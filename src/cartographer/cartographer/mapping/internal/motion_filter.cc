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

#include "cartographer/mapping/internal/motion_filter.h"

#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

// 从 Proto 流中读取配置文件，设置配置项，主要包括最大时间间隔、最大角度间隔、最大距离间隔等
proto::MotionFilterOptions CreateMotionFilterOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::MotionFilterOptions options;
  options.set_max_time_seconds(
      parameter_dictionary->GetDouble("max_time_seconds"));
  options.set_max_distance_meters(
      parameter_dictionary->GetDouble("max_distance_meters"));
  options.set_max_angle_radians(
      parameter_dictionary->GetDouble("max_angle_radians"));
  return options;
}

// MotionFilter构造函数，加载配置
MotionFilter::MotionFilter(const proto::MotionFilterOptions& options)
    : options_(options) {}

// IsSimilar：判断与之前的位姿是否相似。根据预先设置的阈值，如果累积运动小于等于提前预设的阈值，则返回 true；否则返回 false，然后把该数据累加上
bool MotionFilter::IsSimilar(const common::Time time,
                             const transform::Rigid3d& pose) {
  LOG_IF_EVERY_N(INFO, num_total_ >= 500, 500)
      << "Motion filter reduced the number of nodes to "
      << 100. * num_different_ / num_total_ << "%.";
  ++num_total_;
  if (num_total_ > 1 && // 总的 pose 数大于 1
      time - last_time_ <= common::FromSeconds(options_.max_time_seconds()) && // 时间间隔小于给定阈值
      (pose.translation() - last_pose_.translation()).norm() <=
          options_.max_distance_meters() && // 位移间隔小于给定阈值
      transform::GetAngle(pose.inverse() * last_pose_) <=
          options_.max_angle_radians()) { // 偏转角度小于给定阈值
    return true;
  }
  last_time_ = time;
  last_pose_ = pose;
  ++num_different_;
  return false;
}

}  // namespace mapping
}  // namespace cartographer
