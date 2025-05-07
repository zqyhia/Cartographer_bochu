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

#include "cartographer_ros/tf_bridge.h"

#include "absl/memory/memory.h"
#include "cartographer_ros/msg_conversion.h"

namespace cartographer_ros {

// 主要内容就是LookupToTracking函数，负责在ROS2中查找从任意坐标系到跟踪坐标系的TF变换，转换后返回 Cartographer 内部格式的变换。
TfBridge::TfBridge(const std::string& tracking_frame, // 跟踪坐标系名称
                   const double lookup_transform_timeout_sec, // 查找变换的超时时间(秒)
                   const tf2_ros::Buffer* buffer) // 指向tf2缓冲区的指针
    : tracking_frame_(tracking_frame),
      lookup_transform_timeout_sec_(lookup_transform_timeout_sec),
      buffer_(buffer) {}

std::unique_ptr<::cartographer::transform::Rigid3d> TfBridge::LookupToTracking( // 查找从frame_id到tracking_frame_的变换
    const ::cartographer::common::Time time,
    const std::string& frame_id) const {
  tf2::Duration timeout(tf2::durationFromSec(lookup_transform_timeout_sec_)); // 将超时时间从秒转换为tf2::Duration类型timeout
  std::unique_ptr<::cartographer::transform::Rigid3d> frame_id_to_tracking; // 声明返回的变换指针
  try {
    const rclcpp::Time latest_tf_time =
        buffer_
            ->lookupTransform(tracking_frame_, frame_id, ::rclcpp::Time(0.),
                              timeout)
            .header.stamp;
    const rclcpp::Time requested_time = ToRos(time);

    if (latest_tf_time >= requested_time) {
      // We already have newer data, so we do not wait. Otherwise, we would wait
      // for the full 'timeout' even if we ask for data that is too old.
      timeout = tf2::durationFromSec(0.0);
    }
    return absl::make_unique<::cartographer::transform::Rigid3d>( // 用absl::make_unique创建智能指针返回
        ToRigid3d(buffer_->lookupTransform(tracking_frame_, frame_id, // 使用ToRigid3d将tf2变换转换为Cartographer的Rigid3d格式
                                           requested_time, timeout)));
  } catch (const tf2::TransformException& ex) {
    LOG(WARNING) << ex.what();
  }
  return nullptr;
}

}  // namespace cartographer_ros
