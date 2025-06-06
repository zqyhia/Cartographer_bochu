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

#ifndef CARTOGRAPHER_MAPPING_SUBMAPS_H_
#define CARTOGRAPHER_MAPPING_SUBMAPS_H_

#include <memory>
#include <vector>

#include "Eigen/Geometry"
#include "cartographer/common/math.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/probability_values.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/mapping/trajectory_node.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

// Converts the given probability to log odds.
inline float Logit(float probability) {
  return std::log(probability / (1.f - probability));
}

const float kMaxLogOdds = Logit(kMaxProbability);
const float kMinLogOdds = Logit(kMinProbability);

// Converts a probability to a log odds integer. 0 means unknown, [kMinLogOdds,
// kMaxLogOdds] is mapped to [1, 255].
inline uint8 ProbabilityToLogOddsInteger(const float probability) {
  const int value = common::RoundToInt((Logit(probability) - kMinLogOdds) *
                                       254.f / (kMaxLogOdds - kMinLogOdds)) +
                    1;
  CHECK_LE(1, value);
  CHECK_GE(255, value);
  return value;
}

// An individual submap, which has a 'local_pose' in the local map frame, keeps
// track of how many range data were inserted into it, and sets
// 'insertion_finished' when the map no longer changes and is ready for loop
// closing.
class Submap {
 public:
  Submap(const transform::Rigid3d& local_submap_pose) // 构造函数，只包括一个 local_pose。这个 local_pose 可以看做是没有经过全局优化的该 submap 相对于世界坐标系的位姿。
      : local_pose_(local_submap_pose) {}
  virtual ~Submap() {}

  // 序列化与反序列化
  virtual proto::Submap ToProto(bool include_grid_data) const = 0;
  virtual void UpdateFromProto(const proto::Submap& proto) = 0;

  // Fills data into the 'response'.
  // 把 submap 放入到 response 的 proto 流中。方便 service 中查询 submap 讯息。
  virtual void ToResponseProto(
      const transform::Rigid3d& global_submap_pose,
      proto::SubmapQuery::Response* response) const = 0;

  // Pose of this submap in the local map frame.
  // 返回成员变量 local_pose_，即该 submap 的位姿
  transform::Rigid3d local_pose() const { return local_pose_; }

  // Number of RangeData inserted.
  // 返回成员变量 num_range_data_，即插入到该 Submap 中的 range data 的数量。
  int num_range_data() const { return num_range_data_; }
  // 设置成员变量 num_range_data_
  void set_num_range_data(const int num_range_data) {
    num_range_data_ = num_range_data;
  }

  // 查看布尔型成员变量 insertion_finished_，即该子图是否还需要更新
  bool insertion_finished() const { return insertion_finished_; }
  // 设置 insertion_finished_
  void set_insertion_finished(bool insertion_finished) {
    insertion_finished_ = insertion_finished;
  }

 private:
  const transform::Rigid3d local_pose_;
  int num_range_data_ = 0;
  bool insertion_finished_ = false;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_SUBMAPS_H_
