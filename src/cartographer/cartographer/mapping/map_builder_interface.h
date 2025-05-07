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

#ifndef CARTOGRAPHER_MAPPING_MAP_BUILDER_INTERFACE_H_
#define CARTOGRAPHER_MAPPING_MAP_BUILDER_INTERFACE_H_

#include <set>
#include <string>
#include <vector>

#include "Eigen/Geometry"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/io/proto_stream_interface.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/mapping/proto/map_builder_options.pb.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/mapping/trajectory_builder_interface.h"

namespace cartographer {
namespace mapping {

proto::MapBuilderOptions CreateMapBuilderOptions(
    common::LuaParameterDictionary* const parameter_dictionary);

// This interface is used for both library and RPC implementations.
// Implementations wire up the complete SLAM stack.
class MapBuilderInterface {
 public:
  using LocalSlamResultCallback =
      TrajectoryBuilderInterface::LocalSlamResultCallback; // LocalSlamResultCallback全称

  using SensorId = TrajectoryBuilderInterface::SensorId;

  MapBuilderInterface() {}
  virtual ~MapBuilderInterface() {}

  MapBuilderInterface(const MapBuilderInterface&) = delete;
  MapBuilderInterface& operator=(const MapBuilderInterface&) = delete;

  // Creates a new trajectory builder and returns its index.
  // 创建一个 TrajectoryBuilder 并返回他的 index，即 trajectory_id
  virtual int AddTrajectoryBuilder(
      const std::set<SensorId>& expected_sensor_ids,
      const proto::TrajectoryBuilderOptions& trajectory_options,
      LocalSlamResultCallback local_slam_result_callback) = 0; // LocalSlamResultCallback 是一个回调函数

  // Creates a new trajectory and returns its index. Querying the trajectory
  // builder for it will return 'nullptr'.
  // 从一个序列化的数据中构造出一个 trajectory 并返回他的 index
  virtual int AddTrajectoryForDeserialization(
      const proto::TrajectoryBuilderOptionsWithSensorIds&
          options_with_sensor_ids_proto) = 0;

  // Returns the 'TrajectoryBuilderInterface' corresponding to the specified
  // 'trajectory_id' or 'nullptr' if the trajectory has no corresponding
  // builder.
  // 根据 trajectory_id 返回一个 TrajectoryBuilderInterface 的指针（没有就是空指针）。
  virtual mapping::TrajectoryBuilderInterface* GetTrajectoryBuilder(
      int trajectory_id) const = 0;

  // Marks the TrajectoryBuilder corresponding to 'trajectory_id' as finished,
  // i.e. no further sensor data is expected.
  // Finish 指定 id 的 trajectory
  virtual void FinishTrajectory(int trajectory_id) = 0;

  // Fills the SubmapQuery::Response corresponding to 'submap_id'. Returns an
  // error string on failure, or an empty string on success.
  // 根据指定的 submap_id 来查询 submap，把结果放到 SubmapQuery::Response 中。
  // 如果出现错误，返回 error string; 成功则返回 empty string.
  virtual std::string SubmapToProto(const SubmapId& submap_id,
                                    proto::SubmapQuery::Response* response) = 0;

  // Serializes the current state to a proto stream. If
  // 'include_unfinished_submaps' is set to true, unfinished submaps, i.e.
  // submaps that have not yet received all rangefinder data insertions, will
  // be included in the serialized state.
  // 序列化当前状态到一个 proto 流中。
  virtual void SerializeState(bool include_unfinished_submaps,
                              io::ProtoStreamWriterInterface* writer) = 0;

  // Serializes the current state to a proto stream file on the host system. If
  // 'include_unfinished_submaps' is set to true, unfinished submaps, i.e.
  // submaps that have not yet received all rangefinder data insertions, will
  // be included in the serialized state.
  // Returns true if the file was successfully written.
  virtual bool SerializeStateToFile(bool include_unfinished_submaps,
                                    const std::string& filename) = 0;

  // Loads the SLAM state from a proto stream. Returns the remapping of new
  // trajectory_ids.
  // 从一个 proto 流中加载 SLAM 状态
  virtual std::map<int /* trajectory id in proto */, int /* trajectory id */>
  LoadState(io::ProtoStreamReaderInterface* reader, bool load_frozen_state) = 0;

  // Loads the SLAM state from a pbstream file. Returns the remapping of new
  // trajectory_ids.

  virtual std::map<int /* trajectory id in proto */, int /* trajectory id */>
  LoadStateFromFile(const std::string& filename, bool load_frozen_state) = 0;

  // 返回系统中当前已有的 trajectory_builder 的数量
  virtual int num_trajectory_builders() const = 0;

  // 返回一个 PoseGraphInterface 的接口指针。后面我们可以看到，PoseGrapher 用来进行 Loop Closure。
  virtual mapping::PoseGraphInterface* pose_graph() = 0;
  
  // 获取所有 TrajectoryBuilder 的配置项
  virtual const std::vector<proto::TrajectoryBuilderOptionsWithSensorIds>&
  GetAllTrajectoryBuilderOptions() const = 0;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_MAP_BUILDER_INTERFACE_H_
