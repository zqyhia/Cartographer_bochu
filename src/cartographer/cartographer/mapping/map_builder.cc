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

#include "cartographer/mapping/map_builder.h"

#include "absl/memory/memory.h"
#include "absl/types/optional.h"
#include "cartographer/common/time.h"
#include "cartographer/io/internal/mapping_state_serialization.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/io/serialization_format_migration.h"
#include "cartographer/mapping/internal/2d/local_trajectory_builder_2d.h"
#include "cartographer/mapping/internal/2d/pose_graph_2d.h"
#include "cartographer/mapping/internal/3d/local_trajectory_builder_3d.h"
#include "cartographer/mapping/internal/3d/pose_graph_3d.h"
#include "cartographer/mapping/internal/collated_trajectory_builder.h"
#include "cartographer/mapping/internal/global_trajectory_builder.h"
#include "cartographer/mapping/internal/motion_filter.h"
#include "cartographer/sensor/internal/collator.h"
#include "cartographer/sensor/internal/trajectory_collator.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace mapping {
namespace {

using mapping::proto::SerializedData;

// SelectRangeSensorIds：从一组传感器ID中筛选出所有测距传感器（RANGE类型）的ID，并以字符串向量的形式返回
std::vector<std::string> SelectRangeSensorIds(
    const std::set<MapBuilder::SensorId>& expected_sensor_ids) {
  std::vector<std::string> range_sensor_ids;
  for (const MapBuilder::SensorId& sensor_id : expected_sensor_ids) {
    if (sensor_id.type == MapBuilder::SensorId::SensorType::RANGE) {
      range_sensor_ids.push_back(sensor_id.id);
    }
  }
  return range_sensor_ids;
}

// MaybeAddPureLocalizationTrimmer：根据轨迹配置，决定是否为指定轨迹添加"纯定位"修整器
void MaybeAddPureLocalizationTrimmer(
    const int trajectory_id,
    const proto::TrajectoryBuilderOptions& trajectory_options,
    PoseGraph* pose_graph) {
  // 处理旧版配置方式（已废弃）
  if (trajectory_options.pure_localization()) {
    LOG(WARNING)
        << "'TrajectoryBuilderOptions::pure_localization' field is deprecated. "
           "Use 'TrajectoryBuilderOptions::pure_localization_trimmer' instead.";
    pose_graph->AddTrimmer(absl::make_unique<PureLocalizationTrimmer>(
        trajectory_id, 3 /* max_submaps_to_keep */));
    return;
  }
  // 处理新版配置方式
  if (trajectory_options.has_pure_localization_trimmer()) {
    pose_graph->AddTrimmer(absl::make_unique<PureLocalizationTrimmer>(
        trajectory_id,
        trajectory_options.pure_localization_trimmer().max_submaps_to_keep()));
  }
}

}  // namespace

// MapBuilder构造函数：负责根据配置选项初始化2D/3D建图所需的各个子系统，包括位姿图(PoseGraph)和传感器数据整理器(SensorCollator)
MapBuilder::MapBuilder(const proto::MapBuilderOptions& options)
    : options_(options), thread_pool_(options.num_background_threads()) {
  CHECK(options.use_trajectory_builder_2d() ^
        options.use_trajectory_builder_3d());
  if (options.use_trajectory_builder_2d()) { // 2D模式
    pose_graph_ = absl::make_unique<PoseGraph2D>(
        options_.pose_graph_options(),
        absl::make_unique<optimization::OptimizationProblem2D>(
            options_.pose_graph_options().optimization_problem_options()),
        &thread_pool_);
  }
  if (options.use_trajectory_builder_3d()) { // 3D模式
    pose_graph_ = absl::make_unique<PoseGraph3D>(
        options_.pose_graph_options(),
        absl::make_unique<optimization::OptimizationProblem3D>(
            options_.pose_graph_options().optimization_problem_options()),
        &thread_pool_);
  }
  if (options.collate_by_trajectory()) { // 按轨迹整理——sensor_collator 的设置：sensor_collator_是一个接口<sensor::CollatorInterface>的智能指针。根据配置collate_by_trajectory，有两种实现方式：
    sensor_collator_ = absl::make_unique<sensor::TrajectoryCollator>(); // 在/sensor/collator_interface.h中
  } else { // 默认整理
    sensor_collator_ = absl::make_unique<sensor::Collator>(); // 在/sensor/internal/collator.h 中
  }
}

// AddTrajectoryBuilder：添加新轨迹构建器。初始化完整的SLAM处理流水线，包括前端局部建图、后端优化以及传感器数据处理等组件。
int MapBuilder::AddTrajectoryBuilder(
    const std::set<SensorId>& expected_sensor_ids,
    const proto::TrajectoryBuilderOptions& trajectory_options,
    LocalSlamResultCallback local_slam_result_callback) {
  // 阶段1：初始化基础信息
  const int trajectory_id = trajectory_builders_.size(); // 基于现有构建器数量自动递增

  // 阶段2：配置运动滤波器
  absl::optional<MotionFilter> pose_graph_odometry_motion_filter;
  if (trajectory_options.has_pose_graph_odometry_motion_filter()) {
    LOG(INFO) << "Using a motion filter for adding odometry to the pose graph.";
    pose_graph_odometry_motion_filter.emplace(
        MotionFilter(trajectory_options.pose_graph_odometry_motion_filter()));
  }

  // 阶段3：创建3D或2D轨迹构建器——这两个类是不带 Loop Closure 的 Local Slam, 包含了 Pose Extrapolator, Scan Matching 等。
  // 这两个类并没有继承 TrajectoryBuilder,并不是一个 TrajectoryBuilder 的实现，而只是一个工具类
  if (options_.use_trajectory_builder_3d()) {
    // 3D构建器初始化
    std::unique_ptr<LocalTrajectoryBuilder3D> local_trajectory_builder;
    if (trajectory_options.has_trajectory_builder_3d_options()) {
      local_trajectory_builder = absl::make_unique<LocalTrajectoryBuilder3D>(
          trajectory_options.trajectory_builder_3d_options(),
          SelectRangeSensorIds(expected_sensor_ids));
    }
    DCHECK(dynamic_cast<PoseGraph3D*>(pose_graph_.get())); // 检查类型转换：dynamic_cast 是强制类型转化，把PoseGraphInterface 的指针转化为 PoseGraph3D
    trajectory_builders_.push_back(absl::make_unique<CollatedTrajectoryBuilder>( // 在push_back函数里面，真正创建了一个TrajectoryBuilder。将生成的 CollatedTrajectoryBuilder 压入向量列表中
        trajectory_options, sensor_collator_.get(), trajectory_id,
        expected_sensor_ids,
        CreateGlobalTrajectoryBuilder3D(
            std::move(local_trajectory_builder), trajectory_id,
            static_cast<PoseGraph3D*>(pose_graph_.get()),
            local_slam_result_callback, pose_graph_odometry_motion_filter)));
  } else {
    // 2D构建器初始化
    std::unique_ptr<LocalTrajectoryBuilder2D> local_trajectory_builder;
    if (trajectory_options.has_trajectory_builder_2d_options()) {
      local_trajectory_builder = absl::make_unique<LocalTrajectoryBuilder2D>(
          trajectory_options.trajectory_builder_2d_options(), // 配置选项
          SelectRangeSensorIds(expected_sensor_ids)); // 筛选的距离传感器ID
    }
    DCHECK(dynamic_cast<PoseGraph2D*>(pose_graph_.get()));
    trajectory_builders_.push_back(absl::make_unique<CollatedTrajectoryBuilder>( // CollatedTrajectoryBuilder：传感器数据整理，继承了接口 TrajectoryBuilder
        trajectory_options, sensor_collator_.get(), trajectory_id,
        expected_sensor_ids,
        CreateGlobalTrajectoryBuilder2D( // GlobalTrajectoryBuilder：连接前后端的适配器
            std::move(local_trajectory_builder), trajectory_id, // local_trajectory_builder用在这里，作为参数用于生成一个CollatedTrajectoryBuilder 的智能指针
            static_cast<PoseGraph2D*>(pose_graph_.get()),
            local_slam_result_callback, pose_graph_odometry_motion_filter)));
  }
  // 阶段4：添加修整器（如纯定位模式）
  MaybeAddPureLocalizationTrimmer(trajectory_id, trajectory_options,
                                  pose_graph_.get());

  // 阶段5：设置初始位姿
  if (trajectory_options.has_initial_trajectory_pose()) { // 开始一条轨迹前我们是否已知初始位姿
    const auto& initial_trajectory_pose =
        trajectory_options.initial_trajectory_pose();
    pose_graph_->SetInitialTrajectoryPose( // 以该位姿作为新增加的 trajectory 的初始位姿
        trajectory_id, initial_trajectory_pose.to_trajectory_id(),
        transform::ToRigid3(initial_trajectory_pose.relative_pose()),
        common::FromUniversal(initial_trajectory_pose.timestamp()));
  }
  // 阶段6：记录配置信息——将一些跟传感器相关的配置项转成 proto 流，然后统一放到 all_trajectory_builder_options_这个向量列表中
  proto::TrajectoryBuilderOptionsWithSensorIds options_with_sensor_ids_proto;
  for (const auto& sensor_id : expected_sensor_ids) {
    *options_with_sensor_ids_proto.add_sensor_id() = ToProto(sensor_id);
  }
  *options_with_sensor_ids_proto.mutable_trajectory_builder_options() =
      trajectory_options;
  all_trajectory_builder_options_.push_back(options_with_sensor_ids_proto);
  CHECK_EQ(trajectory_builders_.size(), all_trajectory_builder_options_.size());
  return trajectory_id;
}

// AddTrajectoryForDeserialization：反序列化时重建轨迹的简化接口，仅初始化轨迹的元数据而不创建完整的处理流水线
int MapBuilder::AddTrajectoryForDeserialization(
    const proto::TrajectoryBuilderOptionsWithSensorIds&
        options_with_sensor_ids_proto) {
  const int trajectory_id = trajectory_builders_.size(); // 分配新轨迹ID（基于当前容器大小）
  trajectory_builders_.emplace_back(); // 添加空轨迹构建器占位符
  all_trajectory_builder_options_.push_back(options_with_sensor_ids_proto); // 保存完整的配置信息
  CHECK_EQ(trajectory_builders_.size(), all_trajectory_builder_options_.size()); // 验证容器同步性
  return trajectory_id;
}

// FinishTrajectory：标记轨迹完成
void MapBuilder::FinishTrajectory(const int trajectory_id) {
  sensor_collator_->FinishTrajectory(trajectory_id); // 通知传感器数据收集器停止该轨迹的数据处理
  pose_graph_->FinishTrajectory(trajectory_id); // 通知位姿图进行该轨迹的最终优化和收尾
}

// SubmapToProto：将子地图数据序列化为protobuf格式
// 如果出现错误，返回 error string; 成功则返回 empty string.
std::string MapBuilder::SubmapToProto(
    const SubmapId& submap_id, proto::SubmapQuery::Response* const response) {
  // 阶段1：验证轨迹ID有效性
  if (submap_id.trajectory_id < 0 ||
      submap_id.trajectory_id >= num_trajectory_builders()) {
    return "Requested submap from trajectory " +
           std::to_string(submap_id.trajectory_id) + " but there are only " +
           std::to_string(num_trajectory_builders()) + " trajectories.";
  }

  // 阶段2：获取子地图数据
  const auto submap_data = pose_graph_->GetSubmapData(submap_id);
  // 阶段3：验证子地图存在性
  if (submap_data.submap == nullptr) {
    return "Requested submap " + std::to_string(submap_id.submap_index) +
           " from trajectory " + std::to_string(submap_id.trajectory_id) +
           " but it does not exist: maybe it has been trimmed.";
  }
  // 阶段4：执行序列化
  submap_data.submap->ToResponseProto(submap_data.pose, response);
  return "";
}

// SerializeState：序列化整个SLAM系统状态。将内存中的地图数据、轨迹配置等信息写入到指定的输出流中
void MapBuilder::SerializeState(bool include_unfinished_submaps,
                                io::ProtoStreamWriterInterface* const writer) {
  io::WritePbStream(*pose_graph_, all_trajectory_builder_options_, writer,
                    include_unfinished_submaps);
}

// SerializeStateToFile：将整个SLAM系统状态序列化到pbstream文件
bool MapBuilder::SerializeStateToFile(bool include_unfinished_submaps,
                                      const std::string& filename) {
  io::ProtoStreamWriter writer(filename);
  io::WritePbStream(*pose_graph_, all_trajectory_builder_options_, &writer,
                    include_unfinished_submaps);
  return (writer.Close());
}

// LoadState：加载pbstream文件的状态。返回值：轨迹 ID 的映射关系，键是原始轨迹 ID，值是加载后的新轨迹 ID。
std::map<int, int> MapBuilder::LoadState(
    io::ProtoStreamReaderInterface* const reader, bool load_frozen_state) { // 输入：reader：用于读取序列化数据的接口；load_frozen_state: 布尔值，指示是否以冻结状态加载轨迹
  io::ProtoStreamDeserializer deserializer(reader); // 初始化反序列化器，用于从 reader 中反序列化 protobuf 数据

  // Create a copy of the pose_graph_proto, such that we can re-write the
  // trajectory ids.
  // 创建一个姿态图副本（pose_graph_proto）来修改轨迹ID
  proto::PoseGraph pose_graph_proto = deserializer.pose_graph(); // 反序列化获得可以使用的 pose_graph_proto ——从.pbstream文件读取的 ​​最顶层的姿势图结构，包含：所有轨迹的 ​​ID列表​、每个轨迹下的 ​​子地图位姿（​​不包含​​ 子地图的具体栅格数据）
  const auto& all_builder_options_proto =
      deserializer.all_trajectory_builder_options(); // 获取所有轨迹的构建选项配置

  std::map<int, int> trajectory_remapping; // 创建一个映射表，用于记录原始轨迹 ID 到新轨迹 ID 的映射关系

  // 将pose_graph_proto中的轨迹原型trajectory_proto的轨迹ID更新为新ID，将原ID到新ID的映射记录在trajectory_remapping中
  for (int i = 0; i < pose_graph_proto.trajectory_size(); ++i) { // 遍历姿势图中的每条轨迹
    auto& trajectory_proto = *pose_graph_proto.mutable_trajectory(i); // 获取当前轨迹的 protobuf 表示，mutable_旨在获取轨迹数据的 ​​可修改引用
    const auto& options_with_sensor_ids_proto =
        all_builder_options_proto.options_with_sensor_ids(i); // 获取当前轨迹的构建选项和传感器 ID 配置
    const int new_trajectory_id =
        AddTrajectoryForDeserialization(options_with_sensor_ids_proto); // 为反序列化添加新轨迹：根据配置创建一个新轨迹，并返回新轨迹 ID
    CHECK(trajectory_remapping
              .emplace(trajectory_proto.trajectory_id(), new_trajectory_id) // 将原始轨迹 ID 映射到新轨迹 ID，将键值对插入到trajectory_remapping中，使用 CHECK 检查emplace的.second（布尔值），确定是否成功
              .second)
        << "Duplicate trajectory ID: " << trajectory_proto.trajectory_id();
    trajectory_proto.set_trajectory_id(new_trajectory_id); // 将轨迹原型中的轨迹 ID 更新为新 ID，彻底替换
    if (load_frozen_state) {
      pose_graph_->FreezeTrajectory(new_trajectory_id); // 如果 load_frozen_state 为真，则冻结新轨迹
    }
  }

  // Apply the calculated remapping to constraints in the pose graph proto.
  // 根据新旧ID映射表trajectory_remapping，更新pose_graph_proto中的约束原型constraint_proto 中的子地图ID与节点ID的 轨迹ID
  for (auto& constraint_proto : *pose_graph_proto.mutable_constraint()) { // 遍历所有约束，更新其中的子地图 ID 和节点 ID 的轨迹 ID 部分，mutable_旨在获取轨迹数据的 ​​可修改引用
    constraint_proto.mutable_submap_id()->set_trajectory_id(
        trajectory_remapping.at(constraint_proto.submap_id().trajectory_id())); // 更新子地图ID的轨迹ID
    constraint_proto.mutable_node_id()->set_trajectory_id(
        trajectory_remapping.at(constraint_proto.node_id().trajectory_id())); // 更新节点ID的轨迹ID
  }

  // 在刚刚修改完轨迹ID的姿势图原型pose_graph_proto中，遍历轨迹下的轨迹原型trajectory_proto的子地图原型submap_proto以获取子地图索引，并将组合插入到子地图位姿组合submap_poses中
  MapById<SubmapId, transform::Rigid3d> submap_poses; // 创建一个 MapById 结构 submap_poses 来存储子地图 ID 到其姿势的映射
  for (const proto::Trajectory& trajectory_proto : // 遍历所有轨迹
       pose_graph_proto.trajectory()) { // 区别于mutable_trajectory，不可以对其进行修改操作
    for (const proto::Trajectory::Submap& submap_proto : // 遍历轨迹下所有子地图
         trajectory_proto.submap()) {
      submap_poses.Insert(SubmapId{trajectory_proto.trajectory_id(), // 通过 轨迹原型中的轨迹ID（已更新） 和 轨迹原型中的子地图索引（表示子地图在该轨迹中的索引） 构建子地图ID
                                   submap_proto.submap_index()}, // 在 Cartographer 中，子地图通过 (轨迹ID, 子地图索引) 二元组唯一标识
                          transform::ToRigid3(submap_proto.pose())); // 子地图位姿转换为Rigid3d
    }
  }

  // 与上一段类似，遍历修改后的轨迹原型获取轨迹上的节点索引，并构建(节点ID,节点位姿)的组合node_poses
  MapById<NodeId, transform::Rigid3d> node_poses; // 创建一个 MapById 结构 node_poses 来存储节点 ID 到其姿势的映射
  for (const proto::Trajectory& trajectory_proto : // 遍历所有轨迹
       pose_graph_proto.trajectory()) {
    for (const proto::Trajectory::Node& node_proto : trajectory_proto.node()) { // 遍历轨迹下所有节点
      node_poses.Insert(
          NodeId{trajectory_proto.trajectory_id(), node_proto.node_index()}, // 节点通过(轨迹ID, 节点索引)二元组唯一标识
          transform::ToRigid3(node_proto.pose())); // 节点位姿转换为Rigid3d
    }
  }

  // Set global poses of landmarks.
  for (const auto& landmark : pose_graph_proto.landmark_poses()) { // 遍历所有姿态图原型的地标位姿，将其添加到姿势图中
    pose_graph_->SetLandmarkPose(landmark.landmark_id(), // 路标ID
                                 transform::ToRigid3(landmark.global_pose()), // 地标的全局姿势
                                 true); // true表示覆盖已存在的地标姿势
  }
  // 至此，完成了基于位姿图副本pose_graph_proto的轨迹ID修改，获得了轨迹重映射表trajectory_remapping、子地图ID与其姿态的组合submap_poses、节点ID与其姿态的组合node_poses。还把地标加入到了姿态图中

  if (options_.use_trajectory_builder_3d()) { // 3D构建的特殊检查
    CHECK_NE(deserializer.header().format_version(),
             io::kFormatVersionWithoutSubmapHistograms)
        << "The pbstream file contains submaps without rotational histograms. "
           "This can be converted with the 'pbstream migrate' tool, see the "
           "Cartographer documentation for details. ";
  }

  // 下面开始进行pbstream文件详细的读取分析
  SerializedData proto; // 创建 SerializedData 临时对象 (proto)
  while (deserializer.ReadNextSerializedData(&proto)) { // 循环调用 ReadNextSerializedData() 从pbstream文件中读取下一个数据块，将其写入临时对象proto，每次循环都会重新覆盖一次proto
    switch (proto.data_case()) { // 根据 data_case() 判断实际数据类型，根据不同类型调用对应处理逻辑
      case SerializedData::kPoseGraph: // 检测到重复的姿势图数据，记录错误
        LOG(ERROR) << "Found multiple serialized `PoseGraph`. Serialized "
                      "stream likely corrupt!.";
        break;
      case SerializedData::kAllTrajectoryBuilderOptions: // 检测到重复的轨迹构建选项，记录错误
        LOG(ERROR) << "Found multiple serialized "
                      "`AllTrajectoryBuilderOptions`. Serialized stream likely "
                      "corrupt!.";
        break;
      case SerializedData::kSubmap: {
        // 更新轨迹ID映射：原始ID → 查重映射表 → 更新protobuf消息。
        proto.mutable_submap()->mutable_submap_id()->set_trajectory_id( // 这里用的是可修改的mutable
            trajectory_remapping.at(
                proto.submap().submap_id().trajectory_id())); // 查询重映射表用的是只读的方式
        // 构造子地图ID：使用了刚刚更新过的轨迹ID，将要用于下一步的子地图添加
        const SubmapId submap_id(proto.submap().submap_id().trajectory_id(), // 已经重映射的轨迹ID
                                 proto.submap().submap_id().submap_index()); // 原始索引保持不变
        // 子地图添加：(位姿,完整的子地图数据)
        pose_graph_->AddSubmapFromProto(submap_poses.at(submap_id), // 基于先前的姿态图副本预存的 优化位姿
                                        proto.submap()); // 完整的子地图数据
        break;
      }
      case SerializedData::kNode: { // 和刚刚子地图操作相同
        // 轨迹ID重映射
        proto.mutable_node()->mutable_node_id()->set_trajectory_id(
            trajectory_remapping.at(proto.node().node_id().trajectory_id()));
        // 构造节点ID
        const NodeId node_id(proto.node().node_id().trajectory_id(),
                             proto.node().node_id().node_index());
        // 获取预存位姿
        const transform::Rigid3d& node_pose = node_poses.at(node_id);
        // 将节点添加到姿势图
        pose_graph_->AddNodeFromProto(node_pose, proto.node());
        break;
      }
      case SerializedData::kTrajectoryData: {
        // 轨迹ID重映射
        proto.mutable_trajectory_data()->set_trajectory_id(
            trajectory_remapping.at(proto.trajectory_data().trajectory_id()));
        // 设置轨迹全局数据
        pose_graph_->SetTrajectoryDataFromProto(proto.trajectory_data());
        break;
      }
      case SerializedData::kImuData: {
        if (load_frozen_state) break; // 跳过冻结状态的轨迹
        pose_graph_->AddImuData( // 将IMU数据添加到位姿图
            trajectory_remapping.at(proto.imu_data().trajectory_id()), // 轨迹ID：基于轨迹重映射表映射
            sensor::FromProto(proto.imu_data().imu_data())); // IMU数据：经过数据格式转换
        break;
      }
      case SerializedData::kOdometryData: { // 和IMU差不多，将里程计数据添加到位姿图
        if (load_frozen_state) break;
        pose_graph_->AddOdometryData(
            trajectory_remapping.at(proto.odometry_data().trajectory_id()),
            sensor::FromProto(proto.odometry_data().odometry_data()));
        break;
      }
      case SerializedData::kFixedFramePoseData: { // 处理外部参考坐标系数据
        if (load_frozen_state) break;
        pose_graph_->AddFixedFramePoseData(
            trajectory_remapping.at(
                proto.fixed_frame_pose_data().trajectory_id()),
            sensor::FromProto(
                proto.fixed_frame_pose_data().fixed_frame_pose_data()));
        break;
      }
      case SerializedData::kLandmarkData: { // 处理路标数据
        if (load_frozen_state) break;
        pose_graph_->AddLandmarkData(
            trajectory_remapping.at(proto.landmark_data().trajectory_id()),
            sensor::FromProto(proto.landmark_data().landmark_data()));
        break;
      }
      default:
        LOG(WARNING) << "Skipping unknown message type in stream: "
                     << proto.GetTypeName();
    }
  }

  if (load_frozen_state) { // 冻结状态处理
    // Add information about which nodes belong to which submap.
    // This is required, even without constraints.
    for (const proto::PoseGraph::Constraint& constraint_proto :
         pose_graph_proto.constraint()) {
      if (constraint_proto.tag() !=
          proto::PoseGraph::Constraint::INTRA_SUBMAP) {
        continue;
      }
      pose_graph_->AddNodeToSubmap(
          NodeId{constraint_proto.node_id().trajectory_id(),
                 constraint_proto.node_id().node_index()},
          SubmapId{constraint_proto.submap_id().trajectory_id(),
                   constraint_proto.submap_id().submap_index()});
    }
  } else {
    // When loading unfrozen trajectories, 'AddSerializedConstraints' will
    // take care of adding information about which nodes belong to which
    // submap.
    pose_graph_->AddSerializedConstraints(
        FromProto(pose_graph_proto.constraint()));
  }
  CHECK(reader->eof());
  return trajectory_remapping; // 返回原始轨迹 ID 到新轨迹 ID 的映射关系
}

// LoadStateFromFile：从pbstream中加载状态，调用LoadState函数
std::map<int, int> MapBuilder::LoadStateFromFile(
    const std::string& state_filename, const bool load_frozen_state) {
  const std::string suffix = ".pbstream";
  if (state_filename.substr(
          std::max<int>(state_filename.size() - suffix.size(), 0)) != suffix) {
    LOG(WARNING) << "The file containing the state should be a "
                    ".pbstream file.";
  }
  LOG(INFO) << "Loading saved state '" << state_filename << "'...";
  io::ProtoStreamReader stream(state_filename);
  return LoadState(&stream, load_frozen_state);
}

std::unique_ptr<MapBuilderInterface> CreateMapBuilder(
    const proto::MapBuilderOptions& options) {
  return absl::make_unique<MapBuilder>(options);
}

}  // namespace mapping
}  // namespace cartographer
