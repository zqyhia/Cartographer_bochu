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

#include "cartographer_ros/node.h"

#include <chrono>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "absl/memory/memory.h"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/metrics/register.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/metrics/family_factory.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/sensor_bridge.h"
#include "cartographer_ros/tf_bridge.h"
#include "cartographer_ros/time_conversion.h"
#include "cartographer_ros_msgs/msg/status_code.hpp"
#include "cartographer_ros_msgs/msg/status_response.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "glog/logging.h"
#include "nav_msgs/msg/odometry.hpp"
//#include "ros/serialization.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace cartographer_ros {

namespace carto = ::cartographer;

using carto::transform::Rigid3d;
using TrajectoryState =
    ::cartographer::mapping::PoseGraphInterface::TrajectoryState;

namespace { // 匿名命名空间，当前文件可见
// Subscribes to the 'topic' for 'trajectory_id' using the 'node_handle' and
// calls 'handler' on the 'node' to handle messages. Returns the subscriber.
/*
模板函数，用于订阅 ROS 2 话题，并将接收到的消息传递给指定的处理函数handler，返回订阅器

返回值：
::rclcpp::SubscriptionBase::SharedPtr：表示订阅器的共享指针。

参数：
handler：指向 Node 类成员函数的指针，用于处理接收到的消息。
trajectory_id：轨迹 ID，用于标识当前轨迹。
topic：要订阅的话题名称。
node_handle：ROS 2 节点句柄，用于创建订阅器。
node：Node 类的实例，用于调用 handler 函数。

模板参数：
MessageType：消息类型（如 sensor_msgs::msg::LaserScan）。
*/ 

template <typename MessageType>
::rclcpp::SubscriptionBase::SharedPtr SubscribeWithHandler(
    void (Node::*handler)(int, const std::string&,
                          const typename MessageType::ConstSharedPtr&), // 从上一行void到这里都是一个需要输入的参数：Node类的成员函数指针handler，(...)表示的是该函数需要的参数
    const int trajectory_id, const std::string& topic, // 下面两行是其他参数
    ::rclcpp::Node::SharedPtr node_handle, Node* const node) { // node_handle：一个 ROS 2 节点的共享指针，表示 ROS 2 节点的上下文，用于创建订阅者。node：指向 Node 类的原始指针，用于访问 Node 类中的成员函数
  return node_handle->create_subscription<MessageType>( // 这里使用create_subscription创建订阅者，参数为话题、QoS、回调函数（Lambda表达式），返回的是订阅者的智能指针
      topic, rclcpp::SensorDataQoS(),
      [node, handler, trajectory_id, topic](const typename MessageType::ConstSharedPtr msg) {
            (node->*handler)(trajectory_id, topic, msg);
          });
}

std::string TrajectoryStateToString(const TrajectoryState trajectory_state) {
  switch (trajectory_state) {
    case TrajectoryState::ACTIVE:
      return "ACTIVE";
    case TrajectoryState::FINISHED:
      return "FINISHED";
    case TrajectoryState::FROZEN:
      return "FROZEN";
    case TrajectoryState::DELETED:
      return "DELETED";
  }
  return "";
}

}  // namespace
// cartographer_ros::Node构造函数
Node::Node(
    const NodeOptions& node_options,
    std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    rclcpp::Node::SharedPtr node,
    const bool collect_metrics)
    : node_options_(node_options)
{
  node_ = node; // 将传入的 ROS 2 节点句柄赋值给成员变量 node_
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_) ; // 初始化 TF 广播器，用于发布坐标变换

  // 初始化 MapBuilderBridge 对象，用于连接 Cartographer 的核心逻辑和 ROS 2 接口
  map_builder_bridge_.reset(new cartographer_ros::MapBuilderBridge(node_options_, std::move(map_builder), tf_buffer.get()));

  absl::MutexLock lock(&mutex_); // 对互斥量mutex_加锁，防止因为多线程等并行运算的方式产生异常的行为

  // 初始化指标注册器
  if (collect_metrics) {
    metrics_registry_ = absl::make_unique<metrics::FamilyFactory>();
    carto::metrics::RegisterAllMetrics(metrics_registry_.get());
  }

  // 初始化发布器
  /*
  submap_list_publisher_：发布子图列表。
  trajectory_node_list_publisher_：发布轨迹节点列表。
  landmark_poses_list_publisher_：发布地标位姿列表。
  constraint_list_publisher_：发布约束列表。
  tracked_pose_publisher_：发布跟踪的位姿（可选）。
  scan_matched_point_cloud_publisher_：发布匹配后的点云数据。
  */
  submap_list_publisher_ =
      node_->create_publisher<::cartographer_ros_msgs::msg::SubmapList>(
          kSubmapListTopic, 10);
  trajectory_node_list_publisher_ =
      node_->create_publisher<::visualization_msgs::msg::MarkerArray>(
          kTrajectoryNodeListTopic, 10);
  landmark_poses_list_publisher_ =
      node_->create_publisher<::visualization_msgs::msg::MarkerArray>(
          kLandmarkPosesListTopic, 10);
  constraint_list_publisher_ =
      node_->create_publisher<::visualization_msgs::msg::MarkerArray>(
          kConstraintListTopic, 10);
  if (node_options_.publish_tracked_pose) {
    tracked_pose_publisher_ =
        node_->create_publisher<::geometry_msgs::msg::PoseStamped>(
            kTrackedPoseTopic, 10);
  }
  scan_matched_point_cloud_publisher_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        kScanMatchedPointCloudTopic, 10);

  // 初始化服务
  /*
  submap_query_server_：处理子图查询请求。
  trajectory_query_server：处理轨迹查询请求。
  start_trajectory_server_：处理启动轨迹请求。
  finish_trajectory_server_：处理结束轨迹请求。
  write_state_server_：处理保存状态请求。
  get_trajectory_states_server_：处理获取轨迹状态请求。
  read_metrics_server_：处理读取指标请求。
  */
  submap_query_server_ = node_->create_service<cartographer_ros_msgs::srv::SubmapQuery>(
      kSubmapQueryServiceName,
      std::bind(
          &Node::handleSubmapQuery, this, std::placeholders::_1, std::placeholders::_2));
  trajectory_query_server = node_->create_service<cartographer_ros_msgs::srv::TrajectoryQuery>(
      kTrajectoryQueryServiceName,
      std::bind(
          &Node::handleTrajectoryQuery, this, std::placeholders::_1, std::placeholders::_2));
  start_trajectory_server_ = node_->create_service<cartographer_ros_msgs::srv::StartTrajectory>(
      kStartTrajectoryServiceName,
      std::bind(
          &Node::handleStartTrajectory, this, std::placeholders::_1, std::placeholders::_2));
  finish_trajectory_server_ = node_->create_service<cartographer_ros_msgs::srv::FinishTrajectory>(
      kFinishTrajectoryServiceName,
      std::bind(
          &Node::handleFinishTrajectory, this, std::placeholders::_1, std::placeholders::_2));
  write_state_server_ = node_->create_service<cartographer_ros_msgs::srv::WriteState>(
      kWriteStateServiceName,
      std::bind(
          &Node::handleWriteState, this, std::placeholders::_1, std::placeholders::_2));
  get_trajectory_states_server_ = node_->create_service<cartographer_ros_msgs::srv::GetTrajectoryStates>(
      kGetTrajectoryStatesServiceName,
      std::bind(
          &Node::handleGetTrajectoryStates, this, std::placeholders::_1, std::placeholders::_2));
  read_metrics_server_ = node_->create_service<cartographer_ros_msgs::srv::ReadMetrics>(
      kReadMetricsServiceName,
      std::bind(
          &Node::handleReadMetrics, this, std::placeholders::_1, std::placeholders::_2));

  // 初始化定时器
  /*
  submap_list_timer_：定期发布子图列表。
  local_trajectory_data_timer_：定期发布局部轨迹数据（可选）。
  trajectory_node_list_timer_：定期发布轨迹节点列表。
  landmark_pose_list_timer_：定期发布地标位姿列表。
  constrain_list_timer_：定期发布约束列表。
  */
  submap_list_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(int(node_options_.submap_publish_period_sec * 1000)),
    [this]() {
      PublishSubmapList();
    });
  if (node_options_.pose_publish_period_sec > 0) {
    local_trajectory_data_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(int(node_options_.pose_publish_period_sec * 1000)),
      [this]() {
        PublishLocalTrajectoryData(); // 局部轨迹数据发布函数
      });
  }
  trajectory_node_list_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(int(node_options_.trajectory_publish_period_sec * 1000)),
    [this]() {
      PublishTrajectoryNodeList();
    });
  landmark_pose_list_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(int(node_options_.trajectory_publish_period_sec * 1000)),
    [this]() {
      PublishLandmarkPosesList();
    });
  constrain_list_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(int(kConstraintPublishPeriodSec * 1000)),
    [this]() {
      PublishConstraintList();
    });
}

// 析构函数
Node::~Node() { FinishAllTrajectories(); }

bool Node::handleSubmapQuery(
    const cartographer_ros_msgs::srv::SubmapQuery::Request::SharedPtr request,
    cartographer_ros_msgs::srv::SubmapQuery::Response::SharedPtr response) {
  absl::MutexLock lock(&mutex_);
  map_builder_bridge_->HandleSubmapQuery(request, response);
  return true;
}

bool Node::handleTrajectoryQuery(
    const cartographer_ros_msgs::srv::TrajectoryQuery::Request::SharedPtr request,
    cartographer_ros_msgs::srv::TrajectoryQuery::Response::SharedPtr response) {
  absl::MutexLock lock(&mutex_);
  response->status = TrajectoryStateToStatus(
      request->trajectory_id,
      {TrajectoryState::ACTIVE, TrajectoryState::FINISHED,
       TrajectoryState::FROZEN} /* valid states */);
  if (response->status.code != cartographer_ros_msgs::msg::StatusCode::OK) {
    LOG(ERROR) << "Can't query trajectory from pose graph: "
               << response->status.message;
    return true;
  }
  map_builder_bridge_->HandleTrajectoryQuery(request, response);
  return true;
}

void Node::PublishSubmapList() {
  absl::MutexLock lock(&mutex_);
  submap_list_publisher_->publish(map_builder_bridge_->GetSubmapList(node_->now()));
}

// AddExtrapolator：初始化轨迹运动状态预测器
void Node::AddExtrapolator(const int trajectory_id,
                           const TrajectoryOptions& options) {
  constexpr double kExtrapolationEstimationTimeSec = 0.001;  // 定义 外推估计时间常数 1 ms
  CHECK(extrapolators_.count(trajectory_id) == 0); // 检查外推器是否已存在
  const double gravity_time_constant = // 获取重力时间常数，根据use_trajectory_builder_3d选取重力时间常数
      node_options_.map_builder_options.use_trajectory_builder_3d()
          ? options.trajectory_builder_options.trajectory_builder_3d_options()
                .imu_gravity_time_constant()
          : options.trajectory_builder_options.trajectory_builder_2d_options()
                .imu_gravity_time_constant();
  extrapolators_.emplace( // emplace：直接在容器内部构造元素，避免临时对象
      std::piecewise_construct, std::forward_as_tuple(trajectory_id), // std::piecewise_construct：标志位，告诉emplace我们要分别构造key和value；传递轨迹ID作为key
      std::forward_as_tuple( // std::forward_as_tuple：完美转发参数，保持值类别（左值/右值）
          ::cartographer::common::FromSeconds(kExtrapolationEstimationTimeSec), // 将 外推估计时间常数 和 重力时间常数 完美转发给 Extrapolator 的构造函数（自动推导）
          gravity_time_constant));
}

void Node::AddSensorSamplers(const int trajectory_id,
                             const TrajectoryOptions& options) {
  CHECK(sensor_samplers_.count(trajectory_id) == 0);
  sensor_samplers_.emplace(
      std::piecewise_construct, std::forward_as_tuple(trajectory_id),
      std::forward_as_tuple(
          options.rangefinder_sampling_ratio, options.odometry_sampling_ratio,
          options.fixed_frame_pose_sampling_ratio, options.imu_sampling_ratio,
          options.landmarks_sampling_ratio));
}

// 局部轨迹数据发布函数：从 map_builder_bridge_ 获取局部轨迹数据，并根据配置发布 激光点云 和 TF 变换
void Node::PublishLocalTrajectoryData() {
  absl::MutexLock lock(&mutex_);
  for (const auto& entry : map_builder_bridge_->GetLocalTrajectoryData()) { // 通过map_builder_bridge_->GetLocalTrajectoryData()获取局部轨迹数据（一个哈希表）
    const auto& trajectory_data = entry.second; // 哈希表第二项，LocalTrajectoryData结构体数据

    auto& extrapolator = extrapolators_.at(entry.first); // 通过轨迹ID获取对应的位姿预测器 at是map容器的方法，通过键查找对应的值，还会避免键不存在时插入默认值
    // We only publish a point cloud if it has changed. It is not needed at high
    // frequency, and republishing it would be computationally wasteful.
    if (trajectory_data.local_slam_data->time !=
        extrapolator.GetLastPoseTime()) { // 比较当前SLAM时间戳与预测器最后位姿时间，避免重复处理相同数据
      if (scan_matched_point_cloud_publisher_->get_subscription_count() > 0) { // 检查点云话题是否有订阅者，有订阅者的时候才处理
        // TODO(gaschler): Consider using other message without time
        // information.
        carto::sensor::TimedPointCloud point_cloud; // 初始化点云容器
        point_cloud.reserve(trajectory_data.local_slam_data->range_data_in_local // 根据点云数量预留空间
                                .returns.size());
        for (const cartographer::sensor::RangefinderPoint & point :
             trajectory_data.local_slam_data->range_data_in_local.returns) {
          point_cloud.push_back(cartographer::sensor::ToTimedRangefinderPoint( // 处理点云数据，将RangefinderPoint转换为TimedRangefinderPoint
              point, 0.f /* time */)); // 强制设置点时间为0（相对时间）
        }
        scan_matched_point_cloud_publisher_->publish(ToPointCloud2Message( // 发布点云
            carto::common::ToUniversal(trajectory_data.local_slam_data->time),
            node_options_.map_frame, // 引入map的框架
            carto::sensor::TransformTimedPointCloud(
                point_cloud, trajectory_data.local_to_map.cast<float>()))); // 将点云转换为sensor_msgs/PointCloud2格式，使用local_to_map坐标变换
      }
      extrapolator.AddPose(trajectory_data.local_slam_data->time, // AddPose 方法将最新位姿加入预测器内部队列，参数：(时间戳, Rigid3d位姿)
                           trajectory_data.local_slam_data->local_pose);
    }

    geometry_msgs::msg::TransformStamped stamped_transform; // 创建ROS2的带时间戳的TF消息，默认初始化所有字段为0/null
    // If we do not publish a new point cloud, we still allow time of the
    // published poses to advance. If we already know a newer pose, we use its
    // time instead. Since tf knows how to interpolate, providing newer
    // information is better.
    const ::cartographer::common::Time now = std::max(
        FromRos(node_->now()), extrapolator.GetLastExtrapolatedTime()); // 取当前ROS时间和最后预测时间的较大值，FromRos 转换ROS时间到Cartographer内部时间
    stamped_transform.header.stamp =
        node_options_.use_pose_extrapolator // 根据配置中是否使用姿态外推器（预测器）来选择
            ? ToRos(now) // 使用预测器则取当前时间和最后预测时间的较大值
            : ToRos(trajectory_data.local_slam_data->time); // 否则使用原始SLAM数据时间

    // Suppress publishing if we already published a transform at this time.
    // Due to 2020-07 changes to geometry2, tf buffer will issue warnings for
    // repeated transforms with the same timestamp.
    if (last_published_tf_stamps_.count(entry.first) && // 检查有没有这个轨迹ID
        last_published_tf_stamps_[entry.first] == stamped_transform.header.stamp) // 检查是否已发布过相同时间戳的TF
      continue;
    last_published_tf_stamps_[entry.first] = stamped_transform.header.stamp; // 更新最后发布时间记录

    const Rigid3d tracking_to_local_3d =
        node_options_.use_pose_extrapolator // 根据配置选择位姿来源
            ? extrapolator.ExtrapolatePose(now) // 使用预测器则调用 ExtrapolatePose，这个函数可能需要研究
            : trajectory_data.local_slam_data->local_pose; // 否则使用原始SLAM位姿
    const Rigid3d tracking_to_local = [&] { // tracking_to_local_3d在这次优化步骤后（不管有没有优化），会变为tracking_to_local
      if (trajectory_data.trajectory_options.publish_frame_projected_to_2d) { // 如果配置了publish_frame_projected_to_2d，还需要做2d投影处理
        return carto::transform::Embed3D( // Embed3D 重新嵌入为3D位姿
            carto::transform::Project2D(tracking_to_local_3d)); // Project2D 提取XY平面
      }
      return tracking_to_local_3d;
    }();
    // 全局位姿计算
    const Rigid3d tracking_to_map =
        trajectory_data.local_to_map * tracking_to_local; // local_to_map 是局部到全局的变换；tracking_to_local 是跟踪坐标系到局部的变换

    if (trajectory_data.published_to_tracking != nullptr) { // 检查是否存在有效的发布坐标系变换
      if (node_options_.publish_to_tf) { // publish_to_tf在node_options.h中默认为true
        // 情况1：提供 odom 坐标系（map→odom→base_link）
        if (trajectory_data.trajectory_options.provide_odom_frame) {
          std::vector<geometry_msgs::msg::TransformStamped> stamped_transforms; // 创建TF消息容器
          // 1. 发布 map→odom 变换
          stamped_transform.header.frame_id = node_options_.map_frame; // 地图坐标系(map)
          stamped_transform.child_frame_id =
              trajectory_data.trajectory_options.odom_frame; // 里程坐标系(local？)
          stamped_transform.transform =
              ToGeometryMsgTransform(trajectory_data.local_to_map); // ToGeometryMsgTransform需要看一下，好像是将Cartographer下的坐标变换转到ROS2下
          stamped_transforms.push_back(stamped_transform);
          // 2. 发布 odom→base_link 变换
          stamped_transform.header.frame_id =
              trajectory_data.trajectory_options.odom_frame; // 里程坐标系(local?)
          stamped_transform.child_frame_id =
              trajectory_data.trajectory_options.published_frame; // 发布坐标系(published)（看来一般是base_link或者base_footprint）
          stamped_transform.transform = ToGeometryMsgTransform(
              tracking_to_local * (*trajectory_data.published_to_tracking)); // tracking_to_local 跟踪→局部；published_to_tracking 发布→跟踪
          stamped_transforms.push_back(stamped_transform);
          // 3. 批量发送 TF 变换
          tf_broadcaster_->sendTransform(stamped_transforms);
        } 
        // 情况2：不提供 odom 坐标系（直接 map→base_link）
        else {
          stamped_transform.header.frame_id = node_options_.map_frame; // 地图坐标系
          stamped_transform.child_frame_id =
              trajectory_data.trajectory_options.published_frame; // 发布坐标系
          stamped_transform.transform = ToGeometryMsgTransform(
              tracking_to_map * (*trajectory_data.published_to_tracking));
          tf_broadcaster_->sendTransform(stamped_transform);
        }
      }

      // 发布全局坐标系位姿话题信息/tracked_pose
      if (node_options_.publish_tracked_pose) {
        ::geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.frame_id = node_options_.map_frame; // 坐标系固定为 "map"
        pose_msg.header.stamp = stamped_transform.header.stamp; // 时间戳与 TF 同步
        pose_msg.pose = ToGeometryMsgPose(tracking_to_map); // 机器人在地图中的位姿
        tracked_pose_publisher_->publish(pose_msg);
      }
    }
  }
}

void Node::PublishTrajectoryNodeList() {
  if (trajectory_node_list_publisher_->get_subscription_count() > 0) {
    absl::MutexLock lock(&mutex_);
    trajectory_node_list_publisher_->publish(
        map_builder_bridge_->GetTrajectoryNodeList(node_->now()));
  }
}

void Node::PublishLandmarkPosesList() {
  if (landmark_poses_list_publisher_->get_subscription_count() > 0) {
    absl::MutexLock lock(&mutex_);
    landmark_poses_list_publisher_->publish(
        map_builder_bridge_->GetLandmarkPosesList(node_->now()));
  }
}

void Node::PublishConstraintList() {
  if (constraint_list_publisher_->get_subscription_count() > 0) {
    absl::MutexLock lock(&mutex_);
    constraint_list_publisher_->publish(map_builder_bridge_->GetConstraintList(node_->now()));
  }
}

// 根据 TrajectoryOptions 中的配置，生成所有需要订阅的传感器话题的 ID。
std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>
Node::ComputeExpectedSensorIds(const TrajectoryOptions& options) const {
  using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
  using SensorType = SensorId::SensorType;
  std::set<SensorId> expected_topics;
  // Subscribe to all laser scan, multi echo laser scan, and point cloud topics.
  for (const std::string& topic :
       ComputeRepeatedTopicNames(kLaserScanTopic, options.num_laser_scans)) {
    expected_topics.insert(SensorId{SensorType::RANGE, topic});
  }
  for (const std::string& topic : ComputeRepeatedTopicNames(
           kMultiEchoLaserScanTopic, options.num_multi_echo_laser_scans)) {
    expected_topics.insert(SensorId{SensorType::RANGE, topic});
  }
  for (const std::string& topic :
       ComputeRepeatedTopicNames(kPointCloud2Topic, options.num_point_clouds)) {
    expected_topics.insert(SensorId{SensorType::RANGE, topic});
  }
  // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
  // required.
  if (node_options_.map_builder_options.use_trajectory_builder_3d() ||
      (node_options_.map_builder_options.use_trajectory_builder_2d() &&
       options.trajectory_builder_options.trajectory_builder_2d_options()
           .use_imu_data())) {
    expected_topics.insert(SensorId{SensorType::IMU, kImuTopic});
  }
  // Odometry is optional.
  if (options.use_odometry) {
    expected_topics.insert(SensorId{SensorType::ODOMETRY, kOdometryTopic});
  }
  // NavSatFix is optional.
  if (options.use_nav_sat) {
    expected_topics.insert(
        SensorId{SensorType::FIXED_FRAME_POSE, kNavSatFixTopic});
  }
  // Landmark is optional.
  if (options.use_landmarks) {
    expected_topics.insert(SensorId{SensorType::LANDMARK, kLandmarkTopic});
  }
  return expected_topics;
}

// 添加一条新的轨迹，输入轨迹配置选项，返回新添加的轨迹ID
int Node::AddTrajectory(const TrajectoryOptions& options) {
  const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId> // 计算期望的传感器ID
      expected_sensor_ids = ComputeExpectedSensorIds(options);
  const int trajectory_id = // 添加轨迹到地图构建器:通过接口map_builder_bridge_向Cartographer添加一条新的轨迹并获取轨迹的索引
      map_builder_bridge_->AddTrajectory(expected_sensor_ids, options);
  AddExtrapolator(trajectory_id, options); // 初始化外推器：用于位姿插值(PoseExtrapolator)
  AddSensorSamplers(trajectory_id, options); // 初始化传感器采样器：传感器采样(TrajectorySensorSamplers)
  LaunchSubscribers(options, trajectory_id); // 启动传感器数据订阅器
  maybe_warn_about_topic_mismatch_timer_ = node_->create_wall_timer( // 创建定时器定期检查实际订阅的话题与期望的话题是否匹配
    std::chrono::milliseconds(int(kTopicMismatchCheckDelaySec * 1000)),
    [this]() {
      MaybeWarnAboutTopicMismatch();
    });
  for (const auto& sensor_id : expected_sensor_ids) { // 记录订阅的话题
    subscribed_topics_.insert(sensor_id.id);
  }
  return trajectory_id; // 返回轨迹ID
}

// 根据配置选项（TrajectoryOptions）订阅多个传感器话题，并将接收到的数据传递给相应的处理函数
void Node::LaunchSubscribers(const TrajectoryOptions& options, // 输入：轨迹的配置选项 和 轨迹的ID
                             const int trajectory_id) {
  // 订阅激光雷达数据
  for (const std::string& topic :
       ComputeRepeatedTopicNames(kLaserScanTopic, options.num_laser_scans)) { // 函数ComputeRepeatedTopicNames是用来处理有多个相同类型的传感器
    subscribers_[trajectory_id].push_back( // subscribers_中保存的是构建的订阅器
        {SubscribeWithHandler<sensor_msgs::msg::LaserScan>(
             &Node::HandleLaserScanMessage, trajectory_id, topic, node_, this),
         topic});
  }
  // 订阅多回波激光雷达数据
  for (const std::string& topic : ComputeRepeatedTopicNames(
           kMultiEchoLaserScanTopic, options.num_multi_echo_laser_scans)) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::msg::MultiEchoLaserScan>(
             &Node::HandleMultiEchoLaserScanMessage, trajectory_id, topic, node_, this),
         topic});
  }
  // 订阅点云数据
  for (const std::string& topic :
       ComputeRepeatedTopicNames(kPointCloud2Topic, options.num_point_clouds)) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::msg::PointCloud2>(
             &Node::HandlePointCloud2Message, trajectory_id, topic, node_, this),
         topic});
  }

  // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
  // required.
  // 订阅IMU数据（2D可选，3D必须）
  if (node_options_.map_builder_options.use_trajectory_builder_3d() ||
      (node_options_.map_builder_options.use_trajectory_builder_2d() &&
       options.trajectory_builder_options.trajectory_builder_2d_options()
           .use_imu_data())) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::msg::Imu>(&Node::HandleImuMessage,
                                                trajectory_id, kImuTopic,
                                                node_, this),
         kImuTopic});
  }

  // 订阅里程计数据（可选）
  if (options.use_odometry) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<nav_msgs::msg::Odometry>(&Node::HandleOdometryMessage,
                                                  trajectory_id, kOdometryTopic,
                                                  node_, this),
         kOdometryTopic});
  }
  // 订阅导航卫星数据（可选）
  if (options.use_nav_sat) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::msg::NavSatFix>(
             &Node::HandleNavSatFixMessage, trajectory_id, kNavSatFixTopic,
             node_, this),
         kNavSatFixTopic});
  }
  // 订阅路标数据（可选）
  if (options.use_landmarks) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<cartographer_ros_msgs::msg::LandmarkList>(
             &Node::HandleLandmarkMessage, trajectory_id, kLandmarkTopic,
             node_, this),
         kLandmarkTopic});
  }
}

// ValidateTrajectoryOptions：验证轨迹配置
bool Node::ValidateTrajectoryOptions(const TrajectoryOptions& options) {
  if (node_options_.map_builder_options.use_trajectory_builder_2d()) {
    return options.trajectory_builder_options
        .has_trajectory_builder_2d_options();
  }
  if (node_options_.map_builder_options.use_trajectory_builder_3d()) {
    return options.trajectory_builder_options
        .has_trajectory_builder_3d_options();
  }
  return false;
}

// ValidateTopicNames：验证话题名称
bool Node::ValidateTopicNames(const TrajectoryOptions& options) {
  for (const auto& sensor_id : ComputeExpectedSensorIds(options)) {
    const std::string& topic = sensor_id.id;
    if (subscribed_topics_.count(topic) > 0) {
      LOG(ERROR) << "Topic name [" << topic << "] is already used.";
      return false;
    }
  }
  return true;
}

// TrajectoryStateToStatus：​验证指定轨迹的当前状态是否符合预期，​返回结构化的状态响应​​
cartographer_ros_msgs::msg::StatusResponse Node::TrajectoryStateToStatus(
    const int trajectory_id, const std::set<TrajectoryState>& valid_states) {
  // (1) 获取当前所有轨迹状态
  const auto trajectory_states = map_builder_bridge_->GetTrajectoryStates();
  cartographer_ros_msgs::msg::StatusResponse status_response;

  // (2) 检查轨迹是否存在
  const auto it = trajectory_states.find(trajectory_id);
  if (it == trajectory_states.end()) {
    status_response.message = "Trajectory " + std::to_string(trajectory_id) + " doesn't exist.";
    status_response.code = cartographer_ros_msgs::msg::StatusCode::NOT_FOUND;
    return status_response; // 轨迹不存在提前返回
  }

  // ​(3) 状态有效性验证
  status_response.message = "Trajectory " + std::to_string(trajectory_id) + " is in '" +
    TrajectoryStateToString(it->second) + "' state.";
  status_response.code =
      valid_states.count(it->second)
          ? cartographer_ros_msgs::msg::StatusCode::OK
          : cartographer_ros_msgs::msg::StatusCode::INVALID_ARGUMENT;
  return status_response; // 返回有效性验证结果
}

// FinishTrajectoryUnderLock：安全终止指定轨迹
cartographer_ros_msgs::msg::StatusResponse Node::FinishTrajectoryUnderLock(
    const int trajectory_id) {
  cartographer_ros_msgs::msg::StatusResponse status_response;
  // 1.避免重复检查
  if (trajectories_scheduled_for_finish_.count(trajectory_id)) {
    status_response.message =
        "Trajectory " + std::to_string(trajectory_id) + " already pending to finish.";
    status_response.code = cartographer_ros_msgs::msg::StatusCode::OK;
    LOG(INFO) << status_response.message;
    return status_response;
  }

  // 2.检查是否可以终止轨迹
  status_response = TrajectoryStateToStatus(
      trajectory_id, {TrajectoryState::ACTIVE} /* valid states */);
  if (status_response.code != cartographer_ros_msgs::msg::StatusCode::OK) {
    LOG(ERROR) << "Can't finish trajectory: " << status_response.message;
    return status_response;
  }

  // Shutdown the subscribers of this trajectory.
  // A valid case with no subscribers is e.g. if we just visualize states.
  // 3.关闭轨迹的订阅者，释放资源
  if (subscribers_.count(trajectory_id)) {
    for (auto& entry : subscribers_[trajectory_id]) {
      entry.subscriber.reset();
      subscribed_topics_.erase(entry.topic);
      LOG(INFO) << "Shutdown the subscriber of [" << entry.topic << "]";
    }
    CHECK_EQ(subscribers_.erase(trajectory_id), 1);
  }
  // 4.核心终止操作
  map_builder_bridge_->FinishTrajectory(trajectory_id);
  trajectories_scheduled_for_finish_.emplace(trajectory_id);
  status_response.message =
      "Finished trajectory " + std::to_string(trajectory_id) + ".";
  status_response.code = cartographer_ros_msgs::msg::StatusCode::OK;
  return status_response;
}

// handleStartTrajectory："/start_trajectory"的服务响应函数，用于处理启动新轨迹的请求。返回bool表示是否处理成功。输入参数为 请求信息 和 启动轨迹的结果
bool Node::handleStartTrajectory(
    const cartographer_ros_msgs::srv::StartTrajectory::Request::SharedPtr request,
    cartographer_ros_msgs::srv::StartTrajectory::Response::SharedPtr response) {
  // （1）加载轨迹配置
  TrajectoryOptions trajectory_options;
  std::tie(std::ignore, trajectory_options) = LoadOptions(
      request->configuration_directory, request->configuration_basename); // 从指定路径加载轨迹配置文件

  // （2）处理初始位姿
  if (request->use_initial_pose) {
    const auto pose = ToRigid3d(request->initial_pose); // 将初始姿态转换到Rigid3d下
    // 2.1 验证位姿有效性
    if (!pose.IsValid()) { 
      response->status.message =
          "Invalid pose argument. Orientation quaternion must be normalized.";
      LOG(ERROR) << response->status.message;
      response->status.code =
          cartographer_ros_msgs::msg::StatusCode::INVALID_ARGUMENT;
      return true;
    }

    // 2.2 验证参考轨迹是否存在
    response->status = TrajectoryStateToStatus(
        request->relative_to_trajectory_id,
        {TrajectoryState::ACTIVE, TrajectoryState::FROZEN,
         TrajectoryState::FINISHED} /* valid states */);
    if (response->status.code != cartographer_ros_msgs::msg::StatusCode::OK) {
      LOG(ERROR) << "Can't start a trajectory with initial pose: "
                 << response->status.message;
      return true;
    }

    // 2.3 设置初始位姿到protobuf消息
    // 以下代码作用：配置一个初始轨迹位姿，用于SLAM系统（特别是纯定位模式）的初始化
    // 1.创建InitialTrajectoryPose对象​​：创建了一个protobuf消息对象，用于存储初始轨迹位姿信息。
    ::cartographer::mapping::proto::InitialTrajectoryPose initial_trajectory_pose;
    // ​2.​设置相对轨迹ID​​：设置这个初始位姿相对于哪个已有轨迹的ID（在纯定位模式中，这通常是已建图轨迹的ID）
    initial_trajectory_pose.set_to_trajectory_id(request->relative_to_trajectory_id);
    // 3.​设置相对位姿​​：将ROS格式的位姿(pose)转换为Cartographer内部使用的protobuf格式，并设置为相对位姿。
    *initial_trajectory_pose.mutable_relative_pose() = cartographer::transform::ToProto(pose);
    // 4.设置时间戳​​：设置时间戳，这里使用了0时间（rclcpp::Time(0)），表示立即使用这个初始位姿。
    initial_trajectory_pose.set_timestamp(cartographer::common::ToUniversal(::cartographer_ros::FromRos(rclcpp::Time(0))));
    // 5.将初始位姿设置到轨迹选项中​​
    *trajectory_options.trajectory_builder_options.mutable_initial_trajectory_pose() = initial_trajectory_pose; // 使用protobuf格式存入轨迹选项
  }

  // （3）验证轨迹配置，通过验证的话添加轨迹
  if (!ValidateTrajectoryOptions(trajectory_options)) { // 如果轨迹配置无效
    response->status.message = "Invalid trajectory options.";
    LOG(ERROR) << response->status.message;
    response->status.code = cartographer_ros_msgs::msg::StatusCode::INVALID_ARGUMENT;
  } else if (!ValidateTopicNames(trajectory_options)) { // 如果话题名无效
    response->status.message = "Topics are already used by another trajectory.";
    LOG(ERROR) << response->status.message;
    response->status.code = cartographer_ros_msgs::msg::StatusCode::INVALID_ARGUMENT;
  } else { // 有效的情况下，添加轨迹
    response->status.message = "Success.";
    response->trajectory_id = AddTrajectory(trajectory_options); // AddTrajectory
    response->status.code = cartographer_ros_msgs::msg::StatusCode::OK;
  }
  return true;
}

// StartTrajectoryWithDefaultTopics：使用系统默认的订阅话题开始轨迹跟踪
void Node::StartTrajectoryWithDefaultTopics(const TrajectoryOptions& options) {
  absl::MutexLock lock(&mutex_); // 加锁
  CHECK(ValidateTrajectoryOptions(options)); // 检查配置是否合法
  AddTrajectory(options); // 开始轨迹跟踪
}

std::vector<
    std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>>
Node::ComputeDefaultSensorIdsForMultipleBags(
    const std::vector<TrajectoryOptions>& bags_options) const {
  using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
  std::vector<std::set<SensorId>> bags_sensor_ids;
  for (size_t i = 0; i < bags_options.size(); ++i) {
    std::string prefix;
    if (bags_options.size() > 1) {
      prefix = "bag_" + std::to_string(i + 1) + "_";
    }
    std::set<SensorId> unique_sensor_ids;
    for (const auto& sensor_id : ComputeExpectedSensorIds(bags_options.at(i))) {
      unique_sensor_ids.insert(SensorId{sensor_id.type, prefix + sensor_id.id});
    }
    bags_sensor_ids.push_back(unique_sensor_ids);
  }
  return bags_sensor_ids;
}

int Node::AddOfflineTrajectory(
    const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>&
        expected_sensor_ids,
    const TrajectoryOptions& options) {
  absl::MutexLock lock(&mutex_);
  const int trajectory_id =
      map_builder_bridge_->AddTrajectory(expected_sensor_ids, options);
  AddExtrapolator(trajectory_id, options);
  AddSensorSamplers(trajectory_id, options);
  return trajectory_id;
}

bool Node::handleGetTrajectoryStates(
    const cartographer_ros_msgs::srv::GetTrajectoryStates::Request::SharedPtr ,
    cartographer_ros_msgs::srv::GetTrajectoryStates::Response::SharedPtr response) {

  using TrajectoryState =
      ::cartographer::mapping::PoseGraphInterface::TrajectoryState;
  absl::MutexLock lock(&mutex_);
  response->status.code = ::cartographer_ros_msgs::msg::StatusCode::OK;
  response->trajectory_states.header.stamp = node_->now();
  for (const auto& entry : map_builder_bridge_->GetTrajectoryStates()) {
    response->trajectory_states.trajectory_id.push_back(entry.first);
    switch (entry.second) {
      case TrajectoryState::ACTIVE:
        response->trajectory_states.trajectory_state.push_back(
            ::cartographer_ros_msgs::msg::TrajectoryStates::ACTIVE);
        break;
      case TrajectoryState::FINISHED:
        response->trajectory_states.trajectory_state.push_back(
            ::cartographer_ros_msgs::msg::TrajectoryStates::FINISHED);
        break;
      case TrajectoryState::FROZEN:
        response->trajectory_states.trajectory_state.push_back(
            ::cartographer_ros_msgs::msg::TrajectoryStates::FROZEN);
        break;
      case TrajectoryState::DELETED:
        response->trajectory_states.trajectory_state.push_back(
            ::cartographer_ros_msgs::msg::TrajectoryStates::DELETED);
        break;
    }
  }
  return true;
}

// handleFinishTrajectory："/finish_trajectory"的服务响应函数
bool Node::handleFinishTrajectory(
    const cartographer_ros_msgs::srv::FinishTrajectory::Request::SharedPtr request,
    cartographer_ros_msgs::srv::FinishTrajectory::Response::SharedPtr response) {
  absl::MutexLock lock(&mutex_);
  response->status = FinishTrajectoryUnderLock(request->trajectory_id); // 调用函数FinishTrajectoryUnderLock停止路径跟踪
  return true;
}

bool Node::handleWriteState(
    const cartographer_ros_msgs::srv::WriteState::Request::SharedPtr request,
    cartographer_ros_msgs::srv::WriteState::Response::SharedPtr response) {
  absl::MutexLock lock(&mutex_);
  if (map_builder_bridge_->SerializeState(request->filename,
                                         request->include_unfinished_submaps)) {
    response->status.code = cartographer_ros_msgs::msg::StatusCode::OK;
    response->status.message =
        "State written to '" + request->filename + "'.";
  } else {
    response->status.code = cartographer_ros_msgs::msg::StatusCode::INVALID_ARGUMENT;
    response->status.message =
        "Failed to write '" + request->filename + "'.";
  }
  return true;
}

bool Node::handleReadMetrics(
    const cartographer_ros_msgs::srv::ReadMetrics::Request::SharedPtr,
    cartographer_ros_msgs::srv::ReadMetrics::Response::SharedPtr response) {

  absl::MutexLock lock(&mutex_);
  response->timestamp = node_->now();
  if (!metrics_registry_) {
    response->status.code = cartographer_ros_msgs::msg::StatusCode::UNAVAILABLE;
    response->status.message = "Collection of runtime metrics is not activated.";
    return true;
  }
  metrics_registry_->ReadMetrics(response);
  response->status.code = cartographer_ros_msgs::msg::StatusCode::OK;
  response->status.message = "Successfully read metrics.";
  return true;
}

// 析构函数会调用FinishAllTrajectories
void Node::FinishAllTrajectories() {
  absl::MutexLock lock(&mutex_);
  for (const auto& entry : map_builder_bridge_->GetTrajectoryStates()) {
    if (entry.second == TrajectoryState::ACTIVE) {
      const int trajectory_id = entry.first;
      CHECK_EQ(FinishTrajectoryUnderLock(trajectory_id).code,
               cartographer_ros_msgs::msg::StatusCode::OK);
    }
  }
}

bool Node::FinishTrajectory(const int trajectory_id) {
  absl::MutexLock lock(&mutex_);
  return FinishTrajectoryUnderLock(trajectory_id).code ==
         cartographer_ros_msgs::msg::StatusCode::OK;
}

void Node::RunFinalOptimization() {
  {
    for (const auto& entry : map_builder_bridge_->GetTrajectoryStates()) {
      const int trajectory_id = entry.first;
      if (entry.second == TrajectoryState::ACTIVE) {
        LOG(WARNING)
            << "Can't run final optimization if there are one or more active "
               "trajectories. Trying to finish trajectory with ID "
            << std::to_string(trajectory_id) << " now.";
        CHECK(FinishTrajectory(trajectory_id))
            << "Failed to finish trajectory with ID "
            << std::to_string(trajectory_id) << ".";
      }
    }
  }
  // Assuming we are not adding new data anymore, the final optimization
  // can be performed without holding the mutex.
  map_builder_bridge_->RunFinalOptimization();
}

// 以下为各种传感器的订阅回调函数
void Node::HandleOdometryMessage(const int trajectory_id,
                                 const std::string& sensor_id,
                                 const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).odometry_sampler.Pulse()) {
    return;
  }
  auto sensor_bridge_ptr = map_builder_bridge_->sensor_bridge(trajectory_id);
  auto odometry_data_ptr = sensor_bridge_ptr->ToOdometryData(msg);
  if (odometry_data_ptr != nullptr) {
    extrapolators_.at(trajectory_id).AddOdometryData(*odometry_data_ptr);
  }
  sensor_bridge_ptr->HandleOdometryMessage(sensor_id, msg);
}

void Node::HandleNavSatFixMessage(const int trajectory_id,
                                  const std::string& sensor_id,
                                  const sensor_msgs::msg::NavSatFix::ConstSharedPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).fixed_frame_pose_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_->sensor_bridge(trajectory_id)
      ->HandleNavSatFixMessage(sensor_id, msg);
}

void Node::HandleLandmarkMessage(
    const int trajectory_id, const std::string& sensor_id,
    const cartographer_ros_msgs::msg::LandmarkList::ConstSharedPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).landmark_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_->sensor_bridge(trajectory_id)
      ->HandleLandmarkMessage(sensor_id, msg);
}

// IMU的消息回调函数
void Node::HandleImuMessage(const int trajectory_id,
                            const std::string& sensor_id,
                            const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).imu_sampler.Pulse()) { // 通过采样器对传感器的数据进行降采样
    return;
  }
  auto sensor_bridge_ptr = map_builder_bridge_->sensor_bridge(trajectory_id); // 通过map_builder_bridge_将传感器数据喂给Cartographer
  auto imu_data_ptr = sensor_bridge_ptr->ToImuData(msg);
  if (imu_data_ptr != nullptr) {
    extrapolators_.at(trajectory_id).AddImuData(*imu_data_ptr); // 将IMU数据喂给位姿估计器（相比激光雷达的额外步骤）
  }
  sensor_bridge_ptr->HandleImuMessage(sensor_id, msg);
}

void Node::HandleLaserScanMessage(const int trajectory_id,
                                  const std::string& sensor_id,
                                  const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_->sensor_bridge(trajectory_id)
      ->HandleLaserScanMessage(sensor_id, msg);
}

// 多回波激光传感器的消息回调函数
void Node::HandleMultiEchoLaserScanMessage(
    const int trajectory_id, const std::string& sensor_id,
    const sensor_msgs::msg::MultiEchoLaserScan::ConstSharedPtr& msg) {
  absl::MutexLock lock(&mutex_); // 加锁
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) { // 通过采样器对传感器的数据进行降采样
    return;
  }
  map_builder_bridge_->sensor_bridge(trajectory_id) // 通过map_builder_bridge_将传感器数据喂给Cartographer
      ->HandleMultiEchoLaserScanMessage(sensor_id, msg); // 这里的HandleMultiEchoLaserScanMessage是SensorBridge类的成员函数，负责将ROS2消息转换为Cartographer内部数据格式
}

void Node::HandlePointCloud2Message(
    const int trajectory_id, const std::string& sensor_id,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_->sensor_bridge(trajectory_id)
      ->HandlePointCloud2Message(sensor_id, msg);
}


void Node::SerializeState(const std::string& filename,
                          const bool include_unfinished_submaps) {
  absl::MutexLock lock(&mutex_);
  CHECK(
      map_builder_bridge_->SerializeState(filename, include_unfinished_submaps))
      << "Could not write state.";
}

// LoadState：读取pbstream文件存储的状态
void Node::LoadState(const std::string& state_filename,
                     const bool load_frozen_state) {
  absl::MutexLock lock(&mutex_);
  map_builder_bridge_->LoadState(state_filename, load_frozen_state);
}

// TODO: find ROS equivalent to ros::master::getTopics
void Node::MaybeWarnAboutTopicMismatch() {
//  ::ros::master::V_TopicInfo ros_topics;
//  ::ros::master::getTopics(ros_topics);
//  std::set<std::string> published_topics;
//  std::stringstream published_topics_string;
//  for (const auto& it : ros_topics) {
//    std::string resolved_topic = node_handle_.resolveName(it.name, false);
//    published_topics.insert(resolved_topic);
//    published_topics_string << resolved_topic << ",";
//  }
//  bool print_topics = false;
//  for (const auto& entry : subscribers_) {
//    int trajectory_id = entry.first;
//    for (const auto& subscriber : entry.second) {
//      std::string resolved_topic = node_handle_.resolveName(subscriber.topic);
//      if (published_topics.count(resolved_topic) == 0) {
//        LOG(WARNING) << "Expected topic \"" << subscriber.topic
//                     << "\" (trajectory " << trajectory_id << ")"
//                     << " (resolved topic \"" << resolved_topic << "\")"
//                     << " but no publisher is currently active.";
//        print_topics = true;
//      }
//    }
//  }
//  if (print_topics) {
//    LOG(WARNING) << "Currently available topics are: "
//                 << published_topics_string.str();
//  }
}

}  // namespace cartographer_ros
