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

#include "absl/memory/memory.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"
#include "tf2_ros/transform_listener.h"

// cartographer_node使用gflags解析运行参数，以下通过gflags的DEFINE_type形式的宏来定义参数对象
DEFINE_bool(collect_metrics, false, // 是否收集运行时的性能指标
            "Activates the collection of runtime metrics. If activated, the "
            "metrics can be accessed via a ROS service.");
DEFINE_string(configuration_directory, "", // 指定配置文件的搜索目录
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "", // 指定配置文件的名称
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(load_state_filename, "", // 指定加载 SLAM 状态的文件路径
              "If non-empty, filename of a .pbstream file to load, containing "
              "a saved SLAM state.");
DEFINE_bool(load_frozen_state, true, // 是否以冻结状态加载 SLAM 状态
            "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_bool(
    start_trajectory_with_default_topics, true, // 是否使用默认的 ROS 话题启动第一个轨迹
    "Enable to immediately start the first trajectory with default topics.");
DEFINE_string(
    save_state_filename, "", // 指定保存 SLAM 状态的文件路径
    "If non-empty, serialize state and write it to disk before shutting down.");

namespace cartographer_ros {
namespace {

// Run函数：完成所有的定位和建图的逻辑
void Run() {
  rclcpp::Node::SharedPtr cartographer_node = rclcpp::Node::make_shared("cartographer_node"); // 创建一个 ROS 2 节点，节点名称为 cartographer_node
  
  // 初始化 TF 缓冲区：用于存储和查询坐标变换
  constexpr double kTfBufferCacheTimeInSeconds = 10.; // TF 缓冲区的缓存时间（10 秒）
  std::shared_ptr<tf2_ros::Buffer> tf_buffer =
      std::make_shared<tf2_ros::Buffer>(
        cartographer_node->get_clock(), // 使用节点的时钟
        tf2::durationFromSec(kTfBufferCacheTimeInSeconds),
        cartographer_node);

  // 初始化 TF 监听器：用于订阅 TF 数据并填充 TF 缓冲区
  std::shared_ptr<tf2_ros::TransformListener> tf_listener =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  // 加载配置选项
  NodeOptions node_options;
  TrajectoryOptions trajectory_options;
  std::tie(node_options, trajectory_options) =
      LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);

  // ​创建地图构建器：用于管理 SLAM 的核心逻辑（如地图构建和优化）
  auto map_builder =
    cartographer::mapping::CreateMapBuilder(node_options.map_builder_options);

  // 创建 Cartographer ROS 节点
  auto node = std::make_shared<cartographer_ros::Node>(
    node_options, std::move(map_builder), tf_buffer, cartographer_node,
    FLAGS_collect_metrics);
  // 加载pbstream文件获取SLAM状态
  if (!FLAGS_load_state_filename.empty()) { // 如果指定了状态文件，则从文件中加载 SLAM 状态
    node->LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state); // 加载pbstream文件
  }
  // 使用默认话题开始轨迹
  if (FLAGS_start_trajectory_with_default_topics) { // 启动默认轨迹,该选项默认为true
    node->StartTrajectoryWithDefaultTopics(trajectory_options);
  }

  rclcpp::spin(cartographer_node); // 启动cartographer_node

  // 如果在ROS的逻辑循环过程中，触发了退出机制，Run函数就会接着执行如下的语句，完成最终的路径和地图的优化
  node->FinishAllTrajectories();
  node->RunFinalOptimization();

  // 如果运行参数要求保存系统状态，则将当前的系统状态存到参数save_state_filename所指定的文件中
  if (!FLAGS_save_state_filename.empty()) {
    node->SerializeState(FLAGS_save_state_filename,
                        true /* include_unfinished_submaps */);
  }
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  // Init rclcpp first because gflags reorders command line flags in argv
  rclcpp::init(argc, argv);

  google::AllowCommandLineReparsing();
  google::InitGoogleLogging(argv[0]); // 初始化了谷歌的日志系统glog
  google::ParseCommandLineFlags(&argc, &argv, false); // 通过gflags解析运行cartographer_node时的运行参数

  // 检查是否在运行参数中指定了配置文件和目录，若没有程序会报错退出
  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";

  cartographer_ros::ScopedRosLogSink ros_log_sink; // 将 Cartographer 内部的日志（通常使用 glog 库）重定向到 ROS 的日志系统（如 ros::console）
  cartographer_ros::Run(); //开始进行定位建图
  ::rclcpp::shutdown();
}
