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

#include "cartographer/mapping/2d/probability_grid_range_data_inserter_2d.h"

#include <cstdlib>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/mapping/2d/xy_index.h"
#include "cartographer/mapping/internal/2d/ray_to_pixel_mask.h"
#include "cartographer/mapping/probability_values.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace {

// Factor for subpixel accuracy of start and end point for ray casts.
constexpr int kSubpixelScale = 1000;

// GrowAsNeeded：检查当前概率网格是否需要扩展以容纳新的扫描数据，如果扫描数据超出当前网格范围，会自动扩展网格
void GrowAsNeeded(const sensor::RangeData& range_data,
                  ProbabilityGrid* const probability_grid) {
  Eigen::AlignedBox2f bounding_box(range_data.origin.head<2>());
  // Padding around bounding box to avoid numerical issues at cell boundaries.
  constexpr float kPadding = 1e-6f;
  for (const sensor::RangefinderPoint& hit : range_data.returns) {
    bounding_box.extend(hit.position.head<2>());
  }
  for (const sensor::RangefinderPoint& miss : range_data.misses) {
    bounding_box.extend(miss.position.head<2>());
  }
  probability_grid->GrowLimits(bounding_box.min() -
                               kPadding * Eigen::Vector2f::Ones());
  probability_grid->GrowLimits(bounding_box.max() +
                               kPadding * Eigen::Vector2f::Ones());
}

// CastRays：处理激光雷达数据并更新概率网格地图的核心函数
void CastRays(const sensor::RangeData& range_data, // 包含激光雷达扫描数据的结构体，包括原点(origin)、返回点(returns)和未命中点(misses)
              const std::vector<uint16>& hit_table, // 存储的hit网格概率表
              const std::vector<uint16>& miss_table, // 存储的miss网格概率表
              const bool insert_free_space, ProbabilityGrid* probability_grid) { // 需要更新的概率网格地图指针
  GrowAsNeeded(range_data, probability_grid); // 检查当前概率网格是否需要扩展以容纳新的扫描数据，如果扫描数据超出当前网格范围，会自动扩展网格

  const MapLimits& limits = probability_grid->limits(); // 获取栅格地图的 limits
  const double superscaled_resolution = limits.resolution() / kSubpixelScale; // 定义一个超分辨率像素，把当前的分辨率又划分成了 kSubpixelScale=1000 份，提高射线追踪的精度，减少锯齿效应
  const MapLimits superscaled_limits( // 根据超分辨率像素又生成了一个新的 MapLimits
      superscaled_resolution, limits.max(),
      CellLimits(limits.cell_limits().num_x_cells * kSubpixelScale, //划分格数变成了 kSubpixelScale 倍，分辨率变小以后，网格数自然变多
                 limits.cell_limits().num_y_cells * kSubpixelScale));
  // 获取传感器原点（range_data.origin前两项）在超分辨率网格中的坐标 begin。该坐标是我们所求的射线的原点。
  const Eigen::Array2i begin =
      superscaled_limits.GetCellIndex(range_data.origin.head<2>());
  // Compute and add the end points.
  std::vector<Eigen::Array2i> ends; // 定义一个向量集合ends，该集合存储 RangeData 中的 hits 的点在超分辨率网格坐标系下的坐标
  ends.reserve(range_data.returns.size()); // 根据 returns 集合的大小，给 ends 预分配一块存储区。reserve 函数用来给 vector 预分配存储区大小，但是没有给这段内存进行初始化
  for (const sensor::RangefinderPoint& hit : range_data.returns) {
    // 遍历 returns 这个集合
    ends.push_back(superscaled_limits.GetCellIndex(hit.position.head<2>())); // 把每个点转换到超分辨率网格坐标系后，压入 ends 中
    probability_grid->ApplyLookupTable(ends.back() / kSubpixelScale, hit_table); // 这里是在处理hit的概率表
  }

  if (!insert_free_space) { // 如果配置项里设置是不考虑 free space。那么函数到这里结束，只处理完 hit 后返回即可
    return;
  }

  // Now add the misses.
  // 否则的话，需要计算两种 miss 的情况
  // 1.原点 begin 到 占用点 end 之间记作 miss，需要更新栅格地图
  for (const Eigen::Array2i& end : ends) { // 遍历超分辨率坐标系下的 hit 坐标
    std::vector<Eigen::Array2i> ray =
        RayToPixelMask(begin, end, kSubpixelScale); // RayToPixelMask函数计算从begin到end的射线路径上的所有网格单元
    for (const Eigen::Array2i& cell_index : ray) {
      probability_grid->ApplyLookupTable(cell_index, miss_table);
    }
  }

  // Finally, compute and add empty rays based on misses in the range data.
  // 2.range_data中原本就丢失的激光点也需要更新栅格地图
  for (const sensor::RangefinderPoint& missing_echo : range_data.misses) {
    std::vector<Eigen::Array2i> ray = RayToPixelMask( // 将原本就miss的点转换到超分辨率坐标系下
        begin, superscaled_limits.GetCellIndex(missing_echo.position.head<2>()),
        kSubpixelScale);
    for (const Eigen::Array2i& cell_index : ray) {
      probability_grid->ApplyLookupTable(cell_index, miss_table); // 更新miss_table
    }
  }
}
}  // namespace

// 配置函数
proto::ProbabilityGridRangeDataInserterOptions2D
CreateProbabilityGridRangeDataInserterOptions2D(
    common::LuaParameterDictionary* parameter_dictionary) {
  proto::ProbabilityGridRangeDataInserterOptions2D options;
  options.set_hit_probability(
      parameter_dictionary->GetDouble("hit_probability"));
  options.set_miss_probability(
      parameter_dictionary->GetDouble("miss_probability"));
  options.set_insert_free_space(
      parameter_dictionary->HasKey("insert_free_space")
          ? parameter_dictionary->GetBool("insert_free_space")
          : true);
  CHECK_GT(options.hit_probability(), 0.5);
  CHECK_LT(options.miss_probability(), 0.5);
  return options;
}

ProbabilityGridRangeDataInserter2D::ProbabilityGridRangeDataInserter2D(
    const proto::ProbabilityGridRangeDataInserterOptions2D& options)
    : options_(options),
      hit_table_(ComputeLookupTableToApplyCorrespondenceCostOdds(
          Odds(options.hit_probability()))),
      miss_table_(ComputeLookupTableToApplyCorrespondenceCostOdds(
          Odds(options.miss_probability()))) {}

// 二维建图传感器数据帧更新地图的Insert函数的具体实现
void ProbabilityGridRangeDataInserter2D::Insert(
    const sensor::RangeData& range_data, GridInterface* const grid) const {
  ProbabilityGrid* const probability_grid = static_cast<ProbabilityGrid*>(grid); // 将 Grid 类型强制转化为 ProbabilityGrid 类型
  CHECK(probability_grid != nullptr);
  // By not finishing the update after hits are inserted, we give hits priority
  // (i.e. no hits will be ignored because of a miss in the same cell).
  // CastRays定义在/mapping/internal/2d/ray_casting.h中
  // CastRay 中就是把 RangeData 中包含的一系列点 ，计算出一条从原点到 的射线，射线端点处的点是 Hit，射线中间的点是 Free。把所有这些点要在地图上把相应的 cell 进行更新。
  CastRays(range_data, hit_table_, miss_table_, options_.insert_free_space(), // 调用 CastRays 函数更新 Grid
           probability_grid);
  probability_grid->FinishUpdate();
}

}  // namespace mapping
}  // namespace cartographer
