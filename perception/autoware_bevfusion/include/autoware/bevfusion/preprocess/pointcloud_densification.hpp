// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE__BEVFUSION__PREPROCESS__POINTCLOUD_DENSIFICATION_HPP_
#define AUTOWARE__BEVFUSION__PREPROCESS__POINTCLOUD_DENSIFICATION_HPP_

#include "autoware/bevfusion/preprocess/point_type.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>
#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <cuda_blackboard/cuda_pointcloud2.hpp>

#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cstddef>
#include <list>
#include <memory>
#include <string>
#include <utility>

namespace autoware::bevfusion
{

using autoware::cuda_utils::CudaUniquePtr;

class DensificationParam
{
public:
  DensificationParam(const std::string & world_frame_id, const unsigned int num_past_frames)
  : world_frame_id_(std::move(world_frame_id)),
    pointcloud_cache_size_(num_past_frames + /*current frame*/ 1)
  {
  }

  std::string getWorldFrameId() const { return world_frame_id_; }
  unsigned int getPointcloudCacheSize() const { return pointcloud_cache_size_; }

private:
  std::string world_frame_id_;
  unsigned int pointcloud_cache_size_{1};
};

struct PointCloudWithTransform
{
  std::shared_ptr<const cuda_blackboard::CudaPointCloud2> input_pointcloud_msg_ptr;
  Eigen::Affine3f affine_past2world;
};

class PointCloudDensification
{
public:
  explicit PointCloudDensification(const DensificationParam & param);

  bool enqueuePointCloud(
    const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & msg_ptr,
    const tf2_ros::Buffer & tf_buffer);

  double getCurrentTimestamp() const { return current_timestamp_; }
  Eigen::Affine3f getAffineWorldToCurrent() const { return affine_world2current_; }
  std::list<PointCloudWithTransform>::iterator getPointCloudCacheIter()
  {
    return pointcloud_cache_.begin();
  }
  bool isCacheEnd(std::list<PointCloudWithTransform>::iterator iter)
  {
    return iter == pointcloud_cache_.end();
  }
  std::size_t getIdx(std::list<PointCloudWithTransform>::iterator iter)
  {
    return std::distance(pointcloud_cache_.begin(), iter);
  }
  std::size_t getCacheSize()
  {
    return std::distance(pointcloud_cache_.begin(), pointcloud_cache_.end());
  }
  unsigned int getPointcloudCacheSize() const { return param_.getPointcloudCacheSize(); }

private:
  void enqueue(
    const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & msg_ptr,
    const Eigen::Affine3f & affine);
  void dequeue();

  DensificationParam param_;
  double current_timestamp_{0.0};
  Eigen::Affine3f affine_world2current_;
  std::list<PointCloudWithTransform> pointcloud_cache_;
};

}  // namespace autoware::bevfusion

#endif  // AUTOWARE__BEVFUSION__PREPROCESS__POINTCLOUD_DENSIFICATION_HPP_
