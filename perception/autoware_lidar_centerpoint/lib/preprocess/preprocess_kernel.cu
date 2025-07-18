// Copyright 2022 TIER IV, Inc.
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
/*
 * SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "autoware/lidar_centerpoint/cuda_utils.hpp"
#include "autoware/lidar_centerpoint/preprocess/point_type.hpp"
#include "autoware/lidar_centerpoint/preprocess/preprocess_kernel.hpp"
#include "autoware/lidar_centerpoint/utils.hpp"

#include <cassert>
#include <cmath>

namespace
{
const std::size_t MAX_POINT_IN_VOXEL_SIZE = 32;  // the same as max_point_in_voxel_size_ in config
const std::size_t WARPS_PER_BLOCK = 4;

const std::size_t POINT_DIM_XYZT = 4;   // X, Y, Z, Time_lag
const std::size_t POINT_DIM_XYZIT = 5;  // X, Y, Z, Intensity, Time_lag
const std::size_t ENCODER_NUM_FEATURES_9 = 9;
const std::size_t ENCODER_NUM_FEATURES_10 = 10;
const std::size_t ENCODER_NUM_FEATURES_11 = 11;
}  // namespace

namespace autoware::lidar_centerpoint
{
PreprocessCuda::PreprocessCuda(const CenterPointConfig & config, cudaStream_t & stream)
: config_(config), stream_(stream)
{
}

template <std::size_t POINT_NUM_FEATURES>
__global__ void generateSweepPoints_kernel(
  const InputPointType * __restrict__ input_points, std::size_t points_size, float time_lag,
  const float * transform_array, float * __restrict__ output_points)
{
  int point_idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (point_idx >= points_size) return;

  const InputPointType * input_point = &input_points[point_idx];
  float input_x = input_point->x;
  float input_y = input_point->y;
  float input_z = input_point->z;

  output_points[point_idx * POINT_NUM_FEATURES] =
    transform_array[0] * input_x + transform_array[4] * input_y + transform_array[8] * input_z +
    transform_array[12];
  output_points[point_idx * POINT_NUM_FEATURES + 1] =
    transform_array[1] * input_x + transform_array[5] * input_y + transform_array[9] * input_z +
    transform_array[13];
  output_points[point_idx * POINT_NUM_FEATURES + 2] =
    transform_array[2] * input_x + transform_array[6] * input_y + transform_array[10] * input_z +
    transform_array[14];

  // Time_lag
  if (POINT_NUM_FEATURES == POINT_DIM_XYZT) {
    output_points[point_idx * POINT_NUM_FEATURES + 3] = time_lag;
    // Intensity & time_lag
  } else {
    auto input_intensity = static_cast<float>(input_point->intensity);
    output_points[point_idx * POINT_NUM_FEATURES + 3] = input_intensity;
    output_points[point_idx * POINT_NUM_FEATURES + 4] = time_lag;
  }
}

template <std::size_t POINT_NUM_FEATURES>
__global__ void shufflePoints_kernel(
  const float * points, const unsigned int * indices, float * shuffled_points,
  const std::size_t points_size, const std::size_t max_size, const std::size_t offset)
{
  int point_idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (point_idx >= max_size) return;

  int src_idx = indices[(point_idx + offset) % max_size];
  int dst_idx = point_idx;

  if (dst_idx >= points_size) {
    shuffled_points[POINT_NUM_FEATURES * dst_idx + 0] = INFINITY;
    shuffled_points[POINT_NUM_FEATURES * dst_idx + 1] = INFINITY;
    shuffled_points[POINT_NUM_FEATURES * dst_idx + 2] = INFINITY;
    shuffled_points[POINT_NUM_FEATURES * dst_idx + 3] = INFINITY;
    if (POINT_NUM_FEATURES == POINT_DIM_XYZIT) {
      shuffled_points[POINT_NUM_FEATURES * dst_idx + 4] = INFINITY;
    }
  } else {
    shuffled_points[POINT_NUM_FEATURES * dst_idx + 0] = points[POINT_NUM_FEATURES * src_idx + 0];
    shuffled_points[POINT_NUM_FEATURES * dst_idx + 1] = points[POINT_NUM_FEATURES * src_idx + 1];
    shuffled_points[POINT_NUM_FEATURES * dst_idx + 2] = points[POINT_NUM_FEATURES * src_idx + 2];
    shuffled_points[POINT_NUM_FEATURES * dst_idx + 3] = points[POINT_NUM_FEATURES * src_idx + 3];
    if (POINT_NUM_FEATURES == POINT_DIM_XYZIT) {
      shuffled_points[POINT_NUM_FEATURES * dst_idx + 4] = points[POINT_NUM_FEATURES * src_idx + 4];
    }
  }
}

template <std::size_t POINT_NUM_FEATURES>
__global__ void generateVoxels_random_kernel(
  const float * points, std::size_t points_size, float min_x_range, float max_x_range,
  float min_y_range, float max_y_range, float min_z_range, float max_z_range, float pillar_x_size,
  float pillar_y_size, int grid_y_size, int grid_x_size, unsigned int * mask, float * voxels)
{
  int point_idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (point_idx >= points_size) return;

  const float x = points[point_idx * POINT_NUM_FEATURES];
  const float y = points[point_idx * POINT_NUM_FEATURES + 1];
  const float z = points[point_idx * POINT_NUM_FEATURES + 2];

  if (
    x < min_x_range || x >= max_x_range || y < min_y_range || y >= max_y_range || z < min_z_range ||
    z >= max_z_range)
    return;

  int voxel_idx = floorf((x - min_x_range) / pillar_x_size);
  int voxel_idy = floorf((y - min_y_range) / pillar_y_size);
  voxel_idx = voxel_idx < 0 ? 0 : voxel_idx >= grid_x_size ? grid_x_size - 1 : voxel_idx;
  voxel_idy = voxel_idy < 0 ? 0 : voxel_idy >= grid_y_size ? grid_y_size - 1 : voxel_idy;
  unsigned int voxel_index = (grid_x_size - 1 - voxel_idx) * grid_y_size + voxel_idy;

  unsigned int point_id = atomicAdd(&(mask[voxel_index]), 1);

  if (point_id >= MAX_POINT_IN_VOXEL_SIZE) return;

  float * address =
    voxels + (voxel_index * MAX_POINT_IN_VOXEL_SIZE + point_id) * POINT_NUM_FEATURES;
  atomicExch(address, x);
  atomicExch(address + 1, y);
  atomicExch(address + 2, z);
  if (POINT_NUM_FEATURES == POINT_DIM_XYZT) {
    const float t = points[point_idx * POINT_NUM_FEATURES + 3];
    atomicExch(address + 3, t);  // Time_lag
  } else if (POINT_NUM_FEATURES == POINT_DIM_XYZIT) {
    const float i = points[point_idx * POINT_NUM_FEATURES + 3];
    const float t = points[point_idx * POINT_NUM_FEATURES + 4];
    atomicExch(address + 3, i);  // Intensity
    atomicExch(address + 4, t);  // Time_lag
  }
}

template <std::size_t POINT_NUM_FEATURES>
__global__ void generateBaseFeatures_kernel(
  unsigned int * mask, float * voxels, int grid_y_size, int grid_x_size, int max_voxel_size,
  unsigned int * pillar_num, float * voxel_features, float * voxel_num, int * voxel_idxs)
{
  // exchange x and y to process in a row-major order
  // flip x axis direction to process front to back
  unsigned int voxel_idx_inverted = blockIdx.y * blockDim.y + threadIdx.y;
  unsigned int voxel_idy = blockIdx.x * blockDim.x + threadIdx.x;
  if (voxel_idx_inverted >= grid_x_size || voxel_idy >= grid_y_size) return;
  unsigned int voxel_idx = grid_x_size - 1 - voxel_idx_inverted;

  unsigned int voxel_index = voxel_idx_inverted * grid_y_size + voxel_idy;
  unsigned int count = mask[voxel_index];
  if (!(count > 0)) return;
  count = count < MAX_POINT_IN_VOXEL_SIZE ? count : MAX_POINT_IN_VOXEL_SIZE;

  unsigned int current_pillarId = 0;
  current_pillarId = atomicAdd(pillar_num, 1);
  if (current_pillarId > max_voxel_size - 1) return;

  voxel_num[current_pillarId] = count;

  uint3 idx = {0, voxel_idy, voxel_idx};
  ((uint3 *)voxel_idxs)[current_pillarId] = idx;

  if (POINT_NUM_FEATURES == POINT_DIM_XYZT) {
    for (int i = 0; i < count; i++) {
      int inIndex = voxel_index * MAX_POINT_IN_VOXEL_SIZE + i;
      int outIndex = current_pillarId * MAX_POINT_IN_VOXEL_SIZE + i;
      ((float4 *)voxel_features)[outIndex] = ((float4 *)voxels)[inIndex];
    }
  } else {
    for (int i = 0; i < count; i++) {
      int inIndex = voxel_index * MAX_POINT_IN_VOXEL_SIZE + i;
      int outIndex = current_pillarId * MAX_POINT_IN_VOXEL_SIZE + i;
      voxel_features[outIndex * 5] = voxels[inIndex * 5];
      voxel_features[outIndex * 5 + 1] = voxels[inIndex * 5 + 1];
      voxel_features[outIndex * 5 + 2] = voxels[inIndex * 5 + 2];
      voxel_features[outIndex * 5 + 3] = voxels[inIndex * 5 + 3];
      voxel_features[outIndex * 5 + 4] = voxels[inIndex * 5 + 4];
    }
  }

  // clear buffer for next infer
  atomicExch(mask + voxel_index, 0);
}

template <std::size_t ENCODER_IN_FEATURE_SIZE>
__global__ void generateFeatures_kernel(
  const float * voxel_features, const float * voxel_num_points, const int * coords,
  const unsigned int * num_voxels, const float voxel_x, const float voxel_y, const float voxel_z,
  const float range_min_x, const float range_min_y, const float range_min_z, float * features)
{
  // voxel_features (float): (max_voxel_size, max_point_in_voxel_size, point_feature_size)
  // voxel_num_points (int): (max_voxel_size)
  // coords (int): (max_voxel_size, point_dim_size)
  int pillar_idx = blockIdx.x * WARPS_PER_BLOCK + threadIdx.x / MAX_POINT_IN_VOXEL_SIZE;
  int point_idx = threadIdx.x % MAX_POINT_IN_VOXEL_SIZE;
  int pillar_idx_inBlock = threadIdx.x / MAX_POINT_IN_VOXEL_SIZE;  // max_point_in_voxel_size

  unsigned int num_pillars = num_voxels[0];
  if (pillar_idx >= num_pillars) return;

  // point dimemension is 5 if feature size in encoder is 11, otherwise 4
  constexpr int point_dim =
    (ENCODER_IN_FEATURE_SIZE >= ENCODER_NUM_FEATURES_11) ? POINT_DIM_XYZIT : POINT_DIM_XYZT;

  // load src
  __shared__ float pillarSM[WARPS_PER_BLOCK][MAX_POINT_IN_VOXEL_SIZE][point_dim];
  __shared__ float3 pillarSumSM[WARPS_PER_BLOCK];
  __shared__ int3 cordsSM[WARPS_PER_BLOCK];
  __shared__ int pointsNumSM[WARPS_PER_BLOCK];
  __shared__ float pillarOutSM[WARPS_PER_BLOCK][MAX_POINT_IN_VOXEL_SIZE][ENCODER_IN_FEATURE_SIZE];

  if (threadIdx.x < WARPS_PER_BLOCK) {
    pointsNumSM[threadIdx.x] = voxel_num_points[blockIdx.x * WARPS_PER_BLOCK + threadIdx.x];
    cordsSM[threadIdx.x] = ((int3 *)coords)[blockIdx.x * WARPS_PER_BLOCK + threadIdx.x];
    pillarSumSM[threadIdx.x] = {0, 0, 0};
  }

#pragma unroll
  for (int i = 0; i < point_dim; i++) {
    int pillarSMId = pillar_idx_inBlock * MAX_POINT_IN_VOXEL_SIZE * point_dim +
                     i * MAX_POINT_IN_VOXEL_SIZE + point_idx;
    int voxel_feature_id =
      pillar_idx * MAX_POINT_IN_VOXEL_SIZE * point_dim + i * MAX_POINT_IN_VOXEL_SIZE + point_idx;
    ((float *)pillarSM)[pillarSMId] = ((float *)voxel_features)[voxel_feature_id];
  }
  __syncthreads();

  // calculate sm in a pillar
  if (point_idx < pointsNumSM[pillar_idx_inBlock]) {
    atomicAdd(&(pillarSumSM[pillar_idx_inBlock].x), pillarSM[pillar_idx_inBlock][point_idx][0]);
    atomicAdd(&(pillarSumSM[pillar_idx_inBlock].y), pillarSM[pillar_idx_inBlock][point_idx][1]);
    atomicAdd(&(pillarSumSM[pillar_idx_inBlock].z), pillarSM[pillar_idx_inBlock][point_idx][2]);
  }
  __syncthreads();

  // feature-mean
  float3 mean;
  float validPoints = pointsNumSM[pillar_idx_inBlock];
  mean.x = pillarSumSM[pillar_idx_inBlock].x / validPoints;
  mean.y = pillarSumSM[pillar_idx_inBlock].y / validPoints;
  mean.z = pillarSumSM[pillar_idx_inBlock].z / validPoints;

  mean.x = pillarSM[pillar_idx_inBlock][point_idx][0] - mean.x;
  mean.y = pillarSM[pillar_idx_inBlock][point_idx][1] - mean.y;
  mean.z = pillarSM[pillar_idx_inBlock][point_idx][2] - mean.z;

  // calculate offset
  float x_offset = voxel_x / 2 + cordsSM[pillar_idx_inBlock].z * voxel_x + range_min_x;
  float y_offset = voxel_y / 2 + cordsSM[pillar_idx_inBlock].y * voxel_y + range_min_y;
  float z_offset = voxel_z / 2 + cordsSM[pillar_idx_inBlock].x * voxel_z + range_min_z;

  // feature-offset
  float3 center;
  center.x = pillarSM[pillar_idx_inBlock][point_idx][0] - x_offset;
  center.y = pillarSM[pillar_idx_inBlock][point_idx][1] - y_offset;
  center.z = pillarSM[pillar_idx_inBlock][point_idx][2] - z_offset;

  // store output
  if (point_idx < pointsNumSM[pillar_idx_inBlock]) {
    pillarOutSM[pillar_idx_inBlock][point_idx][0] = pillarSM[pillar_idx_inBlock][point_idx][0];
    pillarOutSM[pillar_idx_inBlock][point_idx][1] = pillarSM[pillar_idx_inBlock][point_idx][1];
    pillarOutSM[pillar_idx_inBlock][point_idx][2] = pillarSM[pillar_idx_inBlock][point_idx][2];
    pillarOutSM[pillar_idx_inBlock][point_idx][3] = pillarSM[pillar_idx_inBlock][point_idx][3];

    if (ENCODER_IN_FEATURE_SIZE == ENCODER_NUM_FEATURES_11) {
      pillarOutSM[pillar_idx_inBlock][point_idx][4] = pillarSM[pillar_idx_inBlock][point_idx][4];
      pillarOutSM[pillar_idx_inBlock][point_idx][5] = mean.x;
      pillarOutSM[pillar_idx_inBlock][point_idx][6] = mean.y;
      pillarOutSM[pillar_idx_inBlock][point_idx][7] = mean.z;

      pillarOutSM[pillar_idx_inBlock][point_idx][8] = center.x;
      pillarOutSM[pillar_idx_inBlock][point_idx][9] = center.y;
      pillarOutSM[pillar_idx_inBlock][point_idx][10] = center.z;
    } else {
      pillarOutSM[pillar_idx_inBlock][point_idx][4] = mean.x;
      pillarOutSM[pillar_idx_inBlock][point_idx][5] = mean.y;
      pillarOutSM[pillar_idx_inBlock][point_idx][6] = mean.z;

      pillarOutSM[pillar_idx_inBlock][point_idx][7] = center.x;
      pillarOutSM[pillar_idx_inBlock][point_idx][8] = center.y;

      if (ENCODER_IN_FEATURE_SIZE == ENCODER_NUM_FEATURES_10) {
        pillarOutSM[pillar_idx_inBlock][point_idx][9] = center.z;
      }
    }

  } else {
    pillarOutSM[pillar_idx_inBlock][point_idx][0] = 0;
    pillarOutSM[pillar_idx_inBlock][point_idx][1] = 0;
    pillarOutSM[pillar_idx_inBlock][point_idx][2] = 0;
    pillarOutSM[pillar_idx_inBlock][point_idx][3] = 0;

    pillarOutSM[pillar_idx_inBlock][point_idx][4] = 0;
    pillarOutSM[pillar_idx_inBlock][point_idx][5] = 0;
    pillarOutSM[pillar_idx_inBlock][point_idx][6] = 0;

    pillarOutSM[pillar_idx_inBlock][point_idx][7] = 0;
    pillarOutSM[pillar_idx_inBlock][point_idx][8] = 0;

    if (ENCODER_IN_FEATURE_SIZE >= ENCODER_NUM_FEATURES_10) {
      pillarOutSM[pillar_idx_inBlock][point_idx][9] = 0;
    }
    if (ENCODER_IN_FEATURE_SIZE >= ENCODER_NUM_FEATURES_11) {
      pillarOutSM[pillar_idx_inBlock][point_idx][10] = 0;
    }
  }

  __syncthreads();

  for (int i = 0; i < ENCODER_IN_FEATURE_SIZE; i++) {
    int outputSMId = pillar_idx_inBlock * MAX_POINT_IN_VOXEL_SIZE * ENCODER_IN_FEATURE_SIZE +
                     i * MAX_POINT_IN_VOXEL_SIZE + point_idx;
    int outputId = pillar_idx * MAX_POINT_IN_VOXEL_SIZE * ENCODER_IN_FEATURE_SIZE +
                   i * MAX_POINT_IN_VOXEL_SIZE + point_idx;
    features[outputId] = ((float *)pillarOutSM)[outputSMId];
  }
}

cudaError_t PreprocessCuda::generateSweepPoints_launch(
  const InputPointType * input_points, std::size_t points_size, float time_lag,
  const float * transform_array, float * output_points)
{
  dim3 blocks((points_size + 256 - 1) / 256);
  dim3 threads(256);

  if (config_.point_feature_size_ == POINT_DIM_XYZT) {
    generateSweepPoints_kernel<POINT_DIM_XYZT><<<blocks, threads, 0, stream_>>>(
      input_points, points_size, time_lag, transform_array, output_points);
  } else if (config_.point_feature_size_ == POINT_DIM_XYZIT) {
    generateSweepPoints_kernel<POINT_DIM_XYZIT><<<blocks, threads, 0, stream_>>>(
      input_points, points_size, time_lag, transform_array, output_points);
  } else {
    throw std::runtime_error("Value of point_features_size is not supported!");
  }

  cudaError_t err = cudaGetLastError();
  return err;
}

cudaError_t PreprocessCuda::shufflePoints_launch(
  const float * points, const unsigned int * indices, float * shuffled_points,
  const std::size_t points_size, const std::size_t max_size, const std::size_t offset)
{
  dim3 blocks((max_size + 256 - 1) / 256);
  dim3 threads(256);

  if (blocks.x == 0) {
    return cudaGetLastError();
  }

  if (config_.point_feature_size_ == POINT_DIM_XYZT) {
    shufflePoints_kernel<POINT_DIM_XYZT><<<blocks, threads, 0, stream_>>>(
      points, indices, shuffled_points, points_size, max_size, offset);
  } else if (config_.point_feature_size_ == POINT_DIM_XYZIT) {
    shufflePoints_kernel<POINT_DIM_XYZIT><<<blocks, threads, 0, stream_>>>(
      points, indices, shuffled_points, points_size, max_size, offset);
  } else {
    throw std::runtime_error("Value of point_features_size is not supported!");
  }
  cudaError_t err = cudaGetLastError();
  return err;
}

cudaError_t PreprocessCuda::generateVoxels_random_launch(
  const float * points, std::size_t points_size, unsigned int * mask, float * voxels)
{
  dim3 blocks((points_size + 256 - 1) / 256);
  dim3 threads(256);

  if (blocks.x == 0) {
    return cudaGetLastError();
  }

  if (config_.point_feature_size_ == POINT_DIM_XYZT) {
    generateVoxels_random_kernel<POINT_DIM_XYZT><<<blocks, threads, 0, stream_>>>(
      points, points_size, config_.range_min_x_, config_.range_max_x_, config_.range_min_y_,
      config_.range_max_y_, config_.range_min_z_, config_.range_max_z_, config_.voxel_size_x_,
      config_.voxel_size_y_, config_.grid_size_y_, config_.grid_size_x_, mask, voxels);
  } else if (config_.point_feature_size_ == POINT_DIM_XYZIT) {
    generateVoxels_random_kernel<POINT_DIM_XYZIT><<<blocks, threads, 0, stream_>>>(
      points, points_size, config_.range_min_x_, config_.range_max_x_, config_.range_min_y_,
      config_.range_max_y_, config_.range_min_z_, config_.range_max_z_, config_.voxel_size_x_,
      config_.voxel_size_y_, config_.grid_size_y_, config_.grid_size_x_, mask, voxels);
  } else {
    throw std::runtime_error("Value of point_features_size is not supported!");
  }

  cudaError_t err = cudaGetLastError();
  return err;
}

// create 4 channels
cudaError_t PreprocessCuda::generateBaseFeatures_launch(
  unsigned int * mask, float * voxels, unsigned int * pillar_num, float * voxel_features,
  float * voxel_num, int * voxel_idxs)
{
  // exchange x and y to process in a row-major order
  dim3 threads = {32, 32};
  dim3 blocks = {
    (static_cast<unsigned int>(config_.grid_size_y_) + threads.x - 1) / threads.x,
    (static_cast<unsigned int>(config_.grid_size_x_) + threads.y - 1) / threads.y};

  if (config_.point_feature_size_ == POINT_DIM_XYZT) {
    generateBaseFeatures_kernel<POINT_DIM_XYZT><<<blocks, threads, 0, stream_>>>(
      mask, voxels, config_.grid_size_y_, config_.grid_size_x_, config_.max_voxel_size_, pillar_num,
      voxel_features, voxel_num, voxel_idxs);
  } else if (config_.point_feature_size_ == POINT_DIM_XYZIT) {
    generateBaseFeatures_kernel<POINT_DIM_XYZIT><<<blocks, threads, 0, stream_>>>(
      mask, voxels, config_.grid_size_y_, config_.grid_size_x_, config_.max_voxel_size_, pillar_num,
      voxel_features, voxel_num, voxel_idxs);
  } else {
    throw std::runtime_error("Value of point_features_size is not supported!");
  }
  cudaError_t err = cudaGetLastError();
  return err;
}

// cspell: ignore divup
cudaError_t PreprocessCuda::generateFeatures_launch(
  const float * voxel_features, const float * voxel_num_points, const int * coords,
  const unsigned int * num_voxels, float * features)
{
  dim3 blocks(divup(config_.max_voxel_size_, WARPS_PER_BLOCK));
  dim3 threads(WARPS_PER_BLOCK * MAX_POINT_IN_VOXEL_SIZE);
  if (config_.encoder_in_feature_size_ == ENCODER_NUM_FEATURES_9) {
    generateFeatures_kernel<ENCODER_NUM_FEATURES_9><<<blocks, threads, 0, stream_>>>(
      voxel_features, voxel_num_points, coords, num_voxels, config_.voxel_size_x_,
      config_.voxel_size_y_, config_.voxel_size_z_, config_.range_min_x_, config_.range_min_y_,
      config_.range_min_z_, features);
  } else if (config_.encoder_in_feature_size_ == ENCODER_NUM_FEATURES_10) {
    generateFeatures_kernel<ENCODER_NUM_FEATURES_10><<<blocks, threads, 0, stream_>>>(
      voxel_features, voxel_num_points, coords, num_voxels, config_.voxel_size_x_,
      config_.voxel_size_y_, config_.voxel_size_z_, config_.range_min_x_, config_.range_min_y_,
      config_.range_min_z_, features);
  } else if (config_.encoder_in_feature_size_ == ENCODER_NUM_FEATURES_11) {
    generateFeatures_kernel<ENCODER_NUM_FEATURES_11><<<blocks, threads, 0, stream_>>>(
      voxel_features, voxel_num_points, coords, num_voxels, config_.voxel_size_x_,
      config_.voxel_size_y_, config_.voxel_size_z_, config_.range_min_x_, config_.range_min_y_,
      config_.range_min_z_, features);
  } else {
    throw std::runtime_error("Value of encoder_in_feature_size is not supported!");
  }

  return cudaGetLastError();
}

}  // namespace autoware::lidar_centerpoint
