// Copyright 2024 TIER IV, Inc.
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

#include <autoware/behavior_velocity_planner/test_utils.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner
{
TEST(PlanningModuleInterfaceTest, NodeTestWithExceptionPathWithLaneID)
{
  rclcpp::init(0, nullptr);
  auto test_manager = autoware::behavior_velocity_planner::generateTestManager();
  auto test_target_node = autoware::behavior_velocity_planner::generateNode({});

  autoware::behavior_velocity_planner::publishMandatoryTopics(test_manager, test_target_node);

  const std::string input_path_with_lane_id_topic =
    "behavior_velocity_planner_node/input/path_with_lane_id";

  // test with nominal path_with_lane_id
  ASSERT_NO_THROW_WITH_ERROR_MSG(
    test_manager->testWithNormalPathWithLaneId(test_target_node, input_path_with_lane_id_topic));
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  // test with empty path_with_lane_id
  ASSERT_NO_THROW_WITH_ERROR_MSG(
    test_manager->testWithAbnormalPathWithLaneId(test_target_node, input_path_with_lane_id_topic));
  rclcpp::shutdown();
}

TEST(PlanningModuleInterfaceTest, NodeTestWithOffTrackEgoPose)
{
  rclcpp::init(0, nullptr);

  const auto plugin_info_vec = {autoware::behavior_velocity_planner::PluginInfo{
    "blind_spot", "autoware::behavior_velocity_planner::BlindSpotModulePlugin"}};

  auto test_manager = autoware::behavior_velocity_planner::generateTestManager();
  auto test_target_node = autoware::behavior_velocity_planner::generateNode(plugin_info_vec);
  autoware::behavior_velocity_planner::publishMandatoryTopics(test_manager, test_target_node);

  const std::string input_path_with_lane_id_topic =
    "behavior_velocity_planner_node/input/path_with_lane_id";
  const std::string input_odometry_topic = "behavior_velocity_planner_node/input/vehicle_odometry";

  // test for normal trajectory
  ASSERT_NO_THROW_WITH_ERROR_MSG(
    test_manager->testWithNormalPathWithLaneId(test_target_node, input_path_with_lane_id_topic));

  // make sure behavior_path_planner is running
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  ASSERT_NO_THROW_WITH_ERROR_MSG(
    test_manager->testWithOffTrackOdometry(test_target_node, input_odometry_topic));

  rclcpp::shutdown();
}
}  // namespace autoware::behavior_velocity_planner
