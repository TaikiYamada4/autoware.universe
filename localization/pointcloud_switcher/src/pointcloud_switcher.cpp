// Copyright 2018-2019 Autoware Foundation
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

#include <pointcloud_switcher/pointcloud_switcher.hpp>

PointCloudSwitcher::PointCloudSwitcher() : rclcpp::Node("pointcloud_switcher")
{
  this->declare_parameter<vector<string>>("pointcloud_topic_name", vector<string>());
  this->get_parameter("pointcloud_topic_name", pointcloud_candidates_);

  RCLCPP_INFO(this->get_logger(), "pointcloud_topic_name size: %d", pointcloud_candidates_.size());

  for(auto topic_name : pointcloud_candidates_)
  {
    RCLCPP_INFO(this->get_logger(), "pointcloud_topic_name: %s", topic_name.c_str());
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudSwitcher>();
  rclcpp::spin(node);
  return 0;
}