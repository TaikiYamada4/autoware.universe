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
  // Get paramters from yaml file
  this->declare_parameter<vector<string>>("pointcloud_topic_name", vector<string>());
  this->get_parameter("pointcloud_topic_name", pointcloud_candidates_);

  // Check pointcloud_topic_names and create subscribers for each topic
  RCLCPP_INFO(this->get_logger(), "pointcloud_topic_name size: %ld", pointcloud_candidates_.size());
  rclcpp::QoS qos(5);
  qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  for(const auto &topic_name : pointcloud_candidates_)
  {
    RCLCPP_INFO(this->get_logger(), "pointcloud_topic_name: %s", topic_name.c_str());
    auto callback = [this, topic_name](const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg) {
      this->pointcloud_callback(pointcloud_msg, topic_name);
    };
    subscribers_.push_back(this->create_subscription<sensor_msgs::msg::PointCloud2>(topic_name, qos, callback));
  }

  // Set selected_pointcloud_topic_name_ to the first element of pointcloud_candidates_
  selected_pointcloud_topic_name_ = pointcloud_candidates_[0];

  // Create publisher of /selected/pointcloud
  selected_pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/selected/pointcloud", 10);
}

void PointCloudSwitcher::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg, const string topic_name)
{
  RCLCPP_DEBUG(this->get_logger(), "pointcloud_callback: %s", topic_name.c_str());
  if(topic_name == selected_pointcloud_topic_name_)
  {
    selected_pointcloud_publisher_->publish(*pointcloud_msg);
    return;
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudSwitcher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}