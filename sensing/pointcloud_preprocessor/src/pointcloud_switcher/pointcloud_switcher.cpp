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

#include "pointcloud_preprocessor/pointcloud_switcher/pointcloud_switcher.hpp"

namespace pointcloud_preprocessor
{
PointCloudSwitcher::PointCloudSwitcher(const rclcpp::NodeOptions & options)
: rclcpp::Node("pointcloud_switcher", options)
{
  // Set logger level to DEBUG
  auto logger = this->get_logger();
  rcutils_ret_t ret = rcutils_logging_set_logger_level(logger.get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
  if (ret != RCUTILS_RET_OK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set logger level");
  }

  // Get paramters from yaml file
  this->declare_parameter<vector<string>>("pointcloud_topic_name", vector<string>());
  this->declare_parameter<int>("heartbeat_confirmation_time_span", 1000);
  this->declare_parameter<int>("steps_for_moving_average", 5);

  this->get_parameter("pointcloud_topic_name", pointcloud_candidates_);
  this->get_parameter("heartbeat_confirmation_time_span", heartbeat_confimation_time_span);
  this->get_parameter("steps_for_moving_average", steps_for_moving_average);

  // Check pointcloud_topic_names and create subscribers for each topic
  int i = 0;
  RCLCPP_INFO(this->get_logger(), "pointcloud_topic_name size: %ld", pointcloud_candidates_.size());
  rclcpp::QoS qos(5);
  qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  for(const auto &topic_name : pointcloud_candidates_)
  {
    RCLCPP_INFO(this->get_logger(), "pointcloud_topic_name: %s", topic_name.c_str());
    auto callback = [this, topic_name](const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg) {
      this->pointcloud_callback(pointcloud_msg, topic_name);
    };
    pointcloud_subscribers_.push_back(this->create_subscription<sensor_msgs::msg::PointCloud2>(topic_name, qos, callback));

    // Set last_received_time_
    last_received_time_[topic_name] = this->now();

    // Set priority (The earliest element has the highest priority)
    pointcloud_priority_[topic_name] = i++;
  }

  // Create a timer to check heartbeat
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(heartbeat_confimation_time_span),
    std::bind(&PointCloudSwitcher::check_heartbeat, this)
  );

  // Set selected_pointcloud_topic_name_ to the first element of pointcloud_candidates_
  selected_pointcloud_topic_name_ = pointcloud_candidates_[0];

  // Create publisher of /selected/pointcloud
  selected_pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output/pointcloud", 10);
}

void PointCloudSwitcher::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg, const string topic_name)
{
  // Calculate delta time and update last_received_time_
  rclcpp::Time current_time = this->now();
  double delta_time = (current_time - last_received_time_[topic_name]).seconds();
  
  if (delta_times_[topic_name].size() >= steps_for_moving_average) {
    delta_times_[topic_name].erase(delta_times_[topic_name].begin());
  }
  delta_times_[topic_name].push_back(delta_time);

  last_received_time_[topic_name] = current_time;

  // Check if topic_name is equal to selected_pointcloud_topic_name_, if so, publish pointcloud_msg to /selected/pointcloud
  if(topic_name == selected_pointcloud_topic_name_)
  {
    selected_pointcloud_publisher_->publish(*pointcloud_msg);
    return;
  }
}

void PointCloudSwitcher::check_heartbeat()
{
  rclcpp::Time current_time = this->now();
  double timer_span = static_cast<double>(heartbeat_confimation_time_span)/1000.0; // in [s]

  // Watch delta time average
  for(const auto &topic_name : pointcloud_candidates_)
  {
    // Check if delta_times_ is empty, if so, print warning and continue
    if (delta_times_[topic_name].size() == 0) {
      RCLCPP_WARN(this->get_logger(), "Haven't received pointcloud from %s yet", topic_name.c_str());
      continue;
    } else {
      // If delta time is greater than 1.0, print warning and update delta_times_ by adding 1.0 to the last element
      if ((current_time - last_received_time_[topic_name]).seconds() > timer_span) {
        // Add timer span to the last element of delta_times_ 
        delta_times_[topic_name].back() += timer_span;
        RCLCPP_WARN(this->get_logger(), "No pointcloud received from %s in %lf seconds", topic_name.c_str(), delta_times_[topic_name].back());
      }
      // Calculate delta time average
      double delta_time_sum = 0;
      for (const auto &delta_time : delta_times_[topic_name]) {
        delta_time_sum += delta_time;
      }
      delta_time_average_[topic_name] = delta_time_sum / delta_times_[topic_name].size();

      RCLCPP_DEBUG(this->get_logger(), "delta_time_average of %s: %f", topic_name.c_str(), delta_time_average_[topic_name]);
    }
  }

  // If the selected_pointcloud has not received pointcloud yet, skip the following process
  if (delta_times_[selected_pointcloud_topic_name_].size() == 0) {
    // TODO(Taiki Yamada): Check the time difference between current time and last_received_time_[selected_pointcloud_topic_name_]
    return;
  }

  // If the selected_pointcloud has larger delta time than the threshold, switch to the available pointcloud with the highest priority
  if(delta_time_average_[selected_pointcloud_topic_name_] > 1.0){
    int max_priority = 65535;
    string next_pointcloud_topic_name_;

    for(const auto &topic_name : pointcloud_candidates_)
    {
      if(delta_time_average_[topic_name] < 1.0 && pointcloud_priority_[topic_name] < max_priority)
      {
        max_priority = pointcloud_priority_[topic_name];
        next_pointcloud_topic_name_ = topic_name;
      }
    }

    if (next_pointcloud_topic_name_.empty()) {
      RCLCPP_WARN(this->get_logger(), "No available pointcloud topic!!");
    } else {
      RCLCPP_WARN(this->get_logger(), "Switching pointcloud from %s to %s", selected_pointcloud_topic_name_.c_str(), next_pointcloud_topic_name_.c_str());
      selected_pointcloud_topic_name_ = next_pointcloud_topic_name_;
    }
  }
}
}  // namespace pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_preprocessor::PointCloudSwitcher)