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
  // Constructor
  PointCloudSwitcher::PointCloudSwitcher(const rclcpp::NodeOptions & options)
  : rclcpp::Node("pointcloud_switcher", options)
  {
    // Get paramters from yaml file
    this->declare_parameter<vector<string>>("pointcloud_topic_names", vector<string>());
    this->declare_parameter<double>("subscription_period_confirmation_time_span", 1.0);
    this->declare_parameter<int>("steps_for_moving_average", 5);
    this->declare_parameter<double>("average_subscription_period_threshold", 1.0);

    this->get_parameter("pointcloud_topic_names", pointcloud_candidates_);
    this->get_parameter("subscription_period_confirmation_time_span", subscription_period_confimation_time_span);
    this->get_parameter("steps_for_moving_average", steps_for_moving_average);
    this->get_parameter("average_subscription_period_threshold", delta_time_average_threshold_);

    // Check pointcloud_topic_names and create subscribers for each topic
    int i = 0;
    RCLCPP_INFO(this->get_logger(), "pointcloud_topic_name size: %ld", pointcloud_candidates_.size());
    rclcpp::QoS qos_pointcloud(5);
    qos_pointcloud.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    for(const auto &topic_name : pointcloud_candidates_)
    {
      // Bind all pointcloud topics to pointcloud_callback(). pointcloud_callback() can distinguish which topic is called by using topic_name
      RCLCPP_INFO(this->get_logger(), "pointcloud_topic_name: %s", topic_name.c_str());
      auto callback = [this, topic_name](const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg) {
        this->pointcloud_callback(pointcloud_msg, topic_name);
      };
      pointcloud_subscribers_.push_back(this->create_subscription<sensor_msgs::msg::PointCloud2>(topic_name, qos_pointcloud, callback));

      // Set last_received_time_
      last_received_time_[topic_name] = this->now();

      // Set priority (The earliest element has the highest priority)
      pointcloud_priority_[topic_name] = i++;

      // Set pending_delta_flag_
      pending_delta_flag_[topic_name] = false;
    }

    // Create a timer to check heartbeat
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(subscription_period_confimation_time_span),
      std::bind(&PointCloudSwitcher::check_heartbeat, this)
    );

    // Set selected_pointcloud_topic_name_ to the first element of pointcloud_candidates_
    selected_pointcloud_topic_name_ = pointcloud_candidates_[0];

    // Create publisher of /selected/pointcloud
    selected_pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output/pointcloud", 10);

    // Create subscriber of /api/localization/initialization_state
    rclcpp::QoS qos_initialization_state(1);
    qos_initialization_state.reliability(rclcpp::ReliabilityPolicy::Reliable);
    qos_initialization_state.durability(rclcpp::DurabilityPolicy::Volatile);
    initialization_state_subscriber_ = this->create_subscription<autoware_adapi_v1_msgs::msg::LocalizationInitializationState>(
      "/api/localization/initialization_state", qos_initialization_state, std::bind(&PointCloudSwitcher::initialization_state_callback, this, std::placeholders::_1));

    // Initialize last_initialization_state_
    last_initialization_state_.stamp = this->now();
    last_initialization_state_.state = autoware_adapi_v1_msgs::msg::LocalizationInitializationState::UNKNOWN;

    // Estimate the time how long will it take to switch to the next pointcloud when the selected pointcloud is down
    RCLCPP_DEBUG(this->get_logger(), "It might take %lf seconds to judge whether the selected pointcloud is down.", ceil(steps_for_moving_average * delta_time_average_threshold_ / subscription_period_confimation_time_span) * subscription_period_confimation_time_span);
  }

  void PointCloudSwitcher::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg, const string topic_name)
  {
    // Calculate delta time and update last_received_time_
    rclcpp::Time current_time = this->now();
    double delta_time = (current_time - last_received_time_[topic_name]).seconds();

    // Update delta_times_ and last_received_time_
    if (pending_delta_flag_[topic_name] == true) {
      delta_times_[topic_name].back() = delta_time;
      pending_delta_flag_[topic_name] = false;
    } else {
      if (delta_times_[topic_name].size() >= steps_for_moving_average) {
        delta_times_[topic_name].erase(delta_times_[topic_name].begin());
      }
      delta_times_[topic_name].push_back(delta_time);
    }

    last_received_time_[topic_name] = current_time;

    // Check if topic_name is equal to selected_pointcloud_topic_name_, if so, publish pointcloud_msg to /selected/pointcloud
    if(topic_name == selected_pointcloud_topic_name_)
    {
      selected_pointcloud_publisher_->publish(*pointcloud_msg);
      return;
    }
  }

  void PointCloudSwitcher::initialization_state_callback(const autoware_adapi_v1_msgs::msg::LocalizationInitializationState::SharedPtr msg)
  {
    // If the state is changed from INITIALIZING to UNINITIALIZED, which means initialization failure, switch to the available pointcloud.
    if(last_initialization_state_.state == autoware_adapi_v1_msgs::msg::LocalizationInitializationState::INITIALIZING && msg->state == autoware_adapi_v1_msgs::msg::LocalizationInitializationState::UNINITIALIZED)
    {
      RCLCPP_INFO(this->get_logger(), "Found initilization failure.");
      string next_pointcloud_topic_name_ = next_pointcloud_topic();//(selected_pointcloud_topic_name_);
      if (next_pointcloud_topic_name_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "No available pointcloud topic!!");
      } else {
        if(next_pointcloud_topic_name_ != selected_pointcloud_topic_name_) RCLCPP_WARN(this->get_logger(), "Switching pointcloud from %s to %s.", selected_pointcloud_topic_name_.c_str(), next_pointcloud_topic_name_.c_str());
        selected_pointcloud_topic_name_ = next_pointcloud_topic_name_;
      }
    }
    last_initialization_state_ = *msg;
  }

  void PointCloudSwitcher::check_heartbeat() // Timer callback
  {
    rclcpp::Time current_time = this->now();

    // Watch delta time average
    for(const auto &topic_name : pointcloud_candidates_)
    {
      // Check if delta_times_ is empty, if so, print warning and continue
      if (delta_times_[topic_name].size() == 0) {
        RCLCPP_WARN(this->get_logger(), "Haven't received pointcloud from %s yet", topic_name.c_str());
        continue;
      } else {
        // If delta time is greater than heartbeat_confirmation_time_span, print warning and update delta_times_ by adding it to the last element
        if ((current_time - last_received_time_[topic_name]).seconds() > subscription_period_confimation_time_span) {
          // If pending_delta_flag_ is false, push_back a new delta time, if not, edit the last element
          // pending_delta_flag_ is used to avoid pushing back a new delta time when "the last element is edited" (= "a new pointcloud topic is still not subscribed")
          if (pending_delta_flag_[topic_name] == false) {
            // Keep the size of delta_times_ to steps_for_moving_average
            if (delta_times_[topic_name].size() >= steps_for_moving_average) {
              delta_times_[topic_name].erase(delta_times_[topic_name].begin());
            }
            delta_times_[topic_name].push_back(subscription_period_confimation_time_span);
            pending_delta_flag_[topic_name] = true;
          } else {
            delta_times_[topic_name].back() += subscription_period_confimation_time_span;
          }

          if ((current_time - last_received_time_[topic_name]).seconds() > delta_time_average_threshold_) {
            RCLCPP_WARN(this->get_logger(), "No pointcloud received from %s in %lf seconds", topic_name.c_str(), delta_times_[topic_name].back());
          }
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

    // Get the next pointcloud topic name
    string next_pointcloud_topic_name_ = next_pointcloud_topic();

    // If next_pointcloud_topic_name_ is empty, print error, if not, switch or keep the pointcloud topic
    if (next_pointcloud_topic_name_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No available pointcloud topic!!");
    } else {
      if(next_pointcloud_topic_name_ != selected_pointcloud_topic_name_) RCLCPP_WARN(this->get_logger(), "Switching pointcloud from %s to %s.", selected_pointcloud_topic_name_.c_str(), next_pointcloud_topic_name_.c_str());
      selected_pointcloud_topic_name_ = next_pointcloud_topic_name_;
    }
  }

  // Returns the most suitable pointcloud topic name. It may switch to another one, or keep the current topic.
  string PointCloudSwitcher::next_pointcloud_topic() 
  {
    string chosen_pointcloud_topic_name_;
    int max_priority = 65535;

    // Find pointcloud topic with higher priority as possible
    for(const auto &topic_name : pointcloud_candidates_)
    {
      if (delta_time_average_.find(topic_name) != delta_time_average_.end()) { // Check if delta_time_average_ has topic_name as a key
        if(delta_time_average_[topic_name] <= delta_time_average_threshold_ && pointcloud_priority_[topic_name] < max_priority) // Check if delta time average is lower than the threshold and priority is higher than the current pointcloud
        {
          chosen_pointcloud_topic_name_ = topic_name;
          max_priority = pointcloud_priority_[chosen_pointcloud_topic_name_];
        }
      }
    }

    return chosen_pointcloud_topic_name_;
  }  
}  // namespace pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_preprocessor::PointCloudSwitcher)