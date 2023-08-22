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

#ifndef POINTCLOUD_PREPROCESSOR__POINTCLOUD_SWITCHER__POINTCLOUD_SWITCHER_HPP_
#define POINTCLOUD_PREPROCESSOR__POINTCLOUD_SWITCHER__POINTCLOUD_SWITCHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>

#include <string>
#include <vector>

using namespace std;

namespace pointcloud_preprocessor
{
  class PointCloudSwitcher : public rclcpp::Node
  {
    public:
      PointCloudSwitcher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    private:
      void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg, const string topic_name);
      void initialization_state_callback(const autoware_adapi_v1_msgs::msg::LocalizationInitializationState::SharedPtr msg);
      void check_heartbeat();
      string next_pointcloud_topic(); //(const string current_pointcloud_topic_name);
      
      vector<string> pointcloud_candidates_;
      map<string, int> pointcloud_priority_;
      string selected_pointcloud_topic_name_;
      vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> pointcloud_subscribers_;
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr selected_pointcloud_publisher_;
      rclcpp::Subscription<autoware_adapi_v1_msgs::msg::LocalizationInitializationState>::SharedPtr initialization_state_subscriber_;
      autoware_adapi_v1_msgs::msg::LocalizationInitializationState last_initialization_state_;

      map<string, rclcpp::Time> last_received_time_; // map of topic_name and last received time
      map<string, vector<double>> delta_times_; // map of topic_name and a vector of delta times
      map<string, double> delta_time_average_; // map of topic_name and delta time average
      map<string, bool> pending_delta_flag_; // flag for pending delta time
      size_t steps_for_moving_average; // Number of delta time to be stored
      double heartbeat_confimation_time_span; // in [s]
      double delta_time_average_threshold_; // in [s]
      
      rclcpp::TimerBase::SharedPtr timer_;
  };
}  // namespace pointcloud_preprocessor

#endif  // POINTCLOUD_SWITCHER__POINTCLOUD_SWITCHER_HPP_