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

#ifndef POINTCLOUD_SWITCHER__POINTCLOUD_SWITCHER_HPP_
#define POINTCLOUD_SWITCHER__POINTCLOUD_SWITCHER_HPP_ 

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <string>
#include <vector>

using namespace std;

class PointCloudSwitcher : public rclcpp::Node
{
  public:
    PointCloudSwitcher();

  private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg, const string topic_name);
    vector<string> pointcloud_candidates_;
    string selected_pointcloud_topic_name_;
    vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subscribers_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr selected_pointcloud_publisher_;
};

#endif  // POINTCLOUD_SWITCHER__POINTCLOUD_SWITCHER_HPP_