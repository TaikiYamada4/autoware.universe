// Copyright 2023 TIER IV, Inc.
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

#include "autoware/behavior_path_goal_planner_module/manager.hpp"

#include "autoware/behavior_path_goal_planner_module/goal_planner_module.hpp"
#include "autoware/behavior_path_goal_planner_module/goal_planner_parameters.hpp"
#include "autoware_utils/ros/update_param.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{

std::unique_ptr<SceneModuleInterface> GoalPlannerModuleManager::createNewSceneModuleInstance()
{
  return std::make_unique<GoalPlannerModule>(
    name_, *node_, parameters_, rtc_interface_ptr_map_,
    objects_of_interest_marker_interface_ptr_map_, planning_factor_interface_);
}

GoalPlannerParameters GoalPlannerModuleManager::initGoalPlannerParameters(
  rclcpp::Node * node, const std::string & base_ns)
{
  GoalPlannerParameters p;
  // general params
  {
    p.th_stopped_velocity = node->declare_parameter<double>(base_ns + "th_stopped_velocity");
    p.th_arrived_distance = node->declare_parameter<double>(base_ns + "th_arrived_distance");
    p.th_stopped_time = node->declare_parameter<double>(base_ns + "th_stopped_time");
    p.center_line_path_interval =
      node->declare_parameter<double>(base_ns + "center_line_path_interval");
  }

  // goal search
  {
    const std::string ns = base_ns + "goal_search.";
    p.goal_priority = node->declare_parameter<std::string>(ns + "goal_priority");
    p.minimum_weighted_distance_lateral_weight =
      node->declare_parameter<double>(ns + "minimum_weighted_distance.lateral_weight");
    p.prioritize_goals_before_objects =
      node->declare_parameter<bool>(ns + "prioritize_goals_before_objects");
    p.forward_goal_search_length =
      node->declare_parameter<double>(ns + "forward_goal_search_length");
    p.backward_goal_search_length =
      node->declare_parameter<double>(ns + "backward_goal_search_length");
    p.goal_search_interval = node->declare_parameter<double>(ns + "goal_search_interval");
    p.longitudinal_margin = node->declare_parameter<double>(ns + "longitudinal_margin");
    p.max_lateral_offset = node->declare_parameter<double>(ns + "max_lateral_offset");
    p.lateral_offset_interval = node->declare_parameter<double>(ns + "lateral_offset_interval");
    p.ignore_distance_from_lane_start =
      node->declare_parameter<double>(ns + "ignore_distance_from_lane_start");
    p.margin_from_boundary = node->declare_parameter<double>(ns + "margin_from_boundary");
    p.high_curvature_threshold = node->declare_parameter<double>(ns + "high_curvature_threshold");
    p.bus_stop_area.use_bus_stop_area =
      node->declare_parameter<bool>(ns + "bus_stop_area.use_bus_stop_area");
    p.bus_stop_area.goal_search_interval =
      node->declare_parameter<double>(ns + "bus_stop_area.goal_search_interval");
    p.bus_stop_area.lateral_offset_interval =
      node->declare_parameter<double>(ns + "bus_stop_area.lateral_offset_interval");

    const std::string parking_policy_name =
      node->declare_parameter<std::string>(ns + "parking_policy");
    if (parking_policy_name == "left_side") {
      p.parking_policy = ParkingPolicy::LEFT_SIDE;
    } else if (parking_policy_name == "right_side") {
      p.parking_policy = ParkingPolicy::RIGHT_SIDE;
    } else {
      RCLCPP_ERROR_STREAM(
        node->get_logger(),
        "[goal_planner] invalid parking_policy: " << parking_policy_name << std::endl);
      exit(EXIT_FAILURE);
    }
  }

  // occupancy grid map
  {
    const std::string ns = base_ns + "occupancy_grid.";
    p.use_occupancy_grid_for_goal_search =
      node->declare_parameter<bool>(ns + "use_occupancy_grid_for_goal_search");
    p.use_occupancy_grid_for_path_collision_check =
      node->declare_parameter<bool>(ns + "use_occupancy_grid_for_path_collision_check");
    p.use_occupancy_grid_for_goal_longitudinal_margin =
      node->declare_parameter<bool>(ns + "use_occupancy_grid_for_goal_longitudinal_margin");
    p.occupancy_grid_collision_check_margin =
      node->declare_parameter<double>(ns + "occupancy_grid_collision_check_margin");
    p.theta_size = node->declare_parameter<int>(ns + "theta_size");
    p.obstacle_threshold = node->declare_parameter<int>(ns + "obstacle_threshold");
  }

  // object recognition
  {
    const std::string ns = base_ns + "object_recognition.";
    p.object_recognition_collision_check_soft_margins =
      node->declare_parameter<std::vector<double>>(ns + "collision_check_soft_margins");
    p.object_recognition_collision_check_hard_margins =
      node->declare_parameter<std::vector<double>>(ns + "collision_check_hard_margins");
    p.object_recognition_collision_check_max_extra_stopping_margin =
      node->declare_parameter<double>(
        ns + "object_recognition_collision_check_max_extra_stopping_margin");
    p.collision_check_outer_margin_factor =
      node->declare_parameter<double>(ns + "collision_check_outer_margin_factor");
    p.th_moving_object_velocity = node->declare_parameter<double>(ns + "th_moving_object_velocity");
    p.detection_bound_offset = node->declare_parameter<double>(ns + "detection_bound_offset");
    p.outer_road_detection_offset =
      node->declare_parameter<double>(ns + "outer_road_detection_offset");
    p.inner_road_detection_offset =
      node->declare_parameter<double>(ns + "inner_road_detection_offset");

    // validate object recognition collision check margins
    if (
      p.object_recognition_collision_check_soft_margins.empty() ||
      p.object_recognition_collision_check_hard_margins.empty()) {
      RCLCPP_FATAL_STREAM(
        node->get_logger(),
        "object_recognition.collision_check_soft_margins and "
          << "object_recognition.collision_check_hard_margins must not be empty. "
          << "Terminating the program...");
      exit(EXIT_FAILURE);
    }
  }

  // pull over general params
  {
    const std::string ns = base_ns + "pull_over.";
    p.pull_over_minimum_request_length =
      node->declare_parameter<double>(ns + "minimum_request_length");
    p.pull_over_prepare_length = node->declare_parameter<double>(ns + "pull_over_prepare_length");
    p.pull_over_velocity = node->declare_parameter<double>(ns + "pull_over_velocity");
    p.pull_over_minimum_velocity =
      node->declare_parameter<double>(ns + "pull_over_minimum_velocity");
    p.decide_path_distance = node->declare_parameter<double>(ns + "decide_path_distance");
    p.maximum_deceleration = node->declare_parameter<double>(ns + "maximum_deceleration");
    p.maximum_jerk = node->declare_parameter<double>(ns + "maximum_jerk");
    p.path_priority = node->declare_parameter<std::string>(ns + "path_priority");
    p.efficient_path_order =
      node->declare_parameter<std::vector<std::string>>(ns + "efficient_path_order");
    p.lane_departure_check_expansion_margin =
      node->declare_parameter<double>(ns + "lane_departure_check_expansion_margin");
  }

  // shift parking
  {
    const std::string ns = base_ns + "pull_over.shift_parking.";
    p.enable_shift_parking = node->declare_parameter<bool>(ns + "enable_shift_parking");
    p.shift_sampling_num = node->declare_parameter<int>(ns + "shift_sampling_num");
    p.maximum_lateral_jerk = node->declare_parameter<double>(ns + "maximum_lateral_jerk");
    p.minimum_lateral_jerk = node->declare_parameter<double>(ns + "minimum_lateral_jerk");
    p.deceleration_interval = node->declare_parameter<double>(ns + "deceleration_interval");
    p.after_shift_straight_distance =
      node->declare_parameter<double>(ns + "after_shift_straight_distance");
  }

  // parallel parking common
  {
    const std::string ns = base_ns + "pull_over.parallel_parking.";
    p.parallel_parking_parameters.center_line_path_interval =
      p.center_line_path_interval;  // for geometric parallel parking
    p.parallel_parking_parameters.max_steer_angle_margin_scale =
      node->declare_parameter<double>(ns + "max_steer_angle_margin_scale");
  }

  // forward parallel parking forward
  {
    const std::string ns = base_ns + "pull_over.parallel_parking.forward.";
    p.enable_arc_forward_parking = node->declare_parameter<bool>(ns + "enable_arc_forward_parking");
    p.parallel_parking_parameters.after_forward_parking_straight_distance =
      node->declare_parameter<double>(ns + "after_forward_parking_straight_distance");
    p.parallel_parking_parameters.forward_parking_velocity =
      node->declare_parameter<double>(ns + "forward_parking_velocity");
    p.parallel_parking_parameters.forward_parking_lane_departure_margin =
      node->declare_parameter<double>(ns + "forward_parking_lane_departure_margin");
    p.parallel_parking_parameters.forward_parking_path_interval =
      node->declare_parameter<double>(ns + "forward_parking_path_interval");
  }

  // forward parallel parking backward
  {
    const std::string ns = base_ns + "pull_over.parallel_parking.backward.";
    p.enable_arc_backward_parking =
      node->declare_parameter<bool>(ns + "enable_arc_backward_parking");
    p.parallel_parking_parameters.after_backward_parking_straight_distance =
      node->declare_parameter<double>(ns + "after_backward_parking_straight_distance");
    p.parallel_parking_parameters.backward_parking_velocity =
      node->declare_parameter<double>(ns + "backward_parking_velocity");
    p.parallel_parking_parameters.backward_parking_lane_departure_margin =
      node->declare_parameter<double>(ns + "backward_parking_lane_departure_margin");
    p.parallel_parking_parameters.backward_parking_path_interval =
      node->declare_parameter<double>(ns + "backward_parking_path_interval");
  }

  // freespace parking general params
  {
    const std::string ns = base_ns + "pull_over.freespace_parking.";
    p.enable_freespace_parking = node->declare_parameter<bool>(ns + "enable_freespace_parking");
    p.freespace_parking_algorithm =
      node->declare_parameter<std::string>(ns + "freespace_parking_algorithm");
    p.freespace_parking_velocity = node->declare_parameter<double>(ns + "velocity");
    p.vehicle_shape_margin = node->declare_parameter<double>(ns + "vehicle_shape_margin");
    p.freespace_parking_common_parameters.time_limit =
      node->declare_parameter<double>(ns + "time_limit");
    p.freespace_parking_common_parameters.max_turning_ratio =
      node->declare_parameter<double>(ns + "max_turning_ratio");
    p.freespace_parking_common_parameters.turning_steps =
      node->declare_parameter<int>(ns + "turning_steps");
  }

  //  freespace parking search config
  {
    const std::string ns = base_ns + "pull_over.freespace_parking.search_configs.";
    p.freespace_parking_common_parameters.theta_size =
      node->declare_parameter<int>(ns + "theta_size");
    p.freespace_parking_common_parameters.angle_goal_range =
      node->declare_parameter<double>(ns + "angle_goal_range");
    p.freespace_parking_common_parameters.curve_weight =
      node->declare_parameter<double>(ns + "curve_weight");
    p.freespace_parking_common_parameters.reverse_weight =
      node->declare_parameter<double>(ns + "reverse_weight");
    p.freespace_parking_common_parameters.lateral_goal_range =
      node->declare_parameter<double>(ns + "lateral_goal_range");
    p.freespace_parking_common_parameters.longitudinal_goal_range =
      node->declare_parameter<double>(ns + "longitudinal_goal_range");
  }

  //  freespace parking costmap configs
  {
    const std::string ns = base_ns + "pull_over.freespace_parking.costmap_configs.";
    p.freespace_parking_common_parameters.obstacle_threshold =
      node->declare_parameter<int>(ns + "obstacle_threshold");
  }

  //  freespace parking astar
  {
    const std::string ns = base_ns + "pull_over.freespace_parking.astar.";
    p.astar_parameters.search_method = node->declare_parameter<std::string>(ns + "search_method");
    p.astar_parameters.only_behind_solutions =
      node->declare_parameter<bool>(ns + "only_behind_solutions");
    p.astar_parameters.use_back = node->declare_parameter<bool>(ns + "use_back");
    p.astar_parameters.distance_heuristic_weight =
      node->declare_parameter<double>(ns + "distance_heuristic_weight");
  }

  //   freespace parking rrtstar
  {
    const std::string ns = base_ns + "pull_over.freespace_parking.rrtstar.";
    p.rrt_star_parameters.enable_update = node->declare_parameter<bool>(ns + "enable_update");
    p.rrt_star_parameters.use_informed_sampling =
      node->declare_parameter<bool>(ns + "use_informed_sampling");
    p.rrt_star_parameters.max_planning_time =
      node->declare_parameter<double>(ns + "max_planning_time");
    p.rrt_star_parameters.neighbor_radius = node->declare_parameter<double>(ns + "neighbor_radius");
    p.rrt_star_parameters.margin = node->declare_parameter<double>(ns + "margin");
  }

  // bezier parking
  {
    const std::string ns = base_ns + "pull_over.bezier_parking.";
    p.bezier_parking.pull_over_angle_threshold =
      node->declare_parameter<double>(ns + "pull_over_angle_threshold");
    p.bezier_parking.after_shift_straight_distance =
      node->declare_parameter<double>(ns + "after_shift_straight_distance");
  }

  // stop condition
  {
    p.maximum_deceleration_for_stop =
      node->declare_parameter<double>(base_ns + "stop_condition.maximum_deceleration_for_stop");
    p.maximum_jerk_for_stop =
      node->declare_parameter<double>(base_ns + "stop_condition.maximum_jerk_for_stop");
  }

  const std::string path_safety_check_ns = "goal_planner.path_safety_check.";

  // EgoPredictedPath
  const std::string ego_path_ns = path_safety_check_ns + "ego_predicted_path.";
  {
    p.ego_predicted_path_params.min_velocity =
      node->declare_parameter<double>(ego_path_ns + "min_velocity");
    p.ego_predicted_path_params.acceleration =
      node->declare_parameter<double>(ego_path_ns + "min_acceleration");
    p.ego_predicted_path_params.time_horizon_for_front_object =
      node->declare_parameter<double>(ego_path_ns + "time_horizon_for_front_object");
    p.ego_predicted_path_params.time_horizon_for_rear_object =
      node->declare_parameter<double>(ego_path_ns + "time_horizon_for_rear_object");
    p.ego_predicted_path_params.time_resolution =
      node->declare_parameter<double>(ego_path_ns + "time_resolution");
    p.ego_predicted_path_params.delay_until_departure =
      node->declare_parameter<double>(ego_path_ns + "delay_until_departure");
  }

  // ObjectFilteringParams
  const std::string obj_filter_ns = path_safety_check_ns + "target_filtering.";
  {
    p.objects_filtering_params.safety_check_time_horizon =
      node->declare_parameter<double>(obj_filter_ns + "safety_check_time_horizon");
    p.objects_filtering_params.safety_check_time_resolution =
      node->declare_parameter<double>(obj_filter_ns + "safety_check_time_resolution");
    p.objects_filtering_params.object_check_forward_distance =
      node->declare_parameter<double>(obj_filter_ns + "object_check_forward_distance");
    p.objects_filtering_params.object_check_backward_distance =
      node->declare_parameter<double>(obj_filter_ns + "object_check_backward_distance");
    p.objects_filtering_params.ignore_object_velocity_threshold =
      node->declare_parameter<double>(obj_filter_ns + "ignore_object_velocity_threshold");
    p.objects_filtering_params.include_opposite_lane =
      node->declare_parameter<bool>(obj_filter_ns + "include_opposite_lane");
    p.objects_filtering_params.invert_opposite_lane =
      node->declare_parameter<bool>(obj_filter_ns + "invert_opposite_lane");
    p.objects_filtering_params.check_all_predicted_path =
      node->declare_parameter<bool>(obj_filter_ns + "check_all_predicted_path");
    p.objects_filtering_params.use_all_predicted_path =
      node->declare_parameter<bool>(obj_filter_ns + "use_all_predicted_path");
    p.objects_filtering_params.use_predicted_path_outside_lanelet =
      node->declare_parameter<bool>(obj_filter_ns + "use_predicted_path_outside_lanelet");
  }

  // ObjectTypesToCheck
  const std::string obj_types_ns = obj_filter_ns + "object_types_to_check.";
  {
    p.objects_filtering_params.object_types_to_check.check_car =
      node->declare_parameter<bool>(obj_types_ns + "check_car");
    p.objects_filtering_params.object_types_to_check.check_truck =
      node->declare_parameter<bool>(obj_types_ns + "check_truck");
    p.objects_filtering_params.object_types_to_check.check_bus =
      node->declare_parameter<bool>(obj_types_ns + "check_bus");
    p.objects_filtering_params.object_types_to_check.check_trailer =
      node->declare_parameter<bool>(obj_types_ns + "check_trailer");
    p.objects_filtering_params.object_types_to_check.check_unknown =
      node->declare_parameter<bool>(obj_types_ns + "check_unknown");
    p.objects_filtering_params.object_types_to_check.check_bicycle =
      node->declare_parameter<bool>(obj_types_ns + "check_bicycle");
    p.objects_filtering_params.object_types_to_check.check_motorcycle =
      node->declare_parameter<bool>(obj_types_ns + "check_motorcycle");
    p.objects_filtering_params.object_types_to_check.check_pedestrian =
      node->declare_parameter<bool>(obj_types_ns + "check_pedestrian");
  }

  // ObjectLaneConfiguration
  const std::string obj_lane_ns = obj_filter_ns + "object_lane_configuration.";
  {
    p.objects_filtering_params.object_lane_configuration.check_current_lane =
      node->declare_parameter<bool>(obj_lane_ns + "check_current_lane");
    p.objects_filtering_params.object_lane_configuration.check_right_lane =
      node->declare_parameter<bool>(obj_lane_ns + "check_right_side_lane");
    p.objects_filtering_params.object_lane_configuration.check_left_lane =
      node->declare_parameter<bool>(obj_lane_ns + "check_left_side_lane");
    p.objects_filtering_params.object_lane_configuration.check_shoulder_lane =
      node->declare_parameter<bool>(obj_lane_ns + "check_shoulder_lane");
    p.objects_filtering_params.object_lane_configuration.check_other_lane =
      node->declare_parameter<bool>(obj_lane_ns + "check_other_lane");
  }

  // SafetyCheckParams
  const std::string safety_check_ns = path_safety_check_ns + "safety_check_params.";
  {
    p.safety_check_params.enable_safety_check =
      node->declare_parameter<bool>(safety_check_ns + "enable_safety_check");
    // NOTE(soblin): remove safety_check_params.enable_safety_check because it does not make sense
    if (!p.safety_check_params.enable_safety_check) {
      throw std::domain_error("goal_planner never works without safety check");
    }
    p.safety_check_params.keep_unsafe_time =
      node->declare_parameter<double>(safety_check_ns + "keep_unsafe_time");
    p.safety_check_params.method = node->declare_parameter<std::string>(safety_check_ns + "method");
    p.safety_check_params.hysteresis_factor_expand_rate =
      node->declare_parameter<double>(safety_check_ns + "hysteresis_factor_expand_rate");
    p.safety_check_params.collision_check_yaw_diff_threshold =
      node->declare_parameter<double>(safety_check_ns + "collision_check_yaw_diff_threshold");
    p.safety_check_params.backward_path_length =
      node->declare_parameter<double>(safety_check_ns + "backward_path_length");
    p.safety_check_params.forward_path_length =
      node->declare_parameter<double>(safety_check_ns + "forward_path_length");
    p.safety_check_params.publish_debug_marker =
      node->declare_parameter<bool>(safety_check_ns + "publish_debug_marker");
  }

  // RSSparams
  const std::string rss_ns = safety_check_ns + "rss_params.";
  {
    p.safety_check_params.rss_params.rear_vehicle_reaction_time =
      node->declare_parameter<double>(rss_ns + "rear_vehicle_reaction_time");
    p.safety_check_params.rss_params.rear_vehicle_safety_time_margin =
      node->declare_parameter<double>(rss_ns + "rear_vehicle_safety_time_margin");
    p.safety_check_params.rss_params.lateral_distance_max_threshold =
      node->declare_parameter<double>(rss_ns + "lateral_distance_max_threshold");
    p.safety_check_params.rss_params.longitudinal_distance_min_threshold =
      node->declare_parameter<double>(rss_ns + "longitudinal_distance_min_threshold");
    p.safety_check_params.rss_params.longitudinal_velocity_delta_time =
      node->declare_parameter<double>(rss_ns + "longitudinal_velocity_delta_time");
  }

  // IntegralPredictedPolygonParams
  const std::string integral_ns = safety_check_ns + "integral_predicted_polygon_params.";
  {
    p.safety_check_params.integral_predicted_polygon_params.forward_margin =
      node->declare_parameter<double>(integral_ns + "forward_margin");
    p.safety_check_params.integral_predicted_polygon_params.backward_margin =
      node->declare_parameter<double>(integral_ns + "backward_margin");
    p.safety_check_params.integral_predicted_polygon_params.lat_margin =
      node->declare_parameter<double>(integral_ns + "lat_margin");
    p.safety_check_params.integral_predicted_polygon_params.time_horizon =
      node->declare_parameter<double>(integral_ns + "time_horizon");
  }

  // debug
  {
    const std::string ns = base_ns + "debug.";
    p.print_debug_info = node->declare_parameter<bool>(ns + "print_debug_info");
  }

  // path decision state controller
  {
    const std::string ns = base_ns + "path_decision_state_controller.";
    p.check_collision_duration = node->declare_parameter<double>(ns + "check_collision_duration");
  }

  // validation of parameters
  if (p.shift_sampling_num < 1) {
    RCLCPP_FATAL_STREAM(
      node->get_logger(), "shift_sampling_num must be positive integer. Given parameter: "
                            << p.shift_sampling_num << std::endl
                            << "Terminating the program...");
    exit(EXIT_FAILURE);
  }
  if (p.maximum_deceleration < 0.0) {
    RCLCPP_FATAL_STREAM(
      node->get_logger(), "maximum_deceleration cannot be negative value. Given parameter: "
                            << p.maximum_deceleration << std::endl
                            << "Terminating the program...");
    exit(EXIT_FAILURE);
  }
  return p;
}

void GoalPlannerModuleManager::init(rclcpp::Node * node)
{
  // init manager interface
  initInterface(node, {""});

  const std::string base_ns = "goal_planner.";
  parameters_ = std::make_shared<GoalPlannerParameters>(initGoalPlannerParameters(node, base_ns));
}

void GoalPlannerModuleManager::updateModuleParams(
  [[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters)
{
  // TODO(someone): This function does not check OR update
  // object_recognition_collision_check_soft_margins,
  // object_recognition_collision_check_hard_margins, maximum_deceleration, shift_sampling_num or
  // parking_policy, there seems to be a problem when we use a temp value to check these values.

  using autoware_utils::update_param;

  auto & p = parameters_;

  const std::string base_ns = "goal_planner.";
  // general params

  {
    update_param<double>(parameters, base_ns + "th_stopped_velocity", p->th_stopped_velocity);
    update_param<double>(parameters, base_ns + "th_arrived_distance", p->th_arrived_distance);
    update_param<double>(parameters, base_ns + "th_stopped_time", p->th_stopped_time);

    update_param<double>(
      parameters, base_ns + "center_line_path_interval", p->center_line_path_interval);
  }
  // goal search

  {
    const std::string ns = base_ns + "goal_search.";
    update_param<std::string>(parameters, ns + "goal_priority", p->goal_priority);
    update_param<double>(
      parameters, ns + "minimum_weighted_distance.lateral_weight",
      p->minimum_weighted_distance_lateral_weight);
    update_param<bool>(
      parameters, ns + "prioritize_goals_before_objects", p->prioritize_goals_before_objects);
    update_param<double>(
      parameters, ns + "forward_goal_search_length", p->forward_goal_search_length);
    update_param<double>(
      parameters, ns + "backward_goal_search_length", p->backward_goal_search_length);
    update_param<double>(parameters, ns + "goal_search_interval", p->goal_search_interval);
    update_param<double>(parameters, ns + "longitudinal_margin", p->longitudinal_margin);
    update_param<double>(parameters, ns + "max_lateral_offset", p->max_lateral_offset);
    update_param<double>(parameters, ns + "lateral_offset_interval", p->lateral_offset_interval);
    update_param<double>(
      parameters, ns + "ignore_distance_from_lane_start", p->ignore_distance_from_lane_start);
    update_param<double>(parameters, ns + "margin_from_boundary", p->margin_from_boundary);
  }

  // occupancy grid map
  {
    const std::string ns = base_ns + "occupancy_grid.";
    update_param<bool>(
      parameters, ns + "use_occupancy_grid_for_goal_search", p->use_occupancy_grid_for_goal_search);
    update_param<bool>(
      parameters, ns + "use_occupancy_grid_for_path_collision_check",
      p->use_occupancy_grid_for_path_collision_check);
    update_param<bool>(
      parameters, ns + "use_occupancy_grid_for_goal_longitudinal_margin",
      p->use_occupancy_grid_for_goal_longitudinal_margin);
    update_param<double>(
      parameters, ns + "occupancy_grid_collision_check_margin",
      p->occupancy_grid_collision_check_margin);
    update_param<int>(parameters, ns + "theta_size", p->theta_size);
    update_param<int>(parameters, ns + "obstacle_threshold", p->obstacle_threshold);
  }

  // object recognition
  {
    const std::string ns = base_ns + "object_recognition.";

    update_param(
      parameters, ns + "object_recognition_collision_check_max_extra_stopping_margin",
      p->object_recognition_collision_check_max_extra_stopping_margin);
    update_param(parameters, ns + "th_moving_object_velocity", p->th_moving_object_velocity);
    update_param(parameters, ns + "detection_bound_offset", p->detection_bound_offset);
    update_param(parameters, ns + "outer_road_detection_offset", p->outer_road_detection_offset);
    update_param(parameters, ns + "inner_road_detection_offset", p->inner_road_detection_offset);
  }

  // pull over general params
  {
    const std::string ns = base_ns + "pull_over.";

    update_param<double>(
      parameters, ns + "minimum_request_length", p->pull_over_minimum_request_length);
    update_param<double>(parameters, ns + "pull_over_velocity", p->pull_over_velocity);
    update_param<double>(
      parameters, ns + "pull_over_minimum_velocity", p->pull_over_minimum_velocity);
    update_param<double>(parameters, ns + "decide_path_distance", p->decide_path_distance);
    update_param<double>(parameters, ns + "maximum_jerk", p->maximum_jerk);
    update_param<std::string>(parameters, ns + "path_priority", p->path_priority);
    update_param<std::vector<std::string>>(
      parameters, ns + "efficient_path_order", p->efficient_path_order);
  }

  // shift parking
  {
    const std::string ns = base_ns + "pull_over.shift_parking.";
    update_param<bool>(parameters, ns + "enable_shift_parking", p->enable_shift_parking);
    update_param<double>(parameters, ns + "maximum_lateral_jerk", p->maximum_lateral_jerk);
    update_param<double>(parameters, ns + "minimum_lateral_jerk", p->minimum_lateral_jerk);
    update_param<double>(parameters, ns + "deceleration_interval", p->deceleration_interval);
    update_param<double>(
      parameters, ns + "after_shift_straight_distance", p->after_shift_straight_distance);
    update_param<double>(
      parameters, ns + "lane_departure_check_expansion_margin",
      p->lane_departure_check_expansion_margin);
  }

  // parallel parking common
  {
    const std::string ns = base_ns + "pull_over.parallel_parking.";
    p->parallel_parking_parameters.center_line_path_interval =
      p->center_line_path_interval;  // for geometric parallel parking
    update_param<double>(
      parameters, ns + "max_steer_angle_margin_scale",
      p->parallel_parking_parameters.max_steer_angle_margin_scale);
  }

  // forward parallel parking forward
  {
    const std::string ns = base_ns + "pull_over.parallel_parking.forward.";
    update_param<bool>(
      parameters, ns + "enable_arc_forward_parking", p->enable_arc_forward_parking);
    update_param<double>(
      parameters, ns + "after_forward_parking_straight_distance",
      p->parallel_parking_parameters.after_forward_parking_straight_distance);
    update_param<double>(
      parameters, ns + "forward_parking_velocity",
      p->parallel_parking_parameters.forward_parking_velocity);
    update_param<double>(
      parameters, ns + "forward_parking_lane_departure_margin",
      p->parallel_parking_parameters.forward_parking_lane_departure_margin);
    update_param<double>(
      parameters, ns + "forward_parking_path_interval",
      p->parallel_parking_parameters.forward_parking_path_interval);
  }

  // forward parallel parking backward
  {
    const std::string ns = base_ns + "pull_over.parallel_parking.backward.";
    update_param<bool>(
      parameters, ns + "enable_arc_backward_parking", p->enable_arc_backward_parking);
    update_param<double>(
      parameters, ns + "after_backward_parking_straight_distance",
      p->parallel_parking_parameters.after_backward_parking_straight_distance);
    update_param<double>(
      parameters, ns + "backward_parking_velocity",
      p->parallel_parking_parameters.backward_parking_velocity);
    update_param<double>(
      parameters, ns + "backward_parking_lane_departure_margin",
      p->parallel_parking_parameters.backward_parking_lane_departure_margin);
    update_param<double>(
      parameters, ns + "backward_parking_path_interval",
      p->parallel_parking_parameters.backward_parking_path_interval);
  }

  // freespace parking general params
  {
    const std::string ns = base_ns + "pull_over.freespace_parking.";
    update_param<bool>(parameters, ns + "enable_freespace_parking", p->enable_freespace_parking);
    update_param<std::string>(
      parameters, ns + "freespace_parking_algorithm", p->freespace_parking_algorithm);
    update_param<double>(parameters, ns + "velocity", p->freespace_parking_velocity);

    update_param<double>(parameters, ns + "vehicle_shape_margin", p->vehicle_shape_margin);
    update_param<double>(
      parameters, ns + "time_limit", p->freespace_parking_common_parameters.time_limit);
    update_param<double>(
      parameters, ns + "max_turning_ratio",
      p->freespace_parking_common_parameters.max_turning_ratio);
    update_param<int>(
      parameters, ns + "turning_steps", p->freespace_parking_common_parameters.turning_steps);
  }

  //  freespace parking search config
  {
    const std::string ns = base_ns + "pull_over.freespace_parking.search_configs.";
    update_param<int>(
      parameters, ns + "theta_size", p->freespace_parking_common_parameters.theta_size);
    update_param<double>(
      parameters, ns + "angle_goal_range", p->freespace_parking_common_parameters.angle_goal_range);
    update_param<double>(
      parameters, ns + "curve_weight", p->freespace_parking_common_parameters.curve_weight);
    update_param<double>(
      parameters, ns + "reverse_weight", p->freespace_parking_common_parameters.reverse_weight);
    update_param<double>(
      parameters, ns + "lateral_goal_range",
      p->freespace_parking_common_parameters.lateral_goal_range);
    update_param<double>(
      parameters, ns + "longitudinal_goal_range",
      p->freespace_parking_common_parameters.longitudinal_goal_range);
  }

  //  freespace parking costmap configs
  {
    const std::string ns = base_ns + "pull_over.freespace_parking.costmap_configs.";
    update_param<int>(
      parameters, ns + "obstacle_threshold",
      p->freespace_parking_common_parameters.obstacle_threshold);
  }

  //  freespace parking astar
  {
    const std::string ns = base_ns + "pull_over.freespace_parking.astar.";
    update_param<std::string>(parameters, ns + "search_method", p->astar_parameters.search_method);
    update_param<bool>(
      parameters, ns + "only_behind_solutions", p->astar_parameters.only_behind_solutions);
    update_param<bool>(parameters, ns + "use_back", p->astar_parameters.use_back);
    update_param<double>(
      parameters, ns + "distance_heuristic_weight", p->astar_parameters.distance_heuristic_weight);
  }

  //  freespace parking rrtstar

  {
    const std::string ns = base_ns + "pull_over.freespace_parking.rrtstar.";
    update_param<bool>(parameters, ns + "enable_update", p->rrt_star_parameters.enable_update);
    update_param<bool>(
      parameters, ns + "use_informed_sampling", p->rrt_star_parameters.use_informed_sampling);
    update_param<double>(
      parameters, ns + "max_planning_time", p->rrt_star_parameters.max_planning_time);
    update_param<double>(
      parameters, ns + "neighbor_radius", p->rrt_star_parameters.neighbor_radius);
    update_param<double>(parameters, ns + "margin", p->rrt_star_parameters.margin);
  }

  // stop condition
  {
    update_param<double>(
      parameters, base_ns + "stop_condition.maximum_deceleration_for_stop",
      p->maximum_deceleration_for_stop);
    update_param<double>(
      parameters, base_ns + "stop_condition.maximum_jerk_for_stop", p->maximum_jerk_for_stop);
  }

  const std::string path_safety_check_ns = "goal_planner.path_safety_check.";
  const std::string ego_path_ns = path_safety_check_ns + "ego_predicted_path.";

  // EgoPredictedPath
  {
    update_param<double>(
      parameters, ego_path_ns + "min_velocity", p->ego_predicted_path_params.min_velocity);
    update_param<double>(
      parameters, ego_path_ns + "min_acceleration", p->ego_predicted_path_params.acceleration);
    update_param<double>(
      parameters, ego_path_ns + "time_horizon_for_front_object",
      p->ego_predicted_path_params.time_horizon_for_front_object);
    update_param<double>(
      parameters, ego_path_ns + "time_horizon_for_rear_object",
      p->ego_predicted_path_params.time_horizon_for_rear_object);
    update_param<double>(
      parameters, ego_path_ns + "time_resolution", p->ego_predicted_path_params.time_resolution);
    update_param<double>(
      parameters, ego_path_ns + "delay_until_departure",
      p->ego_predicted_path_params.delay_until_departure);
  }

  // ObjectFilteringParams
  const std::string obj_filter_ns = path_safety_check_ns + "target_filtering.";
  {
    update_param<double>(
      parameters, obj_filter_ns + "safety_check_time_horizon",
      p->objects_filtering_params.safety_check_time_horizon);
    update_param<double>(
      parameters, obj_filter_ns + "safety_check_time_resolution",
      p->objects_filtering_params.safety_check_time_resolution);
    update_param<double>(
      parameters, obj_filter_ns + "object_check_forward_distance",
      p->objects_filtering_params.object_check_forward_distance);
    update_param<double>(
      parameters, obj_filter_ns + "object_check_backward_distance",
      p->objects_filtering_params.object_check_backward_distance);
    update_param<double>(
      parameters, obj_filter_ns + "ignore_object_velocity_threshold",
      p->objects_filtering_params.ignore_object_velocity_threshold);
    update_param<bool>(
      parameters, obj_filter_ns + "include_opposite_lane",
      p->objects_filtering_params.include_opposite_lane);
    update_param<bool>(
      parameters, obj_filter_ns + "invert_opposite_lane",
      p->objects_filtering_params.invert_opposite_lane);
    update_param<bool>(
      parameters, obj_filter_ns + "check_all_predicted_path",
      p->objects_filtering_params.check_all_predicted_path);
    update_param<bool>(
      parameters, obj_filter_ns + "use_all_predicted_path",
      p->objects_filtering_params.use_all_predicted_path);
    update_param<bool>(
      parameters, obj_filter_ns + "use_predicted_path_outside_lanelet",
      p->objects_filtering_params.use_predicted_path_outside_lanelet);
  }

  // ObjectTypesToCheck
  const std::string obj_types_ns = obj_filter_ns + "object_types_to_check.";
  {
    update_param<bool>(
      parameters, obj_types_ns + "check_car",
      p->objects_filtering_params.object_types_to_check.check_car);
    update_param<bool>(
      parameters, obj_types_ns + "check_truck",
      p->objects_filtering_params.object_types_to_check.check_truck);
    update_param<bool>(
      parameters, obj_types_ns + "check_bus",
      p->objects_filtering_params.object_types_to_check.check_bus);
    update_param<bool>(
      parameters, obj_types_ns + "check_trailer",
      p->objects_filtering_params.object_types_to_check.check_trailer);
    update_param<bool>(
      parameters, obj_types_ns + "check_unknown",
      p->objects_filtering_params.object_types_to_check.check_unknown);
    update_param<bool>(
      parameters, obj_types_ns + "check_bicycle",
      p->objects_filtering_params.object_types_to_check.check_bicycle);
    update_param<bool>(
      parameters, obj_types_ns + "check_motorcycle",
      p->objects_filtering_params.object_types_to_check.check_motorcycle);
    update_param<bool>(
      parameters, obj_types_ns + "check_pedestrian",
      p->objects_filtering_params.object_types_to_check.check_pedestrian);
  }
  // ObjectLaneConfiguration
  const std::string obj_lane_ns = obj_filter_ns + "object_lane_configuration.";
  {
    update_param<bool>(
      parameters, obj_lane_ns + "check_current_lane",
      p->objects_filtering_params.object_lane_configuration.check_current_lane);
    update_param<bool>(
      parameters, obj_lane_ns + "check_right_side_lane",
      p->objects_filtering_params.object_lane_configuration.check_right_lane);
    update_param<bool>(
      parameters, obj_lane_ns + "check_left_side_lane",
      p->objects_filtering_params.object_lane_configuration.check_left_lane);
    update_param<bool>(
      parameters, obj_lane_ns + "check_shoulder_lane",
      p->objects_filtering_params.object_lane_configuration.check_shoulder_lane);
    update_param<bool>(
      parameters, obj_lane_ns + "check_other_lane",
      p->objects_filtering_params.object_lane_configuration.check_other_lane);
  }

  // SafetyCheckParams
  const std::string safety_check_ns = path_safety_check_ns + "safety_check_params.";
  {
    update_param<double>(
      parameters, safety_check_ns + "keep_unsafe_time", p->safety_check_params.keep_unsafe_time);
    update_param<std::string>(
      parameters, safety_check_ns + "method", p->safety_check_params.method);
    update_param<double>(
      parameters, safety_check_ns + "hysteresis_factor_expand_rate",
      p->safety_check_params.hysteresis_factor_expand_rate);
    update_param<double>(
      parameters, safety_check_ns + "collision_check_yaw_diff_threshold",
      p->safety_check_params.collision_check_yaw_diff_threshold);
    update_param<double>(
      parameters, safety_check_ns + "backward_path_length",
      p->safety_check_params.backward_path_length);
    update_param<double>(
      parameters, safety_check_ns + "forward_path_length",
      p->safety_check_params.forward_path_length);
    update_param<bool>(
      parameters, safety_check_ns + "publish_debug_marker",
      p->safety_check_params.publish_debug_marker);
  }

  // RSSparams
  const std::string rss_ns = safety_check_ns + "rss_params.";
  {
    update_param<double>(
      parameters, rss_ns + "rear_vehicle_reaction_time",
      p->safety_check_params.rss_params.rear_vehicle_reaction_time);
    update_param<double>(
      parameters, rss_ns + "rear_vehicle_safety_time_margin",
      p->safety_check_params.rss_params.rear_vehicle_safety_time_margin);
    update_param<double>(
      parameters, rss_ns + "lateral_distance_max_threshold",
      p->safety_check_params.rss_params.lateral_distance_max_threshold);
    update_param<double>(
      parameters, rss_ns + "longitudinal_distance_min_threshold",
      p->safety_check_params.rss_params.longitudinal_distance_min_threshold);
    update_param<double>(
      parameters, rss_ns + "longitudinal_velocity_delta_time",
      p->safety_check_params.rss_params.longitudinal_velocity_delta_time);
  }

  // IntegralPredictedPolygonParams
  const std::string integral_ns = safety_check_ns + "integral_predicted_polygon_params.";
  {
    update_param<double>(
      parameters, integral_ns + "forward_margin",
      p->safety_check_params.integral_predicted_polygon_params.forward_margin);
    update_param<double>(
      parameters, integral_ns + "backward_margin",
      p->safety_check_params.integral_predicted_polygon_params.backward_margin);
    update_param<double>(
      parameters, integral_ns + "lat_margin",
      p->safety_check_params.integral_predicted_polygon_params.lat_margin);
    update_param<double>(
      parameters, integral_ns + "time_horizon",
      p->safety_check_params.integral_predicted_polygon_params.time_horizon);
  }

  // debug
  {
    const std::string ns = base_ns + "debug.";
    update_param<bool>(parameters, ns + "print_debug_info", p->print_debug_info);
  }

  // path decision state controller
  {
    const std::string ns = base_ns + "path_decision_state_controller.";
    update_param<double>(parameters, ns + "check_collision_duration", p->check_collision_duration);
  }

  std::for_each(observers_.begin(), observers_.end(), [&p](const auto & observer) {
    if (!observer.expired()) observer.lock()->updateModuleParams(p);
  });
}

bool GoalPlannerModuleManager::isSimultaneousExecutableAsApprovedModule() const
{
  // enable SimultaneousExecutable whenever goal modification is not allowed
  // because only minor path refinements are made for fixed goals
  if (!utils::isAllowedGoalModification(planner_data_->route_handler)) {
    return true;
  }

  return config_.enable_simultaneous_execution_as_approved_module;
}

bool GoalPlannerModuleManager::isSimultaneousExecutableAsCandidateModule() const
{
  // enable SimultaneousExecutable whenever goal modification is not allowed
  // because only minor path refinements are made for fixed goals
  if (!utils::isAllowedGoalModification(planner_data_->route_handler)) {
    return true;
  }

  return config_.enable_simultaneous_execution_as_candidate_module;
}

}  // namespace autoware::behavior_path_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_path_planner::GoalPlannerModuleManager,
  autoware::behavior_path_planner::SceneModuleManagerInterface)
