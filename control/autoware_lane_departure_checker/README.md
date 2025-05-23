# Lane Departure Checker

The **Lane Departure Checker** checks if vehicle follows a trajectory. If it does not follow the trajectory, it reports its status via `diagnostic_updater`.

## Features

This package includes the following features:

- **Lane Departure**: Check if ego vehicle is going to be out of lane boundaries based on output from control module (predicted trajectory).
- **Trajectory Deviation**: Check if ego vehicle's pose does not deviate from the trajectory. Checking lateral, longitudinal and yaw deviation.
- **Road Border Departure**: Check if ego vehicle's footprint, generated from the control's output, extends beyond the road border.

## Inner-workings / Algorithms

### How to extend footprint by covariance

1. Calculate the standard deviation of error ellipse(covariance) in vehicle coordinate.

   1.Transform covariance into vehicle coordinate.

   $$
   \begin{align}
   \left( \begin{array}{cc} x_{vehicle}\\ y_{vehicle}\\ \end{array} \right) = R_{map2vehicle}  \left( \begin{array}{cc} x_{map}\\ y_{map}\\ \end{array} \right)
   \end{align}
   $$

   Calculate covariance in vehicle coordinate.

   $$
   \begin{align}
   Cov_{vehicle} &= E \left[
   \left( \begin{array}{cc} x_{vehicle}\\ y_{vehicle}\\ \end{array} \right) (x_{vehicle}, y_{vehicle}) \right] \\
   &= E \left[ R\left( \begin{array}{cc} x_{map}\\ y_{map}\\ \end{array} \right)
   (x_{map}, y_{map})R^t
   \right] \\
   &= R E\left[ \left( \begin{array}{cc} x_{map}\\ y_{map}\\ \end{array} \right)
   (x_{map}, y_{map})
   \right] R^t \\
   &= R Cov_{map} R^t
   \end{align}
   $$

   2.The longitudinal length we want to expand is correspond to marginal distribution of $x_{vehicle}$, which is represented in $Cov_{vehicle}(0,0)$. In the same way, the lateral length is represented in $Cov_{vehicle}(1,1)$. Wikipedia reference [here](https://en.wikipedia.org/wiki/Multivariate_normal_distribution#Marginal_distributions).

2. Expand footprint based on the standard deviation multiplied with `footprint_margin_scale`.

## Interface

### Input

- /localization/kinematic_state [`nav_msgs::msg::Odometry`]
- /map/vector_map [`autoware_map_msgs::msg::LaneletMapBin`]
- /planning/mission_planning/route [`autoware_planning_msgs::msg::LaneletRoute`]
- /planning/scenario_planning/trajectory [`autoware_planning_msgs::msg::Trajectory`]
- /control/trajectory_follower/predicted_trajectory [`autoware_planning_msgs::msg::Trajectory`]

### Output

- [`diagnostic_updater`] lane_departure : Update diagnostic level when ego vehicle is out of lane.

## Parameters

### Node Parameters

#### General Parameters

| Name                       | Type   | Description                                                                                                 | Default value |
| :------------------------- | :----- | :---------------------------------------------------------------------------------------------------------- | :------------ |
| will_out_of_lane_checker   | bool   | Enable checker whether ego vehicle footprint will depart from lane                                          | True          |
| out_of_lane_checker        | bool   | Enable checker whether ego vehicle footprint is out of lane                                                 | True          |
| boundary_departure_checker | bool   | Enable checker whether ego vehicle footprint wil depart from boundary specified by boundary_types_to_detect | False         |
| update_rate                | double | Frequency for publishing [Hz]                                                                               | 10.0          |
| visualize_lanelet          | bool   | Flag for visualizing lanelet                                                                                | False         |

#### Parameters For Lane Departure

| Name                      | Type | Description                                       | Default value |
| :------------------------ | :--- | :------------------------------------------------ | :------------ |
| include_right_lanes       | bool | Flag for including right lanelet in borders       | False         |
| include_left_lanes        | bool | Flag for including left lanelet in borders        | False         |
| include_opposite_lanes    | bool | Flag for including opposite lanelet in borders    | False         |
| include_conflicting_lanes | bool | Flag for including conflicting lanelet in borders | False         |

#### Parameters For Road Border Departure

| Name                     | Type                       | Description                                                 | Default value |
| :----------------------- | :------------------------- | :---------------------------------------------------------- | :------------ |
| boundary_types_to_detect | std::vector\<std::string\> | line_string types to detect with boundary_departure_checker | [road_border] |

### Core Parameters

| Name                       | Type   | Description                                                                        | Default value |
| :------------------------- | :----- | :--------------------------------------------------------------------------------- | :------------ |
| footprint_margin_scale     | double | Coefficient for expanding footprint margin. Multiplied by 1 standard deviation.    | 1.0           |
| footprint_extra_margin     | double | Coefficient for expanding footprint margin. When checking for lane departure       | 0.0           |
| resample_interval          | double | Minimum Euclidean distance between points when resample trajectory.[m]             | 0.3           |
| max_deceleration           | double | Maximum deceleration when calculating braking distance.                            | 2.8           |
| delay_time                 | double | Delay time which took to actuate brake when calculating braking distance. [second] | 1.3           |
| max_lateral_deviation      | double | Maximum lateral deviation in vehicle coordinate. [m]                               | 2.0           |
| max_longitudinal_deviation | double | Maximum longitudinal deviation in vehicle coordinate. [m]                          | 2.0           |
| max_yaw_deviation_deg      | double | Maximum ego yaw deviation from trajectory. [deg]                                   | 60.0          |
