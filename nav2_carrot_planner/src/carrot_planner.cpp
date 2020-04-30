// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2018 Simbe Robotics
// Copyright (c) 2019 Samsung Research America
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

// Navigation Strategy based on:
// Brock, O. and Oussama K. (1999). High-Speed Navigation Using
// the Global Dynamic Window Approach. IEEE.
// https://cs.stanford.edu/group/manips/publications/pdfs/Brock_1999_ICRA.pdf

#include "nav2_carrot_planner/carrot_planner.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "nav2_navfn_planner/navfn.hpp"
#include "nav2_util/costmap.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

using namespace std::chrono_literals;
using nav2_util::declare_parameter_if_not_declared;

namespace nav2_carrot_planner
{

double pose_distance(
  const geometry_msgs::msg::Pose & pose1,
  const geometry_msgs::msg::Pose & pose2)
{
  double dx = pose1.position.x - pose2.position.x;
  double dy = pose1.position.y - pose2.position.y;
  double dz = pose1.position.z - pose2.position.z;

  // We could use it if we activate c++17, but it produces some compilation errors
  // return std::hypot(dx, dy, dz);  

  return sqrt(dx * dx + dy * dy + dz * dz);
}

CarrotPlanner::CarrotPlanner()
{
}

CarrotPlanner::~CarrotPlanner()
{
  RCLCPP_INFO(
    node_->get_logger(), "Destroying plugin %s of type CarrotPlanner",
    name_.c_str());
}

void
CarrotPlanner::configure(
  rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  NavfnPlanner::configure(parent, name, tf, costmap_ros);

  // Initialize parameters
  // Declare this plugin's parameters
  declare_parameter_if_not_declared(node_, name + ".carrot_distance", rclcpp::ParameterValue(1.0));
  node_->get_parameter(name + ".carrot_distance", carrot_distance_);
}

nav_msgs::msg::Path CarrotPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  // Update planner based on the new costmap size
  if (isPlannerOutOfDate()) {
    planner_->setNavArr(
      costmap_->getSizeInCellsX(),
      costmap_->getSizeInCellsY());
  }

  nav_msgs::msg::Path path;

  if (!makePlan(start.pose, goal.pose, tolerance_, path)) {
    RCLCPP_WARN(
      node_->get_logger(), "%s: failed to create plan with "
      "tolerance %.2f.", name_.c_str(), tolerance_);
  }

  double distance = 0.0;
  geometry_msgs::msg::PoseStamped last = path.poses.back();
  do {
    geometry_msgs::msg::PoseStamped current = path.poses.back();
    path.poses.pop_back();

    distance = distance + pose_distance(current.pose, path.poses.back().pose);
  } while (distance < carrot_distance_);


  // Set orientation of last path pose facing to goal
  double last_dx = last.pose.position.x - path.poses.back().pose.position.x;
  double last_dy = last.pose.position.y - path.poses.back().pose.position.y;
  double final_angle = atan2(last_dy, last_dx);

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, final_angle);

  path.poses.back().pose.orientation.x = q.x();
  path.poses.back().pose.orientation.y = q.y();
  path.poses.back().pose.orientation.z = q.z();
  path.poses.back().pose.orientation.w = q.w();

  return path;
}

}  // namespace nav2_carrot_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_carrot_planner::CarrotPlanner, nav2_core::GlobalPlanner)
