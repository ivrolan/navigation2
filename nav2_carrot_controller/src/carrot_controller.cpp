/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>


#include "nav2_carrot_controller/carrot_controller.hpp"

#include "nav_2d_msgs/msg/twist2_d.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "nav_2d_utils/parameters.hpp"
#include "nav_2d_utils/tf_help.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

using nav2_util::declare_parameter_if_not_declared;

namespace nav2_carrot_controller
{

CarrotController::CarrotController()
{
}

void
CarrotController::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
{
  DWBLocalPlanner::configure(node, name, tf, costmap_ros);

  update_point_sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
    "clicked_point", 
    rclcpp::SystemDefaultsQoS(),
    std::bind(&CarrotController::onGoalPointReceived, this, std::placeholders::_1));

  declare_parameter_if_not_declared(
    node_, name + ".carrot_distance",
    rclcpp::ParameterValue(1.0));

  declare_parameter_if_not_declared(
    node_, name + ".goal_point_valid_duration",
    rclcpp::ParameterValue(5.0));

  node_->get_parameter(name + ".carrot_distance", carrot_distance_);
  node_->get_parameter(name + ".goal_point_valid_duration", goal_point_valid_duration_);
}

bool
CarrotController::isGoalReached(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity)
{
  RCLCPP_DEBUG(node_->get_logger(), "isGoalReached: Ignoring pose (%lf, %lf) and vel (%lf, %lf)",
    pose.pose.position.x, pose.pose.position.y, velocity.linear.x, velocity.angular.z);
  return false;
}

void
CarrotController::setPlan(const nav_msgs::msg::Path & path)
{
  if ((node_->now() - goal_point_.header.stamp).seconds() < goal_point_valid_duration_) {
    std::cerr << "===============================================" << std::endl;

    const std::string working_frame = "odom"; 

    nav_2d_msgs::msg::Pose2DStamped start_pose;
    start_pose.header.frame_id = costmap_ros_->getBaseFrameID();
    start_pose.header.stamp = goal_point_.header.stamp;
    start_pose.pose.x = 0;
    start_pose.pose.y = 0;
    start_pose.pose.theta = 0;

    nav_2d_utils::transformPose(
      tf_, working_frame, start_pose,
      start_pose, transform_tolerance_);

    std::cerr << "Going from [" << working_frame << "] ("<< start_pose.pose.x << ", " << start_pose.pose.y << ")" << std::endl;

    nav_2d_msgs::msg::Pose2DStamped goal_pose;

    goal_pose.header = goal_point_.header;
    goal_pose.pose.x = goal_point_.point.x;
    goal_pose.pose.y = goal_point_.point.y;
    goal_pose.pose.theta = 0.0;  // it doesn't really matter

    std::cerr << "Going to [" << goal_pose.header.frame_id << "] ("<< goal_pose.pose.x << ", " << goal_pose.pose.y << ")" << std::endl;

    nav_2d_utils::transformPose(
      tf_, working_frame, goal_pose,
      goal_pose, transform_tolerance_);

    std::cerr << "Going to [" << working_frame << "] ("<< goal_pose.pose.x << ", " << goal_pose.pose.y << ")" << std::endl;


    double distance_to_goal = std::hypot(goal_pose.pose.x - start_pose.pose.x, goal_pose.pose.y - start_pose.pose.y);

    std::cerr << "Distance: " << distance_to_goal << std::endl;

    if (distance_to_goal > 0.0) 
    {
      double dx = (goal_pose.pose.x - start_pose.pose.x) / distance_to_goal;
      double dy = (goal_pose.pose.y - start_pose.pose.y) / distance_to_goal;
      double angle = atan2(dy, dx);

      std::cerr << "uv = (" << dx << ", " << dy<< ") " << std::endl;

      // Apply carrot_distance to goal
      goal_pose.pose.x = goal_pose.pose.x - std::min(carrot_distance_, distance_to_goal) * dx;
      goal_pose.pose.y = goal_pose.pose.y - std::min(carrot_distance_, distance_to_goal) * dy;

      dx = (goal_pose.pose.x - start_pose.pose.x) / distance_to_goal;
      dy = (goal_pose.pose.y - start_pose.pose.y) / distance_to_goal;
      angle = atan2(dy, dx);

      nav_msgs::msg::Path new_path;
      new_path.header.stamp = node_->now();
      new_path.header.frame_id = working_frame;
      
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, angle);

      geometry_msgs::msg::PoseStamped aux_pose;
      aux_pose.header.stamp = node_->now();
      aux_pose.header.frame_id = working_frame;
      aux_pose.pose.position.z = 0.0;
      aux_pose.pose.orientation.x = q.x();
      aux_pose.pose.orientation.y = q.y();
      aux_pose.pose.orientation.z = q.z();
      aux_pose.pose.orientation.w = q.w();


      double acum_distance = 0.0;
      double step_dist = distance_to_goal / 10.0;
      while (acum_distance < distance_to_goal) 
      {
        acum_distance = acum_distance + step_dist;

        aux_pose.pose.position.x = start_pose.pose.x + acum_distance * dx * distance_to_goal;
        aux_pose.pose.position.y = start_pose.pose.y + acum_distance * dy * distance_to_goal;
        
        std::cerr << "Adding path point [" << working_frame << "] (" << aux_pose.pose.position.x << ", " << aux_pose.pose.position.y << ", " << angle << ") dist  = " << acum_distance<< " / " << distance_to_goal << std::endl;

        new_path.poses.push_back(aux_pose);
      }
        std::cerr << "Setting plan of " << new_path.poses.size() << "Poses" <<std::endl;

      DWBLocalPlanner::setPlan(new_path);
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Distance is zero when calculating path");
    }
  } else {
    DWBLocalPlanner::setPlan(path);
  }
}

geometry_msgs::msg::TwistStamped
CarrotController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity)
{
  return DWBLocalPlanner::computeVelocityCommands(pose, velocity);
}

void
CarrotController::onGoalPointReceived(const geometry_msgs::msg::PointStamped::SharedPtr point)
{
  goal_point_ = *point;

  // Temporal workaround until Rviz 8.0.0, as RViz2 is not using sim_time before this version
  goal_point_.header.stamp = node_->now();
}

}  // namespace nav2_carrot_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(
  nav2_carrot_controller::CarrotController,
  nav2_core::Controller)
