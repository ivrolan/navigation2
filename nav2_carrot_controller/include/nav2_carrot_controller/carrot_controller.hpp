/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Intelligent Robotics Lab (URJC)
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

#ifndef NAV2_CARROT_CONTROLLER__NAV2_CARROT_CONTROLLER_HPP_
#define NAV2_CARROT_CONTROLLER__NAV2_CARROT_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "dwb_core/dwb_local_planner.hpp"
#include "nav_2d_msgs/msg/pose2_d_stamped.hpp"
#include "nav_2d_msgs/msg/twist2_d_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_carrot_controller
{

/**
 * @class CarrotController
 * @brief Plugin-based follor point controller
 */
class CarrotController : public dwb_core::DWBLocalPlanner
{
public:
  /**
   * @brief Constructor that brings up pluginlib loaders
   */
  CarrotController();

  virtual ~CarrotController() {}

  void configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> & tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros) override;

  /**
   * @brief nav2_core isGoalReached - Check whether the robot has reached its goal, given the current pose & velocity.
   *
   * The pose that it checks against is the last pose in the current global plan.
   * The calculation is delegated to the goal_checker plugin.
   *
   * @param pose Current pose
   * @param velocity Current velocity
   * @return True if the robot should be considered as having reached the goal.
   */
  bool isGoalReached(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity) override;

  /**
   * @brief nav2_core setPlan - Sets the global plan
   * @param path The global plan
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief nav2_core computeVelocityCommands - calculates the best command given the current pose and velocity
   *
   * It is presumed that the global plan is already set.
   *
   * This is mostly a wrapper for the protected computeVelocityCommands
   * function which has additional debugging info.
   *
   * @param pose Current robot pose
   * @param velocity Current robot velocity
   * @return The best command for the robot to drive
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity) override;

protected:
  double carrot_distance_;
  double goal_point_valid_duration_;
  double distance_to_goal_;

  /**
   * @brief A subscription and callback to handle the topic-based goal published
   */
  void onGoalPointReceived(const geometry_msgs::msg::PointStamped::SharedPtr point);
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr update_point_sub_;
  geometry_msgs::msg::PointStamped goal_point_;
};

}  // namespace dwb_core

#endif  // NAV2_CARROT_CONTROLLER__NAV2_CARROT_CONTROLLER_HPP_
