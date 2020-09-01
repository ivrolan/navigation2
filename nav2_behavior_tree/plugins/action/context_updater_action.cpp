// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Francisco Martin Rico
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

#include <string>
#include <memory>
#include <limits>

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "behaviortree_cpp_v3/decorator_node.h"

#include "nav2_behavior_tree/plugins/action/context_updater_action.hpp"

namespace nav2_behavior_tree
{

ContextUpdater::ContextUpdater(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
}

inline BT::NodeStatus ContextUpdater::tick()
{
  setStatus(BT::NodeStatus::RUNNING);
  auto current_context = config().blackboard->get<std::string>("current_context");

  if (current_context == "normal") {
    setOutput("controller", "FollowPath");
  } else {
    setOutput("controller", "DynamicFollowPath");
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::ContextUpdater>("ContextUpdater");
}
