// Copyright 2019 Intelligent Robotics Lab
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
#include <iostream>
#include <vector>
#include <memory>

#include "my_bt_example/behavior_tree_nodes/Move.hpp"

// #include "geometry_msgs/msg/pose2_d.hpp"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "control_msgs/action/follow_joint_trajectory.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace plansys2_bt_tests
{

Move::Move(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: plansys2::BtActionNode<control_msgs::action::FollowJointTrajectory>(xml_tag_name, action_name, conf)
{
  std::cout << "Move [1]" << std::endl;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  config().blackboard->get("node", node);

  std::cout << "Move [2]" << std::endl;
//   node->declare_parameter("joint_names");
//   node->declare_parameter("waypoints");
//   node->declare_parameter("waypoint_joints");

  try {
    node->declare_parameter<std::vector<std::string>>("waypoints");
    node->declare_parameter<std::vector<std::string>>("joint_names");
    // node->declare_parameter<std::vector<std::string>>("joint_names");
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & e) {
    // Do nothing;
  }

  std::cout << "Move [3]" << std::endl;
  if (node->has_parameter("joint_names")) {
    node->get_parameter_or("joint_names", joint_names, {});
  }

  std::cout << "Move [4]" << std::endl;
  if (node->has_parameter("waypoints")) {
    std::vector<std::string> wp_names;

    node->get_parameter_or("waypoints", wp_names, {});

    for (auto & wp : wp_names) {
    //   node->declare_parameter("waypoint_joints." + wp);
      try {
        node->declare_parameter<std::vector<double>>("waypoint_joints." + wp);
      } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & e) {
        // Do nothing;
      }

      std::vector<double> joints;
      if (node->get_parameter_or("waypoint_joints." + wp, joints, {})) {
        trajectory_msgs::msg::JointTrajectoryPoint point;

        point.time_from_start = rclcpp::Duration::from_seconds(1.0);  // start asap
        point.positions.resize(joint_names.size());

        for (int i=0; i<joint_names.size(); i++) {
          point.positions[i] = joints[i];
        }

        waypoints_[wp] = point;
      } else {
        std::cerr << "No joint configuration configured for waypoint [" << wp << "]" << std::endl;
      }
    }
  }
  std::cout << "Move [5]" << std::endl;
}

BT::NodeStatus
Move::on_tick()
{
  std::cout << "Move [6]" << std::endl;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  config().blackboard->get("node", node);

  std::cout << "Move [7]" << std::endl;
  std::string goal;
  getInput<std::string>("goal", goal);

  std::cout << "Move [8]" << std::endl;
  trajectory_msgs::msg::JointTrajectoryPoint joint2mov;
//   geometry_msgs::msg::Pose2D pose2nav;
  if (waypoints_.find(goal) != waypoints_.end()) {
    joint2mov = waypoints_[goal];
  } else {
    std::cerr << "No joint configuration for waypoint [" << goal << "]" << std::endl;
  }

  std::cout << "Move [9]" << std::endl;
  trajectory_msgs::msg::JointTrajectory traj;
//   geometry_msgs::msg::PoseStamped goal_pos;

//   traj.joint_names = joint_names;
  traj.points.resize(1);
  traj.points[0] = joint2mov;

  std::cout << "Move [10]" << std::endl;
  goal_.trajectory = traj;

  return BT::NodeStatus::RUNNING;

}

BT::NodeStatus
Move::on_success()
{
  return BT::NodeStatus::SUCCESS;
}


}  // namespace plansys2_bt_tests

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<plansys2_bt_tests::Move>(
        name, "move_to_pose", config);
    };

  factory.registerBuilder<plansys2_bt_tests::Move>(
    "Move", builder);
}
