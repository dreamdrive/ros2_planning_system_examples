// Copyright 2020 Intelligent Robotics Lab
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

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
// #include "nav2_msgs/action/navigate_to_pose.hpp"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "control_msgs/action/follow_joint_trajectory.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class Mov2Sim : public rclcpp::Node
{
public:
  using MoveToPose = control_msgs::action::FollowJointTrajectory;
  using GoalHandleMoveToPose = rclcpp_action::ServerGoalHandle<MoveToPose>;

  Mov2Sim()
  : Node("move_to_pose_server")
  {
  }

  void start_server()
  {
    using namespace std::placeholders;

    repeat_sentence_action_server_ = rclcpp_action::create_server<MoveToPose>(
      shared_from_this(),
      "move_to_pose",
      std::bind(&Mov2Sim::handle_goal, this, _1, _2),
      std::bind(&Mov2Sim::handle_cancel, this, _1),
      std::bind(&Mov2Sim::handle_accepted, this, _1));

    RCLCPP_INFO(get_logger(), "Ready.");
  }

private:
  rclcpp_action::Server<MoveToPose>::SharedPtr repeat_sentence_action_server_;
  MoveToPose::Goal current_goal_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveToPose::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Callback for goal");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void execute(const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
  {
    rclcpp::Rate loop_rate(1);
    auto feedback = std::make_shared<MoveToPose::Feedback>();
    auto result = std::make_shared<MoveToPose::Result>();

    auto joint_cmd = goal_handle->get_goal()->trajectory.points;

    RCLCPP_INFO(
      this->get_logger(), "Starting move to %lf, %lf, %lf, %lf, %lf, %lf, %lf",
      joint_cmd[0].positions[0], joint_cmd[0].positions[1], joint_cmd[0].positions[2],
      joint_cmd[0].positions[3], joint_cmd[0].positions[4], joint_cmd[0].positions[5], joint_cmd[0].positions[6]);

    auto start = now();
    int current_times = 0;
    while (rclcpp::ok() && current_times++ < 10) {
      RCLCPP_INFO(this->get_logger(), "Moving %d ", current_times);

      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);

        RCLCPP_INFO(this->get_logger(), "Action Canceled");

        return;
      }

      loop_rate.sleep();
    }

    if (rclcpp::ok()) {
      goal_handle->succeed(result);

      RCLCPP_INFO(this->get_logger(), "Move Succeeded");
    }
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Callback for accepted");
    using namespace std::placeholders;
    std::thread{std::bind(&Mov2Sim::execute, this, _1), goal_handle}.detach();
  }
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Mov2Sim>();

  node->start_server();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
