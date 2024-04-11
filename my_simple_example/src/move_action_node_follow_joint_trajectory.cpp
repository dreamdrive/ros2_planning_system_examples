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

#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// #include "moveit/move_group_interface/move_group_interface.h"
// #include "tf2/LinearMath/Quaternion.h"
#include "control_msgs/action/follow_joint_trajectory.hpp"

using namespace std::chrono_literals;


bool common_goal_accepted = false;
rclcpp_action::ResultCode common_resultcode = rclcpp_action::ResultCode::UNKNOWN;
int common_action_result_code = control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL;

void common_goal_response(
    // std::shared_future<rclcpp_action::ClientGoalHandle
    // <control_msgs::action::FollowJointTrajectory>::SharedPtr> future) // foxy
    const rclcpp_action::ClientGoalHandle
    <control_msgs::action::FollowJointTrajectory>::SharedPtr & goal_handle)
{
    printf("common_goal_response time: %f\n", rclcpp::Clock().now().seconds());
    // auto goal_handle = future.get(); // foxy
    if (!goal_handle) {
        common_goal_accepted = false;
        printf("Goal rejected\n");
    } else {
        common_goal_accepted = true;
        printf("Goal accepted\n");
    }
}
void common_result_response(
    const rclcpp_action::ClientGoalHandle
    <control_msgs::action::FollowJointTrajectory>::WrappedResult & result)
{
    printf("common_result_response time: %f\n", rclcpp::Clock().now().seconds());
    common_resultcode = result.code;
    common_action_result_code = result.result->error_code;
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            printf("SUCCEEDED result code");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            printf("Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            printf("Goal was canceled");
            return;
        default:
            printf("Unknown result code");
            return;
    }
}
void common_feedback(
    rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr,
    const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> feedback)
{
    // trajectory_msgs/JointTrajectoryPoint desired
    // trajectory_msgs/JointTrajectoryPoint actual
    // trajectory_msgs/JointTrajectoryPoint error
    // current_position = feedback->actual;
    std::cout << "feedback->desired.positions :";
    for (auto & x : feedback->desired.positions) {
        std::cout << x << "\t";
    }
    std::cout << std::endl;
    // std::cout << "feedback->desired.velocities :";
    // for (auto & x : feedback->desired.velocities) {
    //     std::cout << x << "\t";
    // }
    // std::cout << std::endl;
}

// moveit::planning_interface::MoveGroupInterfaceを通してmove_groupを操作するplansys2のaction node
class MoveAction : public plansys2::ActionExecutorClient 
{
public:
    MoveAction()
        : plansys2::ActionExecutorClient("move", 250ms)
    {
        joint_names = {
            "panda_joint1", "panda_joint2", "panda_joint3", 
            "panda_joint4", "panda_joint5", "panda_joint6", 
            "panda_joint7"
        };

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.time_from_start = rclcpp::Duration::from_seconds(1.0);  // start asap
        point.positions.resize(joint_names.size());

        point.positions[0] = 0.0; // 0.72;
        point.positions[1] = -0.785; //-1.73;
        point.positions[2] = 0.0; //-1.37;
        point.positions[3] = -2.356; //-2.74;
        point.positions[4] = 0.0; //1.68;
        point.positions[5] = 1.571; //1.79;
        point.positions[6] = 0.785; //-0.32;

        trajectory_msgs::msg::JointTrajectoryPoint point2;
        point2.time_from_start = rclcpp::Duration::from_seconds(2.0);
        point2.positions.resize(joint_names.size());

        point2.positions[0] = 0.72;
        point2.positions[1] = -1.73;
        point2.positions[2] = -1.37;
        point2.positions[3] = -2.74;
        point2.positions[4] = 1.68;
        point2.positions[5] = 1.79;
        point2.positions[6] = -0.32;

        goal_points.push_back(point);
        goal_points.push_back(point2);

    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State & previous_state)
    {
        send_feedback(0.0, "Move starting");

        move_action_client = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
            get_node_base_interface(),
            get_node_graph_interface(),
            get_node_logging_interface(),
            get_node_waitables_interface(),
            "/panda_arm_controller/follow_joint_trajectory");

        bool is_action_server_ready = false;
        do {
            RCLCPP_INFO(get_logger(), "Waiting for navigation action server...");

            is_action_server_ready = 
                move_action_client->wait_for_action_server(std::chrono::seconds(5));
        } while (!is_action_server_ready);

        RCLCPP_INFO(get_logger(), "Move action server ready");

        auto point_to_move = get_arguments()[2];  // The goal is in the 3rd argument of the action
        RCLCPP_INFO(get_logger(), "Start moving to [%s]", point_to_move.c_str());

        dist_to_move = getDistance(goal_msg.trajectory.points[0], current_position);

        opt.goal_response_callback = std::bind(common_goal_response, std::placeholders::_1);
        // opt.result_callback        = std::bind(common_result_response, std::placeholders::_1);
        opt.result_callback = [this](auto) 
            {
                finish(true, 1.0, "Move completed");
            };
        opt.feedback_callback      = std::bind(common_feedback, std::placeholders::_1, std::placeholders::_2);
        // opt.feedback_callback = [this](
        //     rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr,
        //     const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> feedback)
        //     NavigationGoalHandle::SharedPtr,
        //     NavigationFeedback feedback) 
        //     {
        //         send_feedback(
        //             std::min(1.0, std::max(0.0, 1.0 - (feedback->distance_remaining / dist_to_move))),
        //             "Move running");
        //     };

        goal_msg.goal_time_tolerance = rclcpp::Duration::from_seconds(1.0);
        goal_msg.trajectory.joint_names = joint_names;
        goal_msg.trajectory.points = goal_points;

        future_move_goal_handle = move_action_client->async_send_goal(goal_msg, opt);

        return ActionExecutorClient::on_activate(previous_state);
    }

private:
    double getDistance(const trajectory_msgs::msg::JointTrajectoryPoint & po1, 
                       const trajectory_msgs::msg::JointTrajectoryPoint & po2)
    {
        return 0.0;
    }

    void do_work()
    {
    }

    std::vector<std::string> joint_names;
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> goal_points;

    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions opt;
    control_msgs::action::FollowJointTrajectory_Goal goal_msg;

    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr move_action_client;
    std::shared_future<rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr> future_move_goal_handle;

    double dist_to_move;

    trajectory_msgs::msg::JointTrajectoryPoint current_position;

};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveAction>();

    node->set_parameter(rclcpp::Parameter("action_name", "move"));
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

    rclcpp::spin(node->get_node_base_interface());

    rclcpp::shutdown();
    return 0;
}
