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

// #include "tf2/LinearMath/Quaternion.h"
//#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit_msgs/action/execute_trajectory.hpp"

using namespace std::chrono_literals;


bool common_goal_accepted = false;
rclcpp_action::ResultCode common_resultcode = rclcpp_action::ResultCode::UNKNOWN;
std::shared_ptr<moveit_msgs::action::MoveGroup_Result> common_action_result_code;

void common_goal_response(
    // std::shared_future<rclcpp_action::ClientGoalHandle
    // <moveit_msgs::action::MoveGroup>::SharedPtr> future)
    const rclcpp_action::ClientGoalHandle
    <moveit_msgs::action::MoveGroup>::SharedPtr & goal_handle)
{
    printf("common_goal_response time: %f\n", rclcpp::Clock().now().seconds());
    // auto goal_handle = future.get();
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
    <moveit_msgs::action::MoveGroup>::WrappedResult & result)
{
    printf("common_result_response time: %f\n", rclcpp::Clock().now().seconds());
    common_resultcode = result.code;
    common_action_result_code = result.result; // ->error_code;
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
    rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::SharedPtr,
    const std::shared_ptr<const moveit_msgs::action::MoveGroup::Feedback> feedback)
{
    // trajectory_msgs/JointTrajectoryPoint desired
    // trajectory_msgs/JointTrajectoryPoint actual
    // trajectory_msgs/JointTrajectoryPoint error
    // current_position = feedback->actual;
    // std::cout << "feedback->desired.positions :";
    // for (auto & x : feedback->desired.positions) {
    //     std::cout << x << "\t";
    // }
    // std::cout << std::endl;
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
        : plansys2::ActionExecutorClient("zero_move", 250ms)
    {
        joint_names = {
            "left_joint1", "left_joint2", "left_joint3", 
            "left_joint4", "left_joint5", "left_joint6", 
            "left_joint7"
        };

        //moveit_msgs::action::MoveGroup::Goal goal;
        
        goal.request.group_name = "left_arm";
        goal.request.num_planning_attempts = 1;
        goal.request.allowed_planning_time = 5.0;
        goal.request.max_velocity_scaling_factor = 1.0;
        goal.request.max_acceleration_scaling_factor = 1.0;
        goal.request.planner_id = "RRTConnectkConfigDefault";
        goal.request.workspace_parameters.min_corner.x = -2.0;
        goal.request.workspace_parameters.min_corner.y = -2.0;
        goal.request.workspace_parameters.min_corner.z = -2.0;
        goal.request.workspace_parameters.max_corner.x = 2.0;
        goal.request.workspace_parameters.max_corner.y = 2.0;
        goal.request.workspace_parameters.max_corner.z = 2.0;
        //goal.request.start_state.is_diff = true;
        goal.request.goal_constraints.resize(1);

        // JOINT CONSTRAINTS
        goal.request.goal_constraints[0].joint_constraints.resize(joint_names.size());
        for (int i=0; i<joint_names.size(); i++) {
            goal.request.goal_constraints[0].joint_constraints[i].joint_name = joint_names[i];
            // goal.request.goal_constraints[0].joint_constraints[i].position = 0.0;
            goal.request.goal_constraints[0].joint_constraints[i].tolerance_above = 0.01;
            goal.request.goal_constraints[0].joint_constraints[i].tolerance_below = 0.01;
            goal.request.goal_constraints[0].joint_constraints[i].weight = 1.0;
        }

        // goal.request.goal_constraints[0].joint_constraints[0].position = -0.234;
        // goal.request.goal_constraints[0].joint_constraints[1].position = -1.065;
        // goal.request.goal_constraints[0].joint_constraints[2].position = -0.486;
        // goal.request.goal_constraints[0].joint_constraints[3].position = 0.524;
        // // goal.request.goal_constraints[0].joint_constraints[4].position = 0.0;
        // goal.request.goal_constraints[0].joint_constraints[4].position = 0.0825;
        // goal.request.goal_constraints[0].joint_constraints[5].position = -0.423;
        // goal.request.goal_constraints[0].joint_constraints[6].position = 0.323;

        goal.request.goal_constraints[0].joint_constraints[0].position = 0.0;
        goal.request.goal_constraints[0].joint_constraints[1].position = 0.0;
        goal.request.goal_constraints[0].joint_constraints[2].position = 0.0;
        goal.request.goal_constraints[0].joint_constraints[3].position = 0.0;
        // goal.request.goal_constraints[0].joint_constraints[4].position = 0.0;
        goal.request.goal_constraints[0].joint_constraints[4].position = 0.0;
        goal.request.goal_constraints[0].joint_constraints[5].position = 0.0;
        goal.request.goal_constraints[0].joint_constraints[6].position = 0.0;

        goal.planning_options.plan_only = false;
        goal.planning_options.replan = false;

    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State & previous_state)
    {
        send_feedback(0.0, "Zero Move starting");

        move_action_client = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(
            get_node_base_interface(),
            get_node_graph_interface(),
            get_node_logging_interface(),
            get_node_waitables_interface(),
            "/move_action");

        bool is_action_server_ready = false;
        do {
            RCLCPP_INFO(get_logger(), "Waiting for move action server...");

            is_action_server_ready = 
                move_action_client->wait_for_action_server(std::chrono::seconds(5));
        } while (!is_action_server_ready);

        RCLCPP_INFO(get_logger(), "Move action server ready");

        auto point_to_move = get_arguments()[2];  // The goal is in the 3rd argument of the action
        RCLCPP_INFO(get_logger(), "Start moving to [%s]", point_to_move.c_str());

        //dist_to_move = getDistance(goal_msg.trajectory.points[0], current_position);

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

        future_move_goal_handle = move_action_client->async_send_goal(goal, opt);

        return ActionExecutorClient::on_activate(previous_state);
    }

private:
    // double getDistance(const trajectory_msgs::msg::JointTrajectoryPoint & po1, 
    //                    const trajectory_msgs::msg::JointTrajectoryPoint & po2)
    // {
    //     return 0.0;
    // }

    void do_work()
    {
    }

    std::vector<std::string> joint_names;

    rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SendGoalOptions opt;
    moveit_msgs::action::MoveGroup::Goal goal;

    rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SharedPtr move_action_client;
    std::shared_future<rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::SharedPtr> future_move_goal_handle;

    double dist_to_move;

    trajectory_msgs::msg::JointTrajectoryPoint current_position;

};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveAction>();

    node->set_parameter(rclcpp::Parameter("action_name", "zero_move"));
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

    rclcpp::spin(node->get_node_base_interface());

    rclcpp::shutdown();
    return 0;
}
