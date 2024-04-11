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

#include <plansys2_pddl_parser/Utils.h>

#include <memory>

#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class Motion : public rclcpp::Node
{
public:
  Motion()
  : rclcpp::Node("motion_controller")
  {
  }

  bool init()
  {
    std::cout << ">>>> 1.1 <<<<" << std::endl;

    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();

    std::cout << ">>>> 1.2 <<<<" << std::endl;
    init_knowledge();
    std::cout << ">>>> 1.3 <<<<" << std::endl;

    auto domain = domain_expert_->getDomain();
    auto problem = problem_expert_->getProblem();
    auto plan = planner_client_->getPlan(domain, problem);

    if (!plan.has_value()) {
      std::cout << "Could not find plan to reach goal " <<
        parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
      return false;
    }

    if (!executor_client_->start_plan_execution(plan.value())) {
      RCLCPP_ERROR(get_logger(), "Error starting a new plan (first)");
    }

    return true;
  }

  void init_knowledge()
  {
    problem_expert_->addInstance(plansys2::Instance{"nyokkey", "robot"});

    problem_expert_->addInstance(plansys2::Instance{"ini", "pose"});
    problem_expert_->addInstance(plansys2::Instance{"fin", "pose"});
    problem_expert_->addInstance(plansys2::Instance{"p1",  "pose"});
    problem_expert_->addInstance(plansys2::Instance{"p2",  "pose"});
    problem_expert_->addInstance(plansys2::Instance{"p3",  "pose"});
    problem_expert_->addInstance(plansys2::Instance{"p4",  "pose"});

    problem_expert_->addInstance(plansys2::Instance{"zero", "pose"});
    problem_expert_->addInstance(plansys2::Instance{"jt1", "pose"});

    problem_expert_->addPredicate(plansys2::Predicate("(connected ini p1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected p1 p2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected p2 p3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected p3 p4)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected p4 fin)"));

    problem_expert_->addPredicate(plansys2::Predicate("(robot_at nyokkey ini)"));

    // problem_expert_->addPredicate(plansys2::Predicate("(zero_point_at zero)"));
    // problem_expert_->addPredicate(plansys2::Predicate("(joint_ar jt1)"));

    problem_expert_->setGoal(plansys2::Goal(
        "(and(robot_at nyokkey p1))"
        // "(and(robot_ar nyokkey zero)"
        // "(and(robot_ar nyokkey jt1)"
        ));
  }

  void step()
  {
    if (!executor_client_->execute_and_check_plan()) {  // Plan finished
      auto result = executor_client_->getResult();

      if (result.value().success) {
        RCLCPP_INFO(get_logger(), "Plan succesfully finished");
      } else {
        RCLCPP_ERROR(get_logger(), "Plan finished with error");
      }
    }
  }

private:
  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Motion>();

  std::cout << ">>>> 1 <<<<" << std::endl;

  if (!node->init()) {
    std::cout << ">>>> 2 <<<<" << std::endl;
    RCLCPP_INFO(node->get_logger(), "init FAILED!!");
    return 0;
  }
  RCLCPP_INFO(node->get_logger(), "init done");

  std::cout << ">>>> 3 <<<<" << std::endl;

  rclcpp::Rate rate(5);
  while (rclcpp::ok()) {
    node->step();

    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}
