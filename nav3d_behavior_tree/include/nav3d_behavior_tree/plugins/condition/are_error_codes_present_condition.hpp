#pragma once

#include <memory>
#include <set>
#include <string>
#include <vector>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"

namespace nav3d_behavior_tree
{

class AreErrorCodesPresent : public BT::ConditionNode
{
public:
  AreErrorCodesPresent(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf)
  {
    std::vector<int> error_codes_to_check_vector;
    getInput("error_codes_to_check", error_codes_to_check_vector);

    error_codes_to_check_ = std::set<uint16_t>(
      error_codes_to_check_vector.begin(),
      error_codes_to_check_vector.end());
  }

  AreErrorCodesPresent() = delete;

  BT::NodeStatus tick() override
  {
    getInput<uint16_t>("error_code", error_code_);

    if (error_codes_to_check_.find(error_code_) != error_codes_to_check_.end()) {
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<uint16_t>("error_code", "The active error codes"),
      BT::InputPort<std::vector<int>>("error_codes_to_check", "Error codes to check")
    };
  }

protected:
  uint16_t error_code_;
  std::set<uint16_t> error_codes_to_check_;
};

}  // namespace nav3d_behavior_tree