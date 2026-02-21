#pragma once

#include <string>

#include "behaviortree_cpp/control_node.h"

namespace nav3d_behavior_tree
{

class RecoveryNode : public BT::ControlNode
{
public:
  RecoveryNode(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  ~RecoveryNode() override = default;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("number_of_retries", 1, "Number of retries")
    };
  }

private:
  unsigned int current_child_idx_;
  unsigned int number_of_retries_;
  unsigned int retry_count_;

  BT::NodeStatus tick() override;
  void halt() override;
};

}  // namespace nav3d_behavior_tree