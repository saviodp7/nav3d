#pragma once

#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/control_node.h"

namespace nav3d_behavior_tree
{

class RoundRobinNode : public BT::ControlNode
{
public:
  explicit RoundRobinNode(const std::string & name);

  RoundRobinNode(const std::string & name, const BT::NodeConfiguration & config);

  BT::NodeStatus tick() override;

  void halt() override;

  static BT::PortsList providedPorts()
  {
    return {};
  }

private:
  unsigned int current_child_idx_{0};
  unsigned int num_failed_children_{0};
};

}  // namespace nav3d_behavior_tree