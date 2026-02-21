#pragma once

#include <chrono>
#include <string>

#include "behaviortree_cpp/decorator_node.h"

namespace nav3d_behavior_tree
{

class SingleTrigger : public BT::DecoratorNode
{
public:
  SingleTrigger(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {};
  }

private:
  BT::NodeStatus tick() override;

  bool first_time_;
};

}  // namespace nav3d_behavior_tree