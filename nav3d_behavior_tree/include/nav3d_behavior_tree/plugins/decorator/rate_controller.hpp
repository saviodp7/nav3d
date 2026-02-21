#pragma once

#include <chrono>
#include <string>

#include "behaviortree_cpp/decorator_node.h"

namespace nav3d_behavior_tree
{

class RateController : public BT::DecoratorNode
{
public:
  RateController(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  void initialize();

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("hz", 10.0, "Rate")
    };
  }

private:
  BT::NodeStatus tick() override;

  std::chrono::time_point<std::chrono::high_resolution_clock> start_;
  double period_;
  bool first_time_;
};

}  // namespace nav3d_behavior_tree