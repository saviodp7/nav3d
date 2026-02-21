#pragma once

#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/control_node.h"

namespace nav3d_behavior_tree
{

class PipelineSequence : public BT::ControlNode
{
public:
  explicit PipelineSequence(const std::string & name);

  PipelineSequence(const std::string & name, const BT::NodeConfiguration & config);

  void halt() override;

  static BT::PortsList providedPorts()
  {
    return {};
  }

protected:
  BT::NodeStatus tick() override;

  std::size_t last_child_ticked_ = 0;
};

}  // namespace nav3d_behavior_tree