#pragma once

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "nav_msgs/msg/goals.hpp"

namespace nav3d_behavior_tree
{

class GetNextFewGoals : public BT::ActionNodeBase
{
public:
  GetNextFewGoals(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Goals>("input_goals", "Input goals for navigation"),
      BT::InputPort<int>("num_goals", "Number of goals to extract"),
      BT::OutputPort<nav_msgs::msg::Goals>("output_goals", "Output goals for navigation")
    };
  }

private:
  void halt() override {}
  BT::NodeStatus tick() override;
};

}  // namespace nav3d_behavior_tree