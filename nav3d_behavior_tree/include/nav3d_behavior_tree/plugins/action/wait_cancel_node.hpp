#pragma once

#include <memory>
#include <string>

#include "nav3d_behavior_tree/bt_cancel_action_node.hpp"
#include "nav3d_msgs/action/wait.hpp"

namespace nav3d_behavior_tree
{

class WaitCancel : public BtCancelActionNode<nav3d_msgs::action::Wait>
{
public:
  WaitCancel(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }
};

}  // namespace nav3d_behavior_tree