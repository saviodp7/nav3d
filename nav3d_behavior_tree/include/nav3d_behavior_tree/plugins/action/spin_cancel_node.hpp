#pragma once

#include "nav3d_behavior_tree/bt_cancel_action_node.hpp"
#include "nav3d_msgs/action/spin.hpp"

#include <memory>
#include <string>

namespace nav3d_behavior_tree {

class SpinCancel : public BtCancelActionNode<nav3d_msgs::action::Spin> {
  public:
    SpinCancel(const std::string& xml_tag_name, const std::string& action_name, const BT::NodeConfiguration& conf);

    static BT::PortsList providedPorts() { return providedBasicPorts({}); }
};

} // namespace nav3d_behavior_tree