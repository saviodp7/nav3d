#include <memory>
#include <string>

#include "nav3d_behavior_tree/plugins/action/controller_cancel_node.hpp"

namespace nav3d_behavior_tree
{

ControllerCancel::ControllerCancel(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtCancelActionNode<nav3d_msgs::action::FollowPath>(xml_tag_name, action_name, conf)
{
}

}  // namespace nav3d_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav3d_behavior_tree::ControllerCancel>(
        name, "follow_path", config);
    };

  factory.registerBuilder<nav3d_behavior_tree::ControllerCancel>(
    "CancelControl", builder);
}