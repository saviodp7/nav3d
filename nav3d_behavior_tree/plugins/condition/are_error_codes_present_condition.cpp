#include "nav3d_behavior_tree/plugins/condition/are_error_codes_present_condition.hpp"

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav3d_behavior_tree::AreErrorCodesPresent>(
    "AreErrorCodesPresent");
}