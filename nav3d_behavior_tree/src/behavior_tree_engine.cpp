#include "nav3d_behavior_tree/behavior_tree_engine.hpp"

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/json_export.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "nav3d_behavior_tree/json_utils.hpp"
#include "nav3d_behavior_tree/utils/loop_rate.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav3d_behavior_tree
{

BehaviorTreeEngine::BehaviorTreeEngine(
  const std::vector<std::string> & plugin_libraries, rclcpp::Node::SharedPtr node)
{
  BT::SharedLibrary loader;
  for (const auto & p : plugin_libraries) {
    factory_.registerFromPlugin(loader.getOSName(p));
  }

  clock_ = node->get_clock();

  BT::ReactiveSequence::EnableException(false);
  BT::ReactiveFallback::EnableException(false);
}

BtStatus BehaviorTreeEngine::run(
  BT::Tree * tree,
  std::function<void()> onLoop,
  std::function<bool()> cancelRequested,
  std::chrono::milliseconds loopTimeout)
{
  nav3d_behavior_tree::LoopRate loopRate(loopTimeout, tree);
  BT::NodeStatus result = BT::NodeStatus::RUNNING;

  try {
    while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
      if (cancelRequested()) {
        tree->haltTree();
        return BtStatus::CANCELED;
      }

      result = tree->tickOnce();

      onLoop();

      if (!loopRate.sleep()) {
        RCLCPP_DEBUG_THROTTLE(
          rclcpp::get_logger("BehaviorTreeEngine"),
          *clock_, 1000,
          "Behavior Tree tick rate %0.2f was exceeded!",
          1.0 / (loopRate.period().count() * 1.0e-9));
      }
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      rclcpp::get_logger("BehaviorTreeEngine"),
      "Behavior tree threw exception: %s. Exiting with failure.", ex.what());
    return BtStatus::FAILED;
  }

  return (result == BT::NodeStatus::SUCCESS) ? BtStatus::SUCCEEDED : BtStatus::FAILED;
}

BT::Tree BehaviorTreeEngine::createTreeFromText(
  const std::string & xml_string,
  BT::Blackboard::Ptr blackboard)
{
  return factory_.createTreeFromText(xml_string, blackboard);
}

BT::Tree BehaviorTreeEngine::createTreeFromFile(
  const std::string & file_path,
  BT::Blackboard::Ptr blackboard)
{
  return factory_.createTreeFromFile(file_path, blackboard);
}

void BehaviorTreeEngine::addGrootMonitoring(
  BT::Tree * tree,
  uint16_t server_port)
{
  groot_monitor_ = std::make_unique<BT::Groot2Publisher>(*tree, server_port);

  BT::RegisterJsonDefinition<builtin_interfaces::msg::Time>();
  BT::RegisterJsonDefinition<std_msgs::msg::Header>();
}

void BehaviorTreeEngine::resetGrootMonitor()
{
  if (groot_monitor_) {
    groot_monitor_.reset();
  }
}

void BehaviorTreeEngine::haltAllActions(BT::Tree & tree)
{
  tree.haltTree();
}

}  // namespace nav3d_behavior_tree