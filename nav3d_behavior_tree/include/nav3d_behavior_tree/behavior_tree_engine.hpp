#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "rclcpp/rclcpp.hpp"

namespace nav3d_behavior_tree
{

enum class BtStatus { SUCCEEDED, FAILED, CANCELED };

class BehaviorTreeEngine
{
public:
  explicit BehaviorTreeEngine(
    const std::vector<std::string> & plugin_libraries,
    rclcpp::Node::SharedPtr node);

  virtual ~BehaviorTreeEngine() = default;

  BtStatus run(
    BT::Tree * tree,
    std::function<void()> onLoop,
    std::function<bool()> cancelRequested,
    std::chrono::milliseconds loopTimeout = std::chrono::milliseconds(10));

  BT::Tree createTreeFromText(
    const std::string & xml_string,
    BT::Blackboard::Ptr blackboard);

  BT::Tree createTreeFromFile(
    const std::string & file_path,
    BT::Blackboard::Ptr blackboard);

  void addGrootMonitoring(BT::Tree * tree, uint16_t server_port);

  void resetGrootMonitor();

  void haltAllActions(BT::Tree & tree);

protected:
  BT::BehaviorTreeFactory factory_;
  rclcpp::Clock::SharedPtr clock_;
  std::unique_ptr<BT::Groot2Publisher> groot_monitor_;
};

}  // namespace nav3d_behavior_tree