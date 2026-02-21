#include <chrono>
#include <string>

#include "nav3d_behavior_tree/plugins/decorator/rate_controller.hpp"

namespace nav3d_behavior_tree
{

RateController::RateController(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf),
  first_time_(false)
{
}

void RateController::initialize()
{
  double hz = 1.0;
  getInput("hz", hz);
  period_ = 1.0 / hz;
}

BT::NodeStatus RateController::tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }

  if (!BT::isStatusActive(status())) {
    start_ = std::chrono::high_resolution_clock::now();
    first_time_ = true;
  }

  setStatus(BT::NodeStatus::RUNNING);

  auto now = std::chrono::high_resolution_clock::now();
  auto elapsed = now - start_;

  using float_seconds = std::chrono::duration<float>;
  auto seconds = std::chrono::duration_cast<float_seconds>(elapsed);

  if (first_time_ || (child_node_->status() == BT::NodeStatus::RUNNING) ||
    seconds.count() >= period_)
  {
    first_time_ = false;
    const BT::NodeStatus child_state = child_node_->executeTick();

    switch (child_state) {
      case BT::NodeStatus::SKIPPED:
      case BT::NodeStatus::RUNNING:
      case BT::NodeStatus::FAILURE:
        return child_state;

      case BT::NodeStatus::SUCCESS:
        start_ = std::chrono::high_resolution_clock::now();
        return BT::NodeStatus::SUCCESS;

      default:
        return BT::NodeStatus::FAILURE;
    }
  }

  return status();
}

}

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav3d_behavior_tree::RateController>("RateController");
}