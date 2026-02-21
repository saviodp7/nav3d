#pragma once

#include <map>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "bondcpp/bond.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "nav3d_msgs/srv/manage_lifecycle_nodes.hpp"
#include "nav3d_util/lifecycle_service_client.hpp"
#include "nav3d_util/node_thread.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace nav3d_lifecycle_manager
{

using namespace std::chrono_literals;  // NOLINT
using nav3d_msgs::srv::ManageLifecycleNodes;

class LifecycleManager : public rclcpp::Node
{
public:
  explicit LifecycleManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~LifecycleManager();

protected:
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::unique_ptr<nav3d_util::NodeThread> service_thread_;

  rclcpp::Service<ManageLifecycleNodes>::SharedPtr manager_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr is_active_srv_;

  void managerCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ManageLifecycleNodes::Request> request,
    std::shared_ptr<ManageLifecycleNodes::Response> response);

  void isActiveCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  bool startup();
  bool shutdown();
  bool reset(bool hard_reset = false);
  bool pause();
  bool resume();

  void onRclPreshutdown();
  void createLifecycleServiceClients();
  void shutdownAllNodes();
  void destroyLifecycleServiceClients();

  void createBondTimer();
  bool createBondConnection(const std::string & node_name);
  void destroyBondTimer();
  void checkBondConnections();
  void checkBondRespawnConnection();

  bool changeStateForNode(
    const std::string & node_name,
    std::uint8_t transition);

  bool changeStateForAllNodes(std::uint8_t transition, bool hard_change = false);

  void message(const std::string & msg);

  void CreateActiveDiagnostic(diagnostic_updater::DiagnosticStatusWrapper & stat);

  void registerRclPreshutdownCallback();

  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr bond_timer_;
  rclcpp::TimerBase::SharedPtr bond_respawn_timer_;
  std::chrono::milliseconds bond_timeout_;

  std::map<std::string, std::shared_ptr<bond::Bond>> bond_map_;
  std::map<std::string, std::shared_ptr<nav3d_util::LifecycleServiceClient>> node_map_;

  std::map<std::uint8_t, std::string> transition_label_map_;
  std::unordered_map<std::uint8_t, std::uint8_t> transition_state_map_;

  std::vector<std::string> node_names_;

  bool autostart_;
  bool attempt_respawn_reconnection_;

  bool system_active_{false};
  diagnostic_updater::Updater diagnostics_updater_;

  rclcpp::Time bond_respawn_start_time_{0};
  rclcpp::Duration bond_respawn_max_duration_{10s};
};

}  // namespace nav3d_lifecycle_manager