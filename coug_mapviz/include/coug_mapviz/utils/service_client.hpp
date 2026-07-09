// Copyright (c) 2026 BYU FROST Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file service_client.hpp
 * @brief MapViz plugin helper that owns the ROS 2 service clients and publishers.
 * @author Nelson Durrant
 * @date May 2026
 */

#pragma once

#include <swri_transform_util/transform_manager.h>

#include <coug_interfaces/msg/way_point_list.hpp>
#include <coug_mapviz/utils/waypoint_manager.hpp>
#include <functional>
#include <geometry_msgs/msg/pose_array.hpp>
#include <map>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <vector>

namespace coug_mapviz::utils {

/**
 * @class ServiceClient
 * @brief MapViz plugin helper that owns the ROS 2 service clients and publishers.
 */
class ServiceClient {
 public:
  /**
   * @enum Status
   * @brief Severity of a status message reported back through the StatusCallback.
   */
  enum class Status { kInfo, kWarning, kError };

  /**
   * @brief Callback used to report status messages to the UI.
   */
  using StatusCallback = std::function<void(Status, const std::string&)>;

  ServiceClient() = default;

  /**
   * @brief Creates the publishers and service clients for each agent namespace.
   * @param node The ROS node used to create publishers and clients.
   * @param tf_manager Transform manager used to project waypoints into the map frame.
   * @param agent_namespaces The agent namespaces to create interfaces for.
   * @param waypoint_topic Topic suffix for the WayPointList publisher.
   * @param waypoint_map_topic Topic suffix for the PoseArray (map frame) publisher.
   * @param services Maps each command name to its Trigger service suffix.
   * @param status_cb Callback invoked to report status messages.
   */
  void initialize(const std::shared_ptr<rclcpp::Node>& node,
                  const swri_transform_util::TransformManagerPtr& tf_manager,
                  const std::vector<std::string>& agent_namespaces,
                  const std::string& waypoint_topic, const std::string& waypoint_map_topic,
                  const std::map<std::string, std::string>& services, StatusCallback status_cb);

  /**
   * @brief Publishes one agent's waypoints as a WayPointList and a map-frame PoseArray.
   * @param agent The agent namespace.
   * @param wps The waypoints to publish.
   * @param target_frame The map frame to project the PoseArray into.
   */
  void publishWaypoints(const std::string& agent,
                        const std::vector<coug_interfaces::msg::WayPoint>& wps,
                        const std::string& target_frame);

  /**
   * @brief Calls a Trigger service on the given agents.
   * @param cmd The command name (e.g. "start", "stop").
   * @param agents The agent namespaces to call.
   * @param aggregate True to report one tallied summary, false to report each response.
   */
  void callService(const std::string& cmd, const std::vector<std::string>& agents, bool aggregate);

 private:
  /**
   * @struct CallState
   * @brief Shared tally for an aggregated multi-agent service call.
   */
  struct CallState {
    int total = 0;
    int responded = 0;
    int succeeded = 0;
    std::string cmd;
    std::vector<std::string> failed;
    std::mutex mutex;
  };

  /**
   * @brief Calls a Trigger service on one agent, tallying into state if provided.
   * @param ns The agent namespace to call.
   * @param cmd The command name.
   * @param state Shared tally for an aggregated call; nullptr for a single call.
   */
  void callAgentService(const std::string& ns, const std::string& cmd,
                        std::shared_ptr<CallState> state = nullptr);

  /**
   * @brief Records one agent's response and reports a summary once all have responded.
   * @param state The shared call tally.
   * @param success Whether the agent's call succeeded.
   * @param agent The agent namespace that responded.
   */
  void recordResult(std::shared_ptr<CallState> state, bool success, const std::string& agent);

  std::shared_ptr<rclcpp::Node> node_;
  swri_transform_util::TransformManagerPtr tf_manager_;
  StatusCallback status_;

  std::string waypoint_topic_;
  std::string waypoint_map_topic_;

  std::map<std::string, rclcpp::Publisher<coug_interfaces::msg::WayPointList>::SharedPtr>
      publishers_;
  std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr>
      map_publishers_;
  std::map<std::string, std::map<std::string, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr>>
      clients_;
};

}  // namespace coug_mapviz::utils
