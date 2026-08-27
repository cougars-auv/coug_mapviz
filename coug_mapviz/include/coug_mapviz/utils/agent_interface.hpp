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

#pragma once

#include <swri_transform_util/transform_manager.h>

#include <array>
#include <coug_interfaces/msg/way_point.hpp>
#include <coug_interfaces/msg/way_point_list.hpp>
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

class AgentInterface {
 public:
  enum class Status { kInfo, kWarning, kError };
  enum class Command { kStart, kStop, kSurface, kHome };

  using StatusCallback = std::function<void(Status, const std::string&)>;

  struct Config {
    std::string waypoint_topic;
    std::string waypoint_map_topic;
    std::array<std::string, 4> service_names;
  };

  AgentInterface() = default;

  void initialize(const std::shared_ptr<rclcpp::Node>& node,
                  const swri_transform_util::TransformManagerPtr& tf_manager,
                  const std::vector<std::string>& agent_namespaces, const Config& config,
                  StatusCallback status_callback);

  void publishWaypoints(const std::string& agent_name,
                        const std::vector<coug_interfaces::msg::WayPoint>& waypoints,
                        const std::string& target_frame);

  void callService(Command command, const std::vector<std::string>& agents, bool aggregate);

 private:
  struct CallState {
    int total = 0;
    int responded = 0;
    int succeeded = 0;
    Command command;
    std::vector<std::string> failed;
    std::mutex mutex;
  };

  struct AgentEntry {
    rclcpp::Publisher<coug_interfaces::msg::WayPointList>::SharedPtr waypoint_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr waypoint_map_pub;
    std::array<rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr, 4> service_clients;
  };

  void callAgentService(const std::string& agent_name, Command command,
                        std::shared_ptr<CallState> state = nullptr);

  void recordResult(std::shared_ptr<CallState> state, bool success, const std::string& agent_name);

  static size_t commandIndex(Command command);

  static const char* commandName(Command command);

  std::string resolvedName(const std::string& agent_name, Command command) const;

  std::shared_ptr<rclcpp::Node> node_;
  swri_transform_util::TransformManagerPtr tf_manager_;
  StatusCallback status_;

  Config config_;
  std::map<std::string, AgentEntry> agents_;
};

}  // namespace coug_mapviz::utils
