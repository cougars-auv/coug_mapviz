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

#include <coug_interfaces/msg/way_point_list.hpp>
#include <coug_mapviz/utils/waypoint_store.hpp>
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

  using StatusCallback = std::function<void(Status, const std::string&)>;

  AgentInterface() = default;

  void initialize(const std::shared_ptr<rclcpp::Node>& node,
                  const swri_transform_util::TransformManagerPtr& tf_manager,
                  const std::vector<std::string>& agent_namespaces,
                  const std::string& waypoint_topic, const std::string& waypoint_map_topic,
                  const std::map<std::string, std::string>& services,
                  StatusCallback status_callback);

  void publishWaypoints(const std::string& agent,
                        const std::vector<coug_interfaces::msg::WayPoint>& waypoints,
                        const std::string& target_frame);

  void callService(const std::string& cmd, const std::vector<std::string>& agents, bool aggregate);

 private:
  struct CallState {
    int total = 0;
    int responded = 0;
    int succeeded = 0;
    std::string cmd;
    std::vector<std::string> failed;
    std::mutex mutex;
  };

  void callAgentService(const std::string& agent_ns, const std::string& cmd,
                        std::shared_ptr<CallState> state = nullptr);

  void recordResult(std::shared_ptr<CallState> state, bool success, const std::string& agent);

  std::string resolvedName(const std::string& agent_ns, const std::string& cmd) const;

  std::shared_ptr<rclcpp::Node> node_;
  swri_transform_util::TransformManagerPtr tf_manager_;
  StatusCallback status_;

  std::string waypoint_topic_;
  std::string waypoint_map_topic_;
  std::map<std::string, std::string> services_;

  std::map<std::string, rclcpp::Publisher<coug_interfaces::msg::WayPointList>::SharedPtr>
      waypoint_pubs_;
  std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr>
      waypoint_map_pubs_;
  std::map<std::string, std::map<std::string, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr>>
      service_clients_;
};

}  // namespace coug_mapviz::utils
