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

#include <array>
#include <coug_interfaces/msg/way_point.hpp>
#include <coug_interfaces/msg/way_point_list.hpp>
#include <coug_mapviz/coug_waypoints_parameters.hpp>
#include <cstdint>
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

class FleetInterface {
 public:
  enum class Status : std::uint8_t { kInfo, kWarning, kError };
  enum class Service : std::uint8_t { kStart, kStop, kSurface, kHome };
  static constexpr size_t kServiceCount = 4;

  using StatusCallback = std::function<void(Status, const std::string&)>;

  FleetInterface() = default;

  void initialize(const std::shared_ptr<rclcpp::Node>& node, const coug_waypoints::Params& params,
                  StatusCallback status_callback);

  void publishWaypoints(const std::string& agent_name,
                        const std::vector<coug_interfaces::msg::WayPoint>& waypoints);

  void callService(Service service, const std::vector<std::string>& agents);

 private:
  struct ServiceCallState {
    int total = 0;
    int responded = 0;
    int succeeded = 0;
    Service service = Service::kStart;
    std::vector<std::string> failed;
    std::string response_message;
    Status failure_status = Status::kWarning;
    std::mutex mutex;
  };

  struct AgentEntry {
    rclcpp::Publisher<coug_interfaces::msg::WayPointList>::SharedPtr waypoint_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr waypoint_nav2_pub;
    std::array<rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr, kServiceCount> service_clients;
  };

  void callAgentService(const std::string& agent_name, Service service,
                        const std::shared_ptr<ServiceCallState>& state);

  void recordResult(const std::shared_ptr<ServiceCallState>& state, bool success,
                    const std::string& agent_name, const std::string& response_message,
                    Status failure_status = Status::kWarning);

  [[nodiscard]] auto serviceName(Service service) const -> const std::string&;

  std::shared_ptr<rclcpp::Node> node_;
  StatusCallback status_;

  coug_waypoints::Params params_;
  std::map<std::string, AgentEntry> agents_;
};

}  // namespace coug_mapviz::utils
