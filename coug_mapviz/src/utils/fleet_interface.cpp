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

#include <coug_mapviz/utils/fleet_interface.hpp>
#include <stdexcept>
#include <string>
#include <utility>

namespace coug_mapviz::utils {

using coug_interfaces::msg::WayPoint;
using coug_interfaces::msg::WayPointList;

namespace {

const auto build_name = [](const std::string& agent_name, const std::string& name) {
  return "/" + agent_name + "/" + name;
};

}  // namespace

void FleetInterface::initialize(const std::shared_ptr<rclcpp::Node>& node,
                                const coug_waypoints::Params& params,
                                StatusCallback status_callback) {
  node_ = node;
  params_ = params;
  status_ = std::move(status_callback);

  for (const auto& agent_name : params_.agent_list) {
    AgentEntry agent;
    agent.waypoint_pub = node_->create_publisher<WayPointList>(
        build_name(agent_name, params_.waypoint_topic), rclcpp::SystemDefaultsQoS());
    agent.waypoint_nav2_pub = node_->create_publisher<geometry_msgs::msg::PoseArray>(
        build_name(agent_name, params_.waypoint_nav2_topic), rclcpp::SystemDefaultsQoS());
    for (size_t i = 0; i < agent.service_clients.size(); ++i) {
      const auto service = static_cast<Service>(i);
      agent.service_clients[i] = node_->create_client<std_srvs::srv::Trigger>(
          build_name(agent_name, serviceName(service)));
    }
    agents_[agent_name] = std::move(agent);
  }
}

void FleetInterface::publishWaypoints(const std::string& agent_name,
                                      const std::vector<WayPoint>& waypoints) {
  auto agent_it = agents_.find(agent_name);
  if (agent_it == agents_.end()) {
    status_(Status::kError, "Publisher not registered: " + agent_name);
    return;
  }

  WayPointList waypoint_list;
  waypoint_list.header.frame_id = params_.map_frame;
  waypoint_list.header.stamp = node_->now();
  waypoint_list.waypoints = waypoints;
  agent_it->second.waypoint_pub->publish(waypoint_list);

  geometry_msgs::msg::PoseArray pose_array;
  pose_array.header.frame_id = params_.map_frame;
  pose_array.header.stamp = node_->now();
  for (const auto& waypoint : waypoints) {
    geometry_msgs::msg::Pose pose;
    pose.position = waypoint.position;
    pose.orientation.w = 1.0;
    pose_array.poses.push_back(pose);
  }
  agent_it->second.waypoint_nav2_pub->publish(pose_array);
}

void FleetInterface::callService(Service service, const std::vector<std::string>& agents) {
  if (agents.empty()) {
    status_(Status::kError, "No agents selected.");
    return;
  }
  const std::string prefix = "[" + serviceName(service) + "] ";
  status_(Status::kInfo, prefix + "Calling service...");
  auto state = std::make_shared<ServiceCallState>();
  state->total = static_cast<int>(agents.size());
  state->service = service;
  for (const auto& agent_name : agents) callAgentService(agent_name, service, state);
}

void FleetInterface::callAgentService(const std::string& agent_name, Service service,
                                      const std::shared_ptr<ServiceCallState>& state) {
  const auto agent_it = agents_.find(agent_name);
  const auto client = agent_it == agents_.end()
                          ? nullptr
                          : agent_it->second.service_clients[static_cast<size_t>(service)];
  if (!client || !client->service_is_ready()) {
    recordResult(state, false, agent_name,
                 "Service not available: " + build_name(agent_name, serviceName(service)),
                 Status::kError);
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  client->async_send_request(
      request, [this, agent_name, service,
                state](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        auto response = future.get();
        if (!response) {
          recordResult(state, false, agent_name,
                       "Service call failed: " + build_name(agent_name, serviceName(service)),
                       Status::kError);
          return;
        }
        recordResult(state, response->success, agent_name, response->message);
      });
}

void FleetInterface::recordResult(const std::shared_ptr<ServiceCallState>& state, bool success,
                                  const std::string& agent_name,
                                  const std::string& response_message, Status failure_status) {
  Status level;
  std::string message;
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    if (success) {
      ++state->succeeded;
    } else {
      state->failed.push_back(agent_name);
      state->failure_status = failure_status;
    }
    state->response_message = response_message;

    if (++state->responded < state->total) return;

    const std::string prefix = "[" + serviceName(state->service) + "] ";
    if (state->total == 1) {
      level = success ? Status::kInfo : state->failure_status;
      message = prefix + (response_message.empty() ? "Service call completed." : response_message);
    } else if (state->succeeded == state->total) {
      level = Status::kInfo;
      message = prefix + "All " + std::to_string(state->total) + " agent(s) confirmed.";
    } else {
      std::string failed_agents;
      for (const auto& failed_agent : state->failed) failed_agents += " " + failed_agent;
      level = state->succeeded == 0 ? Status::kError : Status::kWarning;
      message = prefix + std::to_string(state->succeeded) + "/" + std::to_string(state->total) +
                " confirmed; failed:" + failed_agents + ".";
    }
  }
  status_(level, message);
}

const std::string& FleetInterface::serviceName(Service service) const {
  switch (service) {
    case Service::kStart:
      return params_.start_service;
    case Service::kStop:
      return params_.stop_service;
    case Service::kSurface:
      return params_.surface_service;
    case Service::kHome:
      return params_.home_service;
  }
  throw std::logic_error("UNKNOWN");
}

}  // namespace coug_mapviz::utils
