// Copyright 2026 BYU FROST Lab
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
#include <cstddef>
#include <exception>
#include <map>
#include <memory>
#include <mutex>
#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "coug_interfaces/msg/way_point.hpp"
#include "coug_interfaces/msg/way_point_list.hpp"
#include "coug_mapviz/coug_waypoints_parameters.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "std_srvs/srv/trigger.hpp"

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
    agent.waypoint_viz_pub = node_->create_publisher<geometry_msgs::msg::PoseArray>(
        build_name(agent_name, params_.waypoint_viz_topic), rclcpp::SystemDefaultsQoS());
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
    status_(Status::kError, "No waypoint publisher registered for '" + agent_name + "'.");
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
    pose_array.poses.push_back(pose);

    for (const auto& subwaypoint : waypoint.subwaypoints) {
      geometry_msgs::msg::Pose sub_pose;
      sub_pose.position = subwaypoint;
      pose_array.poses.push_back(sub_pose);
    }
  }
  agent_it->second.waypoint_viz_pub->publish(pose_array);
}

void FleetInterface::callService(Service service, const std::vector<std::string>& agents) {
  if (agents.empty()) {
    status_(Status::kError, "No agents selected.");
    return;
  }
  const std::string prefix = "[" + serviceName(service) + "] ";
  status_(Status::kInfo,
          prefix + "Calling service on " + std::to_string(agents.size()) + " agent(s)...");
  auto state = std::make_shared<ServiceCallState>();
  state->total = static_cast<int>(agents.size());
  state->agents = agents;
  state->service = service;
  for (const auto& agent_name : agents) {
    callAgentService(agent_name, service, state);
  }
}

void FleetInterface::callAgentService(const std::string& agent_name, Service service,
                                      const std::shared_ptr<ServiceCallState>& state) {
  const auto agent_it = agents_.find(agent_name);
  const auto client = agent_it == agents_.end()
                          ? nullptr
                          : agent_it->second.service_clients[static_cast<size_t>(service)];
  if (!client || !client->service_is_ready()) {
    recordResult(state, false, agent_name,
                 "Service '" + build_name(agent_name, serviceName(service)) + "' not available.");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  client->async_send_request(
      request, [this, agent_name, service,
                // NOLINTNEXTLINE(performance-unnecessary-value-param)
                state](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        const std::string failed =
            "Failed to call '" + build_name(agent_name, serviceName(service)) + "'";
        try {
          const auto response = future.get();
          if (!response) {
            recordResult(state, false, agent_name, failed + ".");
            return;
          }
          recordResult(state, response->success, agent_name, response->message);
        } catch (const std::exception& e) {
          recordResult(state, false, agent_name, failed + ": " + e.what());
        }
      });
}

void FleetInterface::recordResult(const std::shared_ptr<ServiceCallState>& state, bool success,
                                  const std::string& agent_name,
                                  const std::string& response_message) {
  Status level = Status::kInfo;
  std::string message;
  {
    const std::lock_guard<std::mutex> lock(state->mutex);
    if (success) {
      ++state->succeeded;
    }
    state->responses[agent_name] =
        "[" + agent_name + "] " +
        (response_message.empty() ? "Service call completed." : response_message);

    if (++state->responded < state->total) {
      return;
    }

    if (state->succeeded != state->total) {
      level = state->succeeded == 0 ? Status::kError : Status::kWarning;
    }
    message = "[" + serviceName(state->service) + "]";
    for (const auto& agent : state->agents) {
      message += " " + state->responses[agent];
    }
  }
  status_(level, message);
}

auto FleetInterface::serviceName(Service service) const -> const std::string& {
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
