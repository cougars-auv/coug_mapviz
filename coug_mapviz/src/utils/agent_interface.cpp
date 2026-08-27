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

#include <swri_transform_util/frames.h>
#include <swri_transform_util/transform.h>

#include <coug_mapviz/utils/agent_interface.hpp>
#include <coug_mapviz/utils/geo_conversions.hpp>
#include <string>
#include <utility>

namespace coug_mapviz::utils {

void AgentInterface::initialize(const std::shared_ptr<rclcpp::Node>& node,
                                const swri_transform_util::TransformManagerPtr& tf_manager,
                                const std::vector<std::string>& agent_namespaces,
                                const Config& config, StatusCallback status_callback) {
  node_ = node;
  tf_manager_ = tf_manager;
  config_ = config;
  status_ = std::move(status_callback);

  for (const auto& agent_name : agent_namespaces) {
    AgentEntry agent;
    agent.waypoint_pub = node_->create_publisher<coug_interfaces::msg::WayPointList>(
        agent_name + "/" + config_.waypoint_topic, rclcpp::SystemDefaultsQoS());
    agent.waypoint_map_pub = node_->create_publisher<geometry_msgs::msg::PoseArray>(
        agent_name + "/" + config_.waypoint_map_topic, rclcpp::SystemDefaultsQoS());
    for (size_t i = 0; i < agent.service_clients.size(); ++i) {
      agent.service_clients[i] =
          node_->create_client<std_srvs::srv::Trigger>(agent_name + "/" + config_.service_names[i]);
    }
    agents_[agent_name] = std::move(agent);
  }
}

void AgentInterface::publishWaypoints(const std::string& agent_name,
                                      const std::vector<coug_interfaces::msg::WayPoint>& waypoints,
                                      const std::string& target_frame) {
  auto agent_it = agents_.find(agent_name);
  if (agent_it == agents_.end()) {
    status_(Status::kError, "Publisher not registered: " + agent_name);
    return;
  }

  coug_interfaces::msg::WayPointList waypoint_list;
  waypoint_list.header.frame_id = swri_transform_util::_wgs84_frame;
  waypoint_list.header.stamp = node_->now();
  waypoint_list.waypoints = waypoints;
  agent_it->second.waypoint_pub->publish(waypoint_list);

  swri_transform_util::Transform fixed_T_wgs84;
  if (tf_manager_->GetTransform(target_frame, swri_transform_util::_wgs84_frame, fixed_T_wgs84)) {
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.frame_id = target_frame;
    pose_array.header.stamp = node_->now();
    for (const auto& waypoint : waypoints) {
      const tf2::Vector3 point = wgs84ToFixed(waypoint, fixed_T_wgs84);
      geometry_msgs::msg::Pose pose;
      pose.position.x = point.x();
      pose.position.y = point.y();
      pose.position.z = waypoint.position.altitude;
      pose.orientation.w = 1.0;
      pose_array.poses.push_back(pose);
    }
    agent_it->second.waypoint_map_pub->publish(pose_array);
  }
}

void AgentInterface::callService(Command command, const std::vector<std::string>& agents,
                                 bool aggregate) {
  const std::string prefix = "[" + std::string(commandName(command)) + "] ";
  status_(Status::kInfo, prefix + "Calling service...");
  if (aggregate) {
    auto state = std::make_shared<CallState>();
    state->total = static_cast<int>(agents.size());
    state->command = command;
    for (const auto& agent_name : agents) callAgentService(agent_name, command, state);
  } else {
    for (const auto& agent_name : agents) callAgentService(agent_name, command);
  }
}

void AgentInterface::callAgentService(const std::string& agent_name, Command command,
                                      std::shared_ptr<CallState> state) {
  const auto agent_it = agents_.find(agent_name);
  const auto client =
      agent_it == agents_.end() ? nullptr : agent_it->second.service_clients[commandIndex(command)];
  if (!client || !client->service_is_ready()) {
    if (state)
      recordResult(state, false, agent_name);
    else
      status_(Status::kError, "Service not available: " + resolvedName(agent_name, command));
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  client->async_send_request(
      request, [this, agent_name, command,
                state](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        auto response = future.get();
        if (!response) {
          if (state)
            recordResult(state, false, agent_name);
          else
            status_(Status::kError, "Service call failed: " + resolvedName(agent_name, command));
          return;
        }
        if (state) {
          recordResult(state, response->success, agent_name);
        } else {
          const std::string prefix = "[" + std::string(commandName(command)) + "] ";
          status_(response->success ? Status::kInfo : Status::kWarning, prefix + response->message);
        }
      });
}

void AgentInterface::recordResult(std::shared_ptr<CallState> state, bool success,
                                  const std::string& agent_name) {
  Status level;
  std::string message;
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    if (success)
      ++state->succeeded;
    else
      state->failed.push_back(agent_name);

    if (++state->responded < state->total) return;

    const std::string prefix = "[" + std::string(commandName(state->command)) + "] ";
    if (state->succeeded == state->total) {
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

size_t AgentInterface::commandIndex(Command command) { return static_cast<size_t>(command); }

const char* AgentInterface::commandName(Command command) {
  switch (command) {
    case Command::kStart:
      return "START";
    case Command::kStop:
      return "STOP";
    case Command::kSurface:
      return "SURFACE";
    case Command::kHome:
      return "HOME";
  }
  return "UNKNOWN";
}

std::string AgentInterface::resolvedName(const std::string& agent_name, Command command) const {
  return agent_name + "/" + config_.service_names[commandIndex(command)];
}

}  // namespace coug_mapviz::utils
