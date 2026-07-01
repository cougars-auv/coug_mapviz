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
 * @file coug_comms_client.cpp
 * @brief Implementation of the CougCommsClient.
 * @author Nelson Durrant
 * @date May 2026
 */

#include <swri_transform_util/frames.h>
#include <swri_transform_util/transform.h>

#include <coug_mapviz/coug_comms_client.hpp>
#include <geographic_msgs/msg/key_value.hpp>
#include <geographic_msgs/msg/way_point.hpp>
#include <string>
#include <utility>

namespace coug_mapviz {

void CougCommsClient::initialize(const std::shared_ptr<rclcpp::Node>& node,
                                 const swri_transform_util::TransformManagerPtr& tf_manager,
                                 const std::vector<std::string>& agent_namespaces,
                                 const std::string& waypoint_topic,
                                 const std::string& waypoints_map_topic,
                                 const std::map<std::string, std::string>& services,
                                 StatusCallback status_cb) {
  node_ = node;
  tf_manager_ = tf_manager;
  status_ = std::move(status_cb);
  waypoint_topic_ = waypoint_topic;
  waypoints_map_topic_ = waypoints_map_topic;

  for (const auto& ns : agent_namespaces) {
    publishers_[ns + "/" + waypoint_topic_] =
        node_->create_publisher<coug_interfaces::msg::WayPointList>(ns + "/" + waypoint_topic_,
                                                                    rclcpp::SystemDefaultsQoS());
    map_publishers_[ns + "/" + waypoints_map_topic_] =
        node_->create_publisher<geometry_msgs::msg::PoseArray>(ns + "/" + waypoints_map_topic_,
                                                               rclcpp::SystemDefaultsQoS());
    for (const auto& [cmd, svc] : services) {
      clients_[ns][cmd] = node_->create_client<std_srvs::srv::Trigger>(ns + "/" + svc);
    }
  }
}

void CougCommsClient::publishWaypoints(const std::string& agent,
                                       const std::vector<CougWaypoint>& wps,
                                       const std::string& target_frame) {
  std::string topic = agent + "/" + waypoint_topic_;
  coug_interfaces::msg::WayPointList waypoint_list;
  waypoint_list.header.frame_id = swri_transform_util::_wgs84_frame;
  waypoint_list.header.stamp = node_->now();
  for (const auto& cwp : wps) {
    geographic_msgs::msg::WayPoint wp;
    wp.position = cwp.position;
    geographic_msgs::msg::KeyValue speed_kv;
    speed_kv.key = "speed_rpm";
    speed_kv.value = std::to_string(cwp.speed_rpm);
    wp.props.push_back(speed_kv);
    geographic_msgs::msg::KeyValue capture_kv;
    capture_kv.key = "capture_radius";
    capture_kv.value = std::to_string(cwp.capture_radius);
    wp.props.push_back(capture_kv);
    geographic_msgs::msg::KeyValue slip_kv;
    slip_kv.key = "slip_radius";
    slip_kv.value = std::to_string(cwp.slip_radius);
    wp.props.push_back(slip_kv);
    waypoint_list.waypoints.push_back(wp);
  }

  publishers_[topic]->publish(waypoint_list);

  std::string map_topic = agent + "/" + waypoints_map_topic_;
  swri_transform_util::Transform transform;
  if (tf_manager_->GetTransform(target_frame, swri_transform_util::_wgs84_frame, transform)) {
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.frame_id = target_frame;
    pose_array.header.stamp = node_->now();
    for (const auto& cwp : wps) {
      tf2::Vector3 point(cwp.position.longitude, cwp.position.latitude, 0.0);
      point = transform * point;
      geometry_msgs::msg::Pose pose;
      pose.position.x = point.x();
      pose.position.y = point.y();
      pose.position.z = cwp.position.altitude;
      pose.orientation.w = 1.0;
      pose_array.poses.push_back(pose);
    }
    map_publishers_[map_topic]->publish(pose_array);
  }
}

void CougCommsClient::callService(const std::string& cmd, const std::vector<std::string>& agents,
                                  bool aggregate) {
  status_(Status::kInfo, "[" + cmd + "] Calling service...");
  if (aggregate) {
    auto state = std::make_shared<CallState>();
    state->total = static_cast<int>(agents.size());
    state->cmd = cmd;
    for (const auto& ns : agents) callAgentService(ns, cmd, state);
  } else {
    for (const auto& ns : agents) callAgentService(ns, cmd);
  }
}

void CougCommsClient::callAgentService(const std::string& ns, const std::string& cmd,
                                       std::shared_ptr<CallState> state) {
  auto ns_it = clients_.find(ns);
  if (ns_it == clients_.end() || ns_it->second.find(cmd) == ns_it->second.end()) {
    if (state)
      recordResult(state, false, ns);
    else
      status_(Status::kError, "Service not available: " + ns + "/" + cmd);
    return;
  }
  auto& client = clients_[ns][cmd];
  if (!client->service_is_ready()) {
    if (state)
      recordResult(state, false, ns);
    else
      status_(Status::kError, "Service not available: " + ns + "/" + cmd);
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  client->async_send_request(
      request, [this, ns, cmd, state](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        auto response = future.get();
        if (!response) {
          if (state)
            recordResult(state, false, ns);
          else
            status_(Status::kError, "Service call failed:" + ns + "/" + cmd);
          return;
        }
        if (state) {
          recordResult(state, response->success, ns);
        } else {
          std::string prefix = "[" + cmd + "] ";
          if (response->success)
            status_(Status::kInfo, prefix + response->message);
          else
            status_(Status::kWarning, prefix + response->message);
        }
      });
}

void CougCommsClient::recordResult(std::shared_ptr<CallState> state, bool success,
                                   const std::string& agent) {
  std::lock_guard<std::mutex> lock(state->mutex);
  if (success)
    state->succeeded++;
  else
    state->failed.push_back(agent);

  if (++state->responded < state->total) return;

  int s = state->succeeded, t = state->total;
  std::string prefix = "[" + state->cmd + "] ";
  if (s == t) {
    status_(Status::kInfo, prefix + "All " + std::to_string(t) + " agent(s) confirmed");
  } else {
    std::string failed_str;
    for (const auto& f : state->failed) failed_str += " " + f;
    std::string msg =
        prefix + std::to_string(s) + "/" + std::to_string(t) + " confirmed; failed:" + failed_str;
    if (s == 0)
      status_(Status::kError, msg);
    else
      status_(Status::kWarning, msg);
  }
}

}  // namespace coug_mapviz
