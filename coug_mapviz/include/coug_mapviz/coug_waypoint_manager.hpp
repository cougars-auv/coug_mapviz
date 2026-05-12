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
 * @file coug_waypoint_manager.hpp
 * @brief MapViz plugin helper, manages waypoint storage and file I/O.
 * @author Nelson Durrant
 * @date May 2026
 */

#pragma once

#include <geographic_msgs/msg/geo_point.hpp>
#include <map>
#include <string>
#include <vector>

namespace coug_mapviz {

/**
 * @class CougWaypointManager
 * @brief Handles storage, retrieval, and serialization of waypoints.
 */
class CougWaypointManager {
 public:
  CougWaypointManager() = default;
  ~CougWaypointManager() = default;

  /**
   * @brief Adds a waypoint for the specified agent.
   * @param agent The agent namespace.
   * @param pose The waypoint pose (in target/local frame).
   */
  void addWaypoint(const std::string& agent, const geographic_msgs::msg::GeoPoint& pose);

  /**
   * @brief Replaces the waypoint list for a specific agent.
   * @param agent The agent namespace.
   * @param waypoints The new list of waypoints.
   */
  void setWaypoints(const std::string& agent,
                    const std::vector<geographic_msgs::msg::GeoPoint>& waypoints);

  /**
   * @brief Gets the waypoints for a specific agent.
   * @param agent The agent namespace.
   * @return A vector of waypoints.
   */
  std::vector<geographic_msgs::msg::GeoPoint> getWaypoints(const std::string& agent) const;

  /**
   * @brief Gets all managed waypoints as a map.
   * @return Map of agent namespace to waypoint list.
   */
  const std::map<std::string, std::vector<geographic_msgs::msg::GeoPoint>>& getAllWaypoints() const;

  /**
   * @brief Removes an agent and its waypoints from the manager.
   * @param agent The agent namespace.
   */
  void removeAgent(const std::string& agent);

  /**
   * @brief Clears waypoints for a specific agent.
   * @param agent The agent namespace.
   */
  void clearWaypoints(const std::string& agent);

  /**
   * @brief Clears all waypoints for all agents.
   */
  void clearAllWaypoints();

  /**
   * @brief Saves waypoints to a JSON file keyed by agent namespace.
   * @param filename The full path to the file.
   * @param agent Optional: only save this agent.
   * @return True if successful.
   */
  bool saveToFile(const std::string& filename, const std::string& agent = "") const;

  /**
   * @brief Loads waypoints from a JSON file keyed by agent namespace.
   * @param filename The full path to the file.
   * @param agent Optional: only load this agent.
   * @return True if successful.
   */
  bool loadFromFile(const std::string& filename, const std::string& agent = "");

 private:
  std::map<std::string, std::vector<geographic_msgs::msg::GeoPoint>> waypoints_;
};

}  // namespace coug_mapviz
