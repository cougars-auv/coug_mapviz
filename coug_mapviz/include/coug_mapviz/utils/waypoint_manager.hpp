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

#include <coug_interfaces/msg/way_point.hpp>
#include <map>
#include <string>
#include <vector>

namespace coug_mapviz::utils {

class WaypointManager {
 public:
  WaypointManager() = default;
  ~WaypointManager() = default;

  void addWaypoint(const std::string& agent, const coug_interfaces::msg::WayPoint& waypoint);

  void setWaypoints(const std::string& agent,
                    const std::vector<coug_interfaces::msg::WayPoint>& waypoints);

  std::vector<coug_interfaces::msg::WayPoint> getWaypoints(const std::string& agent) const;

  coug_interfaces::msg::WayPoint* getWaypointMutable(const std::string& agent, size_t waypoint_idx);

  void removeWaypoint(const std::string& agent, size_t waypoint_idx);

  const std::map<std::string, std::vector<coug_interfaces::msg::WayPoint>>& getAllWaypoints() const;

  void removeAgent(const std::string& agent);

  void clearWaypoints(const std::string& agent);

  void clearAllWaypoints();

  bool saveToFile(const std::string& filename, const std::string& agent = "") const;

  bool loadFromFile(const std::string& filename, const coug_interfaces::msg::WayPoint& defaults,
                    const std::string& agent = "");

 private:
  std::map<std::string, std::vector<coug_interfaces::msg::WayPoint>> waypoints_;
};

}  // namespace coug_mapviz::utils
