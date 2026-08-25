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
#include <utility>
#include <vector>

namespace coug_mapviz::utils {

using WaypointMap = std::map<std::string, std::vector<coug_interfaces::msg::WayPoint>>;

class WaypointStore {
 public:
  WaypointStore() = default;
  ~WaypointStore() = default;

  const std::vector<coug_interfaces::msg::WayPoint>& getWaypoints(const std::string& agent) const {
    static const std::vector<coug_interfaces::msg::WayPoint> kNoWaypoints;

    auto it = waypoints_.find(agent);
    return it != waypoints_.end() ? it->second : kNoWaypoints;
  }

  const WaypointMap& getAllWaypoints() const { return waypoints_; }

  void addWaypoint(const std::string& agent, const coug_interfaces::msg::WayPoint& waypoint) {
    waypoints_[agent].push_back(waypoint);
  }

  void setWaypoints(const std::string& agent,
                    const std::vector<coug_interfaces::msg::WayPoint>& waypoints) {
    waypoints_[agent] = waypoints;
  }

  template <typename Mutator>
  bool modifyWaypoint(const std::string& agent, size_t waypoint_idx, Mutator&& mutate) {
    auto it = waypoints_.find(agent);
    if (it == waypoints_.end() || waypoint_idx >= it->second.size()) {
      return false;
    }
    std::forward<Mutator>(mutate)(it->second[waypoint_idx]);
    return true;
  }

  void removeWaypoint(const std::string& agent, size_t waypoint_idx) {
    auto it = waypoints_.find(agent);
    if (it != waypoints_.end() && waypoint_idx < it->second.size()) {
      it->second.erase(it->second.begin() + waypoint_idx);
    }
  }

  void clearWaypoints(const std::string& agent) { waypoints_[agent].clear(); }

  void clearAllWaypoints() { waypoints_.clear(); }

  void removeAgent(const std::string& agent) { waypoints_.erase(agent); }

 private:
  WaypointMap waypoints_;
};

}  // namespace coug_mapviz::utils
