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

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QString>
#include <coug_mapviz/utils/waypoint_manager.hpp>

namespace coug_mapviz::utils {

void WaypointManager::addWaypoint(const std::string& agent,
                                  const coug_interfaces::msg::WayPoint& waypoint) {
  waypoints_[agent].push_back(waypoint);
}

void WaypointManager::setWaypoints(const std::string& agent,
                                   const std::vector<coug_interfaces::msg::WayPoint>& waypoints) {
  waypoints_[agent] = waypoints;
}

std::vector<coug_interfaces::msg::WayPoint> WaypointManager::getWaypoints(
    const std::string& agent) const {
  if (waypoints_.find(agent) != waypoints_.end()) {
    return waypoints_.at(agent);
  }
  return {};
}

coug_interfaces::msg::WayPoint* WaypointManager::getWaypointMutable(const std::string& agent,
                                                                    size_t waypoint_idx) {
  auto it = waypoints_.find(agent);
  if (it == waypoints_.end() || waypoint_idx >= it->second.size()) {
    return nullptr;
  }
  return &it->second[waypoint_idx];
}

void WaypointManager::removeWaypoint(const std::string& agent, size_t waypoint_idx) {
  auto it = waypoints_.find(agent);
  if (it != waypoints_.end() && waypoint_idx < it->second.size()) {
    it->second.erase(it->second.begin() + waypoint_idx);
  }
}

const std::map<std::string, std::vector<coug_interfaces::msg::WayPoint>>&
WaypointManager::getAllWaypoints() const {
  return waypoints_;
}

void WaypointManager::clearWaypoints(const std::string& agent) { waypoints_[agent].clear(); }

void WaypointManager::clearAllWaypoints() { waypoints_.clear(); }

void WaypointManager::removeAgent(const std::string& agent) { waypoints_.erase(agent); }

bool WaypointManager::saveToFile(const std::string& filename,
                                 const std::string& specific_agent) const {
  QJsonObject root;

  for (const auto& [agent, waypoints] : waypoints_) {
    if (!specific_agent.empty() && agent != specific_agent) {
      continue;
    }

    QJsonArray waypoints_array;
    for (const auto& waypoint : waypoints) {
      QJsonObject waypoint_obj;
      waypoint_obj["lon"] = waypoint.position.longitude;
      waypoint_obj["lat"] = waypoint.position.latitude;
      waypoint_obj["z"] = waypoint.position.altitude;
      waypoint_obj["mode"] = static_cast<int>(waypoint.mode);
      waypoint_obj["speed_rpm"] = waypoint.speed_rpm;
      waypoint_obj["capture_radius"] = waypoint.capture_radius;
      waypoint_obj["capture_radius_z"] = waypoint.capture_radius_z;
      waypoint_obj["slip_radius"] = waypoint.slip_radius;
      waypoint_obj["slip_radius_z"] = waypoint.slip_radius_z;
      waypoints_array.append(waypoint_obj);
    }
    root[QString::fromStdString(agent)] = waypoints_array;
  }

  QJsonDocument json_doc(root);
  QFile file(QString::fromStdString(filename));
  if (file.open(QIODevice::WriteOnly)) {
    file.write(json_doc.toJson());
    file.close();
    return true;
  }
  return false;
}

bool WaypointManager::loadFromFile(const std::string& filename,
                                   const coug_interfaces::msg::WayPoint& defaults,
                                   const std::string& specific_agent) {
  QFile file(QString::fromStdString(filename));
  if (!file.open(QIODevice::ReadOnly)) {
    return false;
  }

  QByteArray data = file.readAll();
  QJsonDocument json_doc = QJsonDocument::fromJson(data);

  if (!json_doc.isObject()) {
    return false;
  }

  QJsonObject root_obj = json_doc.object();
  int loaded_count = 0;

  for (const QString& agent_key : root_obj.keys()) {
    std::string agent = agent_key.toStdString();

    if (!specific_agent.empty() && agent != specific_agent) {
      continue;
    }

    std::vector<coug_interfaces::msg::WayPoint> waypoints;
    QJsonArray array = root_obj[agent_key].toArray();

    for (const auto& array_value : array) {
      QJsonObject waypoint_obj = array_value.toObject();
      if (waypoint_obj.contains("lat") && waypoint_obj.contains("lon")) {
        coug_interfaces::msg::WayPoint waypoint = defaults;
        waypoint.position.longitude = waypoint_obj["lon"].toDouble();
        waypoint.position.latitude = waypoint_obj["lat"].toDouble();
        waypoint.position.altitude = waypoint_obj["z"].toDouble();
        waypoint.mode = static_cast<uint8_t>(waypoint_obj["mode"].toInt());

        auto loadIfPresent = [&waypoint_obj](const QString& key, double& field) {
          if (waypoint_obj.contains(key)) {
            field = waypoint_obj[key].toDouble();
          }
        };
        loadIfPresent("speed_rpm", waypoint.speed_rpm);
        loadIfPresent("capture_radius", waypoint.capture_radius);
        loadIfPresent("capture_radius_z", waypoint.capture_radius_z);
        loadIfPresent("slip_radius", waypoint.slip_radius);
        loadIfPresent("slip_radius_z", waypoint.slip_radius_z);
        waypoints.push_back(waypoint);
      }
    }
    waypoints_[agent] = waypoints;
    loaded_count++;
  }

  return loaded_count > 0;
}

}  // namespace coug_mapviz::utils
