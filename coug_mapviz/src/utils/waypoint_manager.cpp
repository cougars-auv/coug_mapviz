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
                                  const coug_interfaces::msg::WayPoint& wp) {
  waypoints_[agent].push_back(wp);
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
                                                                    size_t index) {
  auto it = waypoints_.find(agent);
  if (it == waypoints_.end() || index >= it->second.size()) {
    return nullptr;
  }
  return &it->second[index];
}

void WaypointManager::removeWaypoint(const std::string& agent, size_t index) {
  auto it = waypoints_.find(agent);
  if (it != waypoints_.end() && index < it->second.size()) {
    it->second.erase(it->second.begin() + index);
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

  for (const auto& [agent, wps] : waypoints_) {
    if (!specific_agent.empty() && agent != specific_agent) {
      continue;
    }

    QJsonArray waypoints_array;
    for (const auto& wp : wps) {
      QJsonObject wp_obj;
      wp_obj["lon"] = wp.position.longitude;
      wp_obj["lat"] = wp.position.latitude;
      wp_obj["z"] = wp.position.altitude;
      wp_obj["mode"] = static_cast<int>(wp.mode);
      wp_obj["speed_rpm"] = wp.speed_rpm;
      wp_obj["capture_radius"] = wp.capture_radius;
      wp_obj["capture_radius_z"] = wp.capture_radius_z;
      wp_obj["slip_radius"] = wp.slip_radius;
      wp_obj["slip_radius_z"] = wp.slip_radius_z;
      waypoints_array.append(wp_obj);
    }
    root[QString::fromStdString(agent)] = waypoints_array;
  }

  QJsonDocument doc(root);
  QFile file(QString::fromStdString(filename));
  if (file.open(QIODevice::WriteOnly)) {
    file.write(doc.toJson());
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
  QJsonDocument doc = QJsonDocument::fromJson(data);

  if (!doc.isObject()) {
    return false;
  }

  QJsonObject obj = doc.object();
  int loaded_count = 0;

  for (const QString& agent_key : obj.keys()) {
    std::string agent = agent_key.toStdString();

    if (!specific_agent.empty() && agent != specific_agent) {
      continue;
    }

    std::vector<coug_interfaces::msg::WayPoint> wps;
    QJsonArray array = obj[agent_key].toArray();

    for (const auto& val : array) {
      QJsonObject wp_obj = val.toObject();
      if (wp_obj.contains("lat") && wp_obj.contains("lon")) {
        coug_interfaces::msg::WayPoint wp = defaults;
        wp.position.longitude = wp_obj["lon"].toDouble();
        wp.position.latitude = wp_obj["lat"].toDouble();
        wp.position.altitude = wp_obj["z"].toDouble();
        wp.mode = static_cast<uint8_t>(wp_obj["mode"].toInt());

        auto load_if_present = [&wp_obj](const QString& key, double& field) {
          if (wp_obj.contains(key)) {
            field = wp_obj[key].toDouble();
          }
        };
        load_if_present("speed_rpm", wp.speed_rpm);
        load_if_present("capture_radius", wp.capture_radius);
        load_if_present("capture_radius_z", wp.capture_radius_z);
        load_if_present("slip_radius", wp.slip_radius);
        load_if_present("slip_radius_z", wp.slip_radius_z);
        wps.push_back(wp);
      }
    }
    waypoints_[agent] = wps;
    loaded_count++;
  }

  return loaded_count > 0;
}

}  // namespace coug_mapviz::utils
