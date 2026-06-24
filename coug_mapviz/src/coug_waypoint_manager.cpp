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
 * @file coug_waypoint_manager.cpp
 * @brief Implementation of the CougWaypointManager.
 * @author Nelson Durrant
 * @date May 2026
 */

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QString>
#include <coug_mapviz/coug_waypoint_manager.hpp>

namespace coug_mapviz {

void CougWaypointManager::addWaypoint(const std::string& agent, const CougWaypoint& wp) {
  waypoints_[agent].push_back(wp);
}

void CougWaypointManager::setWaypoints(const std::string& agent,
                                       const std::vector<CougWaypoint>& waypoints) {
  waypoints_[agent] = waypoints;
}

std::vector<CougWaypoint> CougWaypointManager::getWaypoints(const std::string& agent) const {
  if (waypoints_.find(agent) != waypoints_.end()) {
    return waypoints_.at(agent);
  }
  return {};
}

CougWaypoint* CougWaypointManager::getWaypointMutable(const std::string& agent, size_t index) {
  auto it = waypoints_.find(agent);
  if (it == waypoints_.end() || index >= it->second.size()) {
    return nullptr;
  }
  return &it->second[index];
}

void CougWaypointManager::removeWaypoint(const std::string& agent, size_t index) {
  auto it = waypoints_.find(agent);
  if (it != waypoints_.end() && index < it->second.size()) {
    it->second.erase(it->second.begin() + index);
  }
}

const std::map<std::string, std::vector<CougWaypoint>>& CougWaypointManager::getAllWaypoints()
    const {
  return waypoints_;
}

void CougWaypointManager::clearWaypoints(const std::string& agent) { waypoints_[agent].clear(); }

void CougWaypointManager::clearAllWaypoints() { waypoints_.clear(); }

void CougWaypointManager::removeAgent(const std::string& agent) { waypoints_.erase(agent); }

bool CougWaypointManager::saveToFile(const std::string& filename,
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
      wp_obj["speed_rpm"] = wp.speed_rpm;
      wp_obj["capture_radius"] = wp.capture_radius;
      wp_obj["slip_radius"] = wp.slip_radius;
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

bool CougWaypointManager::loadFromFile(const std::string& filename,
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

    std::vector<CougWaypoint> wps;
    QJsonArray array = obj[agent_key].toArray();

    for (const auto& val : array) {
      QJsonObject wp_obj = val.toObject();
      if (wp_obj.contains("lat") && wp_obj.contains("lon")) {
        CougWaypoint cwp;
        cwp.position.longitude = wp_obj["lon"].toDouble();
        cwp.position.latitude = wp_obj["lat"].toDouble();
        cwp.position.altitude = wp_obj["z"].toDouble();
        cwp.speed_rpm =
            wp_obj.contains("speed_rpm") ? wp_obj["speed_rpm"].toDouble() : kDefaultSpeedRpm;
        cwp.capture_radius = wp_obj.contains("capture_radius") ? wp_obj["capture_radius"].toDouble()
                                                               : kDefaultCaptureRadius;
        cwp.slip_radius =
            wp_obj.contains("slip_radius") ? wp_obj["slip_radius"].toDouble() : kDefaultSlipRadius;
        wps.push_back(cwp);
      }
    }
    waypoints_[agent] = wps;
    loaded_count++;
  }

  return loaded_count > 0;
}

}  // namespace coug_mapviz
