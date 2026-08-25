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

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QString>
#include <coug_interfaces/msg/way_point.hpp>
#include <coug_mapviz/utils/waypoint_store.hpp>
#include <string>
#include <vector>

namespace coug_mapviz::utils {

inline bool saveMission(const std::string& filename, const WaypointMap& waypoints,
                        const std::string& specific_agent = "") {
  QJsonObject root_obj;

  for (const auto& [agent, agent_waypoints] : waypoints) {
    if (!specific_agent.empty() && agent != specific_agent) {
      continue;
    }

    QJsonArray waypoints_array;
    for (const auto& waypoint : agent_waypoints) {
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
    root_obj[QString::fromStdString(agent)] = waypoints_array;
  }

  QJsonDocument json_doc(root_obj);
  QFile file(QString::fromStdString(filename));
  if (file.open(QIODevice::WriteOnly)) {
    file.write(json_doc.toJson());
    file.close();
    return true;
  }
  return false;
}

inline bool loadMission(const std::string& filename, const coug_interfaces::msg::WayPoint& defaults,
                        WaypointMap& waypoints, const std::string& specific_agent = "") {
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

    std::vector<coug_interfaces::msg::WayPoint> agent_waypoints;
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
        agent_waypoints.push_back(waypoint);
      }
    }
    waypoints[agent] = agent_waypoints;
    loaded_count++;
  }

  return loaded_count > 0;
}

}  // namespace coug_mapviz::utils
