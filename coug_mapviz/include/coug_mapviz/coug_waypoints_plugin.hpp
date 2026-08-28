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

#include <mapviz/map_canvas.h>
#include <mapviz/mapviz_plugin.h>
#include <swri_transform_util/transform.h>
#include <ui_coug_waypoints_config.h>

#include <QGLWidget>
#include <QMouseEvent>
#include <QObject>
#include <QPainter>
#include <QWidget>
#include <coug_mapviz/coug_waypoints_parameters.hpp>
#include <coug_mapviz/utils/fleet_interface.hpp>
#include <coug_mapviz/utils/waypoint_renderer.hpp>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace coug_mapviz {

class CougWaypointsPlugin : public mapviz::MapvizPlugin {
  Q_OBJECT

 public:
  // --- Plugin Lifecycle ---
  CougWaypointsPlugin();
  ~CougWaypointsPlugin() override;

  bool Initialize(QGLWidget* canvas) override;

  void Shutdown() override {}

  void Draw(double, double, double) override {}

  void Paint(QPainter* painter, double, double, double) override;

  void Transform() override {}

  void LoadConfig(const YAML::Node&, const std::string&) override {}

  void SaveConfig(YAML::Emitter&, const std::string&) override {}

  QWidget* GetConfigWidget(QWidget* parent) override {
    config_widget_->setParent(parent);
    return config_widget_;
  }

  bool SupportsPainting() override { return true; }

 protected:
  // --- Logging ---
  void PrintError(const std::string& message) override;

  void PrintInfo(const std::string& message) override;

  void PrintWarning(const std::string& message) override;

  // --- Event Handling ---
  bool eventFilter(QObject* object, QEvent* event) override;

  bool handleMousePress(QMouseEvent* event);

  bool handleMouseRelease(QMouseEvent* event);

  bool handleMouseMove(QMouseEvent* event);

 Q_SIGNALS:
  void StatusUpdateRequested(int level, const QString& message);

 public Q_SLOTS:
  void UpdateStatus(int level, const QString& message);

 protected Q_SLOTS:
  // --- UI Callbacks ---
  void AgentChanged(const QString& text);

  void EditorChanged(double value);

  void AltitudeModeChanged(bool checked);

  void PublishWaypoints();

  void ClearWaypoints();

  void SaveWaypoints();

  void LoadWaypoints();

  void Start() { callFleetService(utils::FleetInterface::Service::kStart); }

  void Stop() { callFleetService(utils::FleetInterface::Service::kStop); }

  void Surface() { callFleetService(utils::FleetInterface::Service::kSurface); }

  void Home() { callFleetService(utils::FleetInterface::Service::kHome); }

 private:
  // --- Helpers ---
  const std::vector<coug_interfaces::msg::WayPoint>& waypointsForAgent(
      const std::string& agent) const;

  std::vector<std::string> targetAgents();

  std::vector<coug_interfaces::msg::WayPoint>* currentWaypoints();

  coug_interfaces::msg::WayPoint* selectedWaypoint();

  int findWaypointAt(const QPointF& point);

  void clearWaypointSelection();

  void setEditorsEnabled(bool enabled);

  void setDepthEditorRange(bool altitude_mode);

  void populateEditors(const coug_interfaces::msg::WayPoint& waypoint);

  static QString missionDirectory();

  bool wgs84ToMap(double latitude, double longitude, QPointF& map_point) const;

  bool mapToWgs84(const QPointF& map_point, QPointF& lat_lon) const;

  void callFleetService(utils::FleetInterface::Service service);

  // --- ROS Interfaces ---
  utils::FleetInterface interface_;
  std::unique_ptr<utils::WaypointRenderer> renderer_;

  // --- Parameters ---
  std::shared_ptr<coug_waypoints::ParamListener> param_listener_;
  coug_waypoints::Params params_;

  // --- State ---
  Ui::coug_waypoints_config ui_;
  QWidget* config_widget_;
  mapviz::MapCanvas* map_canvas_;

  std::map<std::string, std::vector<coug_interfaces::msg::WayPoint>> waypoints_;
  std::string current_agent_;

  int selected_idx_;
  int dragged_idx_;
  QPointF mouse_down_pos_;
  qint64 mouse_down_time_;
};

}  // namespace coug_mapviz
