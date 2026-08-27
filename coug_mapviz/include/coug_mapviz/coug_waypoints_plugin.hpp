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

  void Draw(double x, double y, double scale) override;

  void Paint(QPainter* painter, double x, double y, double scale) override;

  void Transform() override {}

  void LoadConfig(const YAML::Node& node, const std::string& path) override {
    (void)node;
    (void)path;
  }

  void SaveConfig(YAML::Emitter& emitter, const std::string& path) override {
    (void)emitter;
    (void)path;
  }

  QWidget* GetConfigWidget(QWidget* parent) override;

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
  void HandleStatusUpdate(int level, const QString& message);

 protected Q_SLOTS:
  // --- UI Callbacks ---
  void PublishWaypoints();

  void Clear();

  void SaveWaypoints();

  void LoadWaypoints();

  void AgentChanged(const QString& text);

  void EditorChanged(double value);

  void AltitudeModeChanged(bool checked);

  void Start() { callService(utils::FleetInterface::Command::kStart); }

  void Stop() { callService(utils::FleetInterface::Command::kStop); }

  void Surface() { callService(utils::FleetInterface::Command::kSurface); }

  void Home() { callService(utils::FleetInterface::Command::kHome); }

 private:
  // --- Helpers ---
  const std::vector<coug_interfaces::msg::WayPoint>& agentWaypoints(const std::string& agent) const;

  bool isAgentKnown(const std::string& agent) const;

  bool resolveSelectedAgent(std::string& agent);

  void applyDefaultsToEditors();

  void setWaypointEditorsEnabled(bool enabled);

  void setDepthEditorRange(bool altitude_mode);

  void populateEditors(const coug_interfaces::msg::WayPoint& waypoint);

  void deselectWaypoint();

  void publishAll();

  void callService(utils::FleetInterface::Command command);

  static QString missionDirectory();

  bool glToWgs84(const QPointF& gl_point, geographic_msgs::msg::GeoPoint& geo_point);

  int findClosestWaypoint(const QPointF& point, double& distance);

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
  coug_interfaces::msg::WayPoint default_waypoint_;
  std::string current_agent_;
  std::vector<std::string> agent_list_;

  int selected_idx_;
  int dragged_idx_;
  QPointF mouse_down_pos_;
  qint64 mouse_down_time_;
};

}  // namespace coug_mapviz
