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
#include <coug_mapviz/utils/agent_interface.hpp>
#include <coug_mapviz/utils/waypoint_manager.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace coug_mapviz {

class CougWaypointsPlugin : public mapviz::MapvizPlugin {
  Q_OBJECT

 public:
  // --- Lifecycle & Mapviz Interface ---
  CougWaypointsPlugin();
  ~CougWaypointsPlugin() override;

  bool Initialize(QGLWidget* canvas) override;

  void Shutdown() override {}

  void Draw(double x, double y, double scale) override;

  void Paint(QPainter* painter, double x, double y, double scale) override;

  void Transform() override {}

  // --- Configuration ---
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
  // --- Error Handling ---
  void PrintError(const std::string& message) override;

  void PrintInfo(const std::string& message) override;

  void PrintWarning(const std::string& message) override;

  // --- Event Handling ---
  bool eventFilter(QObject* object, QEvent* event) override;

  bool handleMousePress(QMouseEvent* event);

  bool handleMouseRelease(QMouseEvent* event);

  bool handleMouseMove(QMouseEvent* event);

  // --- Thread-Safe Status Updates ---
 Q_SIGNALS:
  void StatusUpdateRequested(int level, const QString& msg);

 public Q_SLOTS:
  void HandleStatusUpdate(int level, const QString& msg);

 protected Q_SLOTS:
  // --- UI Slots ---
  void PublishWaypoints();

  void Clear();

  void SaveWaypoints();

  void LoadWaypoints();

  void AgentChanged(const QString& text);

  // --- Mission Control ---
  void Start() { callService("start"); }

  void Stop() { callService("stop"); }

  void Surface() { callService("surface"); }

  void Home() { callService("home"); }

  void EditorChanged(double value);

  void AltitudeModeChanged(bool checked);

 private:
  // --- Components ---
  Ui::coug_waypoints_config ui_;
  QWidget* config_widget_;
  mapviz::MapCanvas* map_canvas_;

  // --- ROS Interfaces ---
  utils::AgentInterface interface_;
  utils::WaypointManager manager_;
  coug_interfaces::msg::WayPoint default_waypoint_;
  std::string current_agent_;
  std::vector<std::string> agent_namespaces_;

  // --- Interaction State ---
  int selected_point_;
  int dragged_point_;
  QPointF mouse_down_pos_;
  qint64 mouse_down_time_;

  // --- Helpers ---
  void applyDefaultsToEditors();

  void deselectWaypoint();

  void populateEditors(const coug_interfaces::msg::WayPoint& wp);

  void publishAll();

  void callService(const std::string& cmd);

  bool isAgentKnown(const std::string& agent);

  static QPointF wgs84ToMap(const coug_interfaces::msg::WayPoint& wp,
                            const swri_transform_util::Transform& transform);

  QPointF wgs84ToGl(const coug_interfaces::msg::WayPoint& wp,
                    const swri_transform_util::Transform& transform);

  bool glToWgs84(const QPointF& gl_point, geographic_msgs::msg::GeoPoint& geo);

  int getClosestPoint(const QPointF& point, double& distance);

  static void drawPath(const std::vector<coug_interfaces::msg::WayPoint>& wps,
                       const swri_transform_util::Transform& transform);

  static void drawWaypointCircles(const std::vector<coug_interfaces::msg::WayPoint>& wps,
                                  const swri_transform_util::Transform& transform);

  static void drawFilledCircle(double cx, double cy, double radius, float r, float g, float b,
                               float a);

  static void drawCircleOutline(double cx, double cy, double radius, float r, float g, float b,
                                float a);

  void paintLabels(QPainter* painter, const std::vector<coug_interfaces::msg::WayPoint>& wps,
                   const swri_transform_util::Transform& transform, const QColor& color);

  void paintPath(QPainter* painter, const std::vector<coug_interfaces::msg::WayPoint>& wps,
                 const QColor& color, const swri_transform_util::Transform& transform,
                 int selected_index = -1);
};

}  // namespace coug_mapviz
