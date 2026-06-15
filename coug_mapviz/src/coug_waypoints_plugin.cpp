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
 * @file coug_waypoints_plugin.cpp
 * @brief Implementation of the CougWaypointsPlugin.
 * @author Nelson Durrant
 * @date May 2026
 */

#include <swri_transform_util/frames.h>

#include <QDateTime>
#include <QDir>
#include <QFileDialog>
#include <QMouseEvent>
#include <QPainter>
#include <algorithm>
#include <coug_mapviz/coug_waypoints_plugin.hpp>
#include <cstdlib>
#include <pluginlib/class_list_macros.hpp>
#include <string>
#include <vector>

PLUGINLIB_EXPORT_CLASS(coug_mapviz::CougWaypointsPlugin, mapviz::MapvizPlugin)

namespace coug_mapviz {

static constexpr double kHitRadiusPx = 15.0;
static constexpr double kClickMaxDistPx = 5.0;
static constexpr qint64 kClickMaxDurationMs = 500;

CougWaypointsPlugin::CougWaypointsPlugin()
    : MapvizPlugin(),
      ui_(),
      config_widget_(new QWidget()),
      map_canvas_(nullptr),
      selected_point_(-1),
      dragged_point_(-1),
      mouse_down_time_(0) {
  ui_.setupUi(config_widget_);

  // --- Widget Styling ---
  QPalette p(config_widget_->palette());
  p.setColor(QPalette::Window, Qt::white);
  config_widget_->setPalette(p);
  QPalette p3(ui_.status->palette());
  p3.setColor(QPalette::Text, Qt::darkGreen);
  ui_.status->setPalette(p3);

  // --- Mission Editing ---
  QObject::connect(ui_.agent_selector, SIGNAL(currentTextChanged(const QString&)), this,
                   SLOT(AgentChanged(const QString&)));
  QObject::connect(ui_.publish, SIGNAL(clicked()), this, SLOT(PublishWaypoints()));
  QObject::connect(ui_.clear, SIGNAL(clicked()), this, SLOT(Clear()));

  // --- Mission Control ---
  QObject::connect(ui_.start, SIGNAL(clicked()), this, SLOT(Start()));
  QObject::connect(ui_.stop, SIGNAL(clicked()), this, SLOT(Stop()));
  QObject::connect(ui_.surface, SIGNAL(clicked()), this, SLOT(Surface()));
  QObject::connect(ui_.home, SIGNAL(clicked()), this, SLOT(Home()));

  // --- Mission File I/O ---
  QObject::connect(ui_.save, SIGNAL(clicked()), this, SLOT(SaveWaypoints()));
  QObject::connect(ui_.load, SIGNAL(clicked()), this, SLOT(LoadWaypoints()));

  // --- Waypoint Editors ---
  QObject::connect(this, SIGNAL(VisibleChanged(bool)), this, SLOT(VisibilityChanged(bool)));
  QObject::connect(ui_.lat_editor, SIGNAL(valueChanged(double)), this, SLOT(LatChanged(double)));
  QObject::connect(ui_.lon_editor, SIGNAL(valueChanged(double)), this, SLOT(LonChanged(double)));
  QObject::connect(ui_.depth_editor, SIGNAL(valueChanged(double)), this,
                   SLOT(DepthChanged(double)));
  QObject::connect(ui_.speed_editor, SIGNAL(valueChanged(double)), this,
                   SLOT(SpeedChanged(double)));
  QObject::connect(ui_.altitude_mode, SIGNAL(toggled(bool)), this, SLOT(AltitudeModeChanged(bool)));
}

CougWaypointsPlugin::~CougWaypointsPlugin() {
  if (map_canvas_) {
    map_canvas_->removeEventFilter(this);
  }
}

bool CougWaypointsPlugin::Initialize(QGLWidget* canvas) {
  map_canvas_ = dynamic_cast<mapviz::MapCanvas*>(canvas);
  map_canvas_->installEventFilter(this);

  std::string waypoint_topic, waypoints_map_topic;
  std::string start_service, stop_service, surface_service, home_service;
  if (!node_->has_parameter("agent_namespaces")) {
    node_->declare_parameter("agent_namespaces", std::vector<std::string>{});
  }
  if (!node_->has_parameter("waypoint_topic")) {
    node_->declare_parameter("waypoint_topic", std::string("waypoints"));
  }
  if (!node_->has_parameter("waypoints_map_topic")) {
    node_->declare_parameter("waypoints_map_topic", std::string("waypoints_map"));
  }
  if (!node_->has_parameter("start_service")) {
    node_->declare_parameter("start_service", std::string("base/start"));
  }
  if (!node_->has_parameter("stop_service")) {
    node_->declare_parameter("stop_service", std::string("base/stop"));
  }
  if (!node_->has_parameter("surface_service")) {
    node_->declare_parameter("surface_service", std::string("base/surface"));
  }
  if (!node_->has_parameter("home_service")) {
    node_->declare_parameter("home_service", std::string("base/home"));
  }
  node_->get_parameter("agent_namespaces", agent_namespaces_);
  node_->get_parameter("waypoint_topic", waypoint_topic);
  node_->get_parameter("waypoints_map_topic", waypoints_map_topic);
  node_->get_parameter("start_service", start_service);
  node_->get_parameter("stop_service", stop_service);
  node_->get_parameter("surface_service", surface_service);
  node_->get_parameter("home_service", home_service);
  const std::map<std::string, std::string> services = {
      {"start", start_service},
      {"stop", stop_service},
      {"surface", surface_service},
      {"home", home_service},
  };
  for (const auto& ns : agent_namespaces_) {
    ui_.agent_selector->addItem(QString::fromStdString(ns));
  }

  comms_.initialize(node_, tf_manager_, agent_namespaces_, waypoint_topic, waypoints_map_topic,
                    services, [this](CougCommsClient::Status level, const std::string& msg) {
                      switch (level) {
                        case CougCommsClient::Status::kInfo:
                          PrintInfo(msg);
                          break;
                        case CougCommsClient::Status::kWarning:
                          PrintWarning(msg);
                          break;
                        case CougCommsClient::Status::kError:
                          PrintError(msg);
                          break;
                      }
                    });

  initialized_ = true;
  return true;
}

void CougWaypointsPlugin::VisibilityChanged(bool visible) {
  if (visible) {
    map_canvas_->installEventFilter(this);
  } else {
    map_canvas_->removeEventFilter(this);
  }
}

void CougWaypointsPlugin::deselectWaypoint() {
  selected_point_ = -1;
  ui_.lat_editor->setEnabled(false);
  ui_.lon_editor->setEnabled(false);
  ui_.depth_editor->setEnabled(false);
  ui_.speed_editor->setEnabled(false);
  ui_.altitude_mode->blockSignals(true);
  ui_.altitude_mode->setChecked(false);
  ui_.altitude_mode->setEnabled(false);
  ui_.altitude_mode->blockSignals(false);
  ui_.depth_editor->setMinimum(-9999.99);
  ui_.depth_editor->setMaximum(0.0);
}

void CougWaypointsPlugin::populateEditors(const CougWaypoint& wp) {
  // Positive altitude means altitude mode (above seafloor); negative means depth.
  bool is_altitude = wp.position.altitude > 0.0;

  auto set_value = [](QDoubleSpinBox* editor, double value) {
    editor->setEnabled(true);
    editor->blockSignals(true);
    editor->setValue(value);
    editor->blockSignals(false);
  };

  set_value(ui_.lat_editor, wp.position.latitude);
  set_value(ui_.lon_editor, wp.position.longitude);

  ui_.altitude_mode->blockSignals(true);
  ui_.altitude_mode->setChecked(is_altitude);
  ui_.altitude_mode->setEnabled(true);
  ui_.altitude_mode->blockSignals(false);

  if (is_altitude) {
    ui_.depth_editor->setMinimum(0.0);
    ui_.depth_editor->setMaximum(9999.99);
  } else {
    ui_.depth_editor->setMinimum(-9999.99);
    ui_.depth_editor->setMaximum(0.0);
  }
  set_value(ui_.depth_editor, wp.position.altitude);
  set_value(ui_.speed_editor, wp.speed_rpm);
}

void CougWaypointsPlugin::AgentChanged(const QString& text) {
  current_agent_ = text.toStdString();
  deselectWaypoint();

  auto wps = manager_.getWaypoints(current_agent_);
  if (wps.empty()) {
    PrintInfo("Click to add waypoints");
  } else {
    PrintInfo(current_agent_ + " (" + std::to_string(wps.size()) + " waypoints)");
  }
}

void CougWaypointsPlugin::callService(const std::string& cmd) {
  if (ui_.apply_all->isChecked()) {
    if (agent_namespaces_.empty()) {
      PrintError("No agents configured");
      return;
    }
    comms_.callService(cmd, agent_namespaces_, true);
  } else if (!current_agent_.empty()) {
    comms_.callService(cmd, {current_agent_}, false);
  } else {
    PrintError("No agent selected");
  }
}

void CougWaypointsPlugin::PublishWaypoints() {
  if (ui_.apply_all->isChecked()) {
    publishAll();
  } else if (!current_agent_.empty()) {
    auto wps = manager_.getWaypoints(current_agent_);
    comms_.publishWaypoints(current_agent_, wps, target_frame_);

    if (wps.empty()) {
      PrintWarning("Mission cleared");
    } else {
      PrintInfo("Published " + std::to_string(wps.size()) + " waypoint(s)");
    }
  } else {
    PrintError("No agent selected");
  }
}

void CougWaypointsPlugin::publishAll() {
  int count = 0;
  bool any_waypoints = false;
  for (const auto& [agent, wps] : manager_.getAllWaypoints()) {
    comms_.publishWaypoints(agent, wps, target_frame_);
    if (!wps.empty()) any_waypoints = true;
    count++;
  }
  if (!any_waypoints) {
    PrintWarning("Mission cleared");
  } else {
    PrintInfo("Published to " + std::to_string(count) + " agent(s)");
  }
}

bool CougWaypointsPlugin::isAgentKnown(const std::string& agent) {
  return std::find(agent_namespaces_.begin(), agent_namespaces_.end(), agent) !=
         agent_namespaces_.end();
}

void CougWaypointsPlugin::Clear() {
  if (ui_.apply_all->isChecked()) {
    manager_.clearAllWaypoints();
  } else if (!current_agent_.empty()) {
    manager_.clearWaypoints(current_agent_);
  }

  ui_.lat_editor->blockSignals(true);
  ui_.lat_editor->setValue(0.0);
  ui_.lat_editor->blockSignals(false);
  ui_.lon_editor->blockSignals(true);
  ui_.lon_editor->setValue(0.0);
  ui_.lon_editor->blockSignals(false);
  ui_.depth_editor->setValue(0.0);
  dragged_point_ = -1;
  deselectWaypoint();
  PrintInfo("Waypoints cleared");
}

void CougWaypointsPlugin::SaveWaypoints() {
  const char* overlay_ws = std::getenv("OVERLAY_WS");
  QString path = overlay_ws
                     ? QString::fromUtf8(overlay_ws) + "/src/coug_mapviz/coug_mapviz/missions"
                     : QString::fromUtf8(std::getenv("HOME"));
  QDir dir(path);
  if (!dir.exists()) {
    dir.mkpath(".");
  }

  QString filename =
      QFileDialog::getSaveFileName(config_widget_, "Save Mission", path, "JSON Files (*.json)");
  if (filename.isEmpty()) {
    return;
  }
  if (!filename.endsWith(".json", Qt::CaseInsensitive)) {
    filename += ".json";
  }

  std::string agent_to_save = "";
  if (!ui_.apply_all->isChecked()) {
    if (current_agent_.empty()) {
      PrintError("No agent selected");
      return;
    }
    agent_to_save = current_agent_;
  }

  if (manager_.saveToFile(filename.toStdString(), agent_to_save)) {
    if (agent_to_save.empty()) {
      PrintInfo("Saved " + std::to_string(manager_.getAllWaypoints().size()) + " agent(s)");
    } else {
      PrintInfo("Saved: " + agent_to_save);
    }
  } else {
    PrintError("Failed to save");
  }
}

void CougWaypointsPlugin::LoadWaypoints() {
  const char* overlay_ws = std::getenv("OVERLAY_WS");
  QString path = overlay_ws
                     ? QString::fromUtf8(overlay_ws) + "/src/coug_mapviz/coug_mapviz/missions"
                     : QString::fromUtf8(std::getenv("HOME"));
  QString filename =
      QFileDialog::getOpenFileName(config_widget_, "Load Mission", path, "JSON Files (*.json)");
  if (filename.isEmpty()) {
    return;
  }

  std::string agent_to_load = "";
  if (!ui_.apply_all->isChecked()) {
    if (current_agent_.empty()) {
      PrintError("No agent selected");
      return;
    }
    agent_to_load = current_agent_;
  }

  if (manager_.loadFromFile(filename.toStdString(), agent_to_load)) {
    std::vector<std::string> unknown_agents;
    for (const auto& [agent, wps] : manager_.getAllWaypoints()) {
      (void)wps;
      if (!isAgentKnown(agent)) {
        unknown_agents.push_back(agent);
      }
    }

    for (const auto& agent : unknown_agents) {
      manager_.removeAgent(agent);
    }

    AgentChanged(QString::fromStdString(current_agent_));
    if (agent_to_load.empty()) {
      PrintInfo("Loaded " + std::to_string(manager_.getAllWaypoints().size()) + " agent(s)");
    } else {
      auto wps = manager_.getWaypoints(agent_to_load);
      PrintInfo("Loaded " + agent_to_load + " (" + std::to_string(wps.size()) + " waypoint(s))");
    }
  } else {
    PrintError("Failed to load");
  }
}

void CougWaypointsPlugin::Draw(double x, double y, double scale) {
  (void)x;
  (void)y;
  (void)scale;

  swri_transform_util::Transform transform;
  if (!tf_manager_->GetTransform(target_frame_, swri_transform_util::_wgs84_frame, transform)) {
    return;
  }

  glLineWidth(2);

  for (const auto& [agent, wps] : manager_.getAllWaypoints()) {
    if (agent != current_agent_ && isAgentKnown(agent)) {
      drawPath(wps, transform);
    }
  }
}

QPointF CougWaypointsPlugin::waypointToFixedFrame(const CougWaypoint& wp,
                                                  const swri_transform_util::Transform& transform) {
  tf2::Vector3 point(wp.position.longitude, wp.position.latitude, 0.0);
  point = transform * point;
  return QPointF(point.x(), point.y());
}

QPointF CougWaypointsPlugin::waypointToMapGl(const CougWaypoint& wp,
                                             const swri_transform_util::Transform& transform) {
  return map_canvas_->FixedFrameToMapGlCoord(waypointToFixedFrame(wp, transform));
}

bool CougWaypointsPlugin::screenToGeo(const QPointF& screen_point,
                                      geographic_msgs::msg::GeoPoint& geo) {
  swri_transform_util::Transform transform;
  if (!tf_manager_->GetTransform(swri_transform_util::_wgs84_frame, target_frame_, transform)) {
    return false;
  }
  QPointF fixed = map_canvas_->MapGlCoordToFixedFrame(screen_point);
  tf2::Vector3 position(fixed.x(), fixed.y(), 0.0);
  position = transform * position;
  geo.longitude = position.x();
  geo.latitude = position.y();
  return true;
}

void CougWaypointsPlugin::drawPath(const std::vector<CougWaypoint>& wps,
                                   const swri_transform_util::Transform& transform) {
  std::vector<QPointF> points;
  points.reserve(wps.size());
  for (const auto& cwp : wps) {
    points.push_back(waypointToFixedFrame(cwp, transform));
  }

  glColor4f(1.0, 1.0, 1.0, 1.0);
  glBegin(GL_LINE_STRIP);
  for (const auto& p : points) {
    glVertex2d(p.x(), p.y());
  }
  glEnd();

  glColor4f(0.5, 0.5, 0.5, 1.0);
  glPointSize(20);
  glBegin(GL_POINTS);
  for (const auto& p : points) {
    glVertex2d(p.x(), p.y());
  }
  glEnd();
}

void CougWaypointsPlugin::Paint(QPainter* painter, double x, double y, double scale) {
  (void)x;
  (void)y;
  (void)scale;

  swri_transform_util::Transform transform;
  if (!tf_manager_->GetTransform(target_frame_, swri_transform_util::_wgs84_frame, transform)) {
    return;
  }

  painter->save();
  painter->resetTransform();

  painter->setFont(QFont("DejaVu Sans Mono", 10, QFont::Bold));
  for (const auto& [agent, wps] : manager_.getAllWaypoints()) {
    if (agent != current_agent_ && isAgentKnown(agent)) {
      paintLabels(painter, wps, transform, QColor(255, 255, 255, 200));
    }
  }

  if (!current_agent_.empty()) {
    auto wps = manager_.getWaypoints(current_agent_);
    if (!wps.empty()) {
      paintPath(painter, wps, QColor(Qt::blue), transform, selected_point_);
      paintLabels(painter, wps, transform, Qt::white);
    }
  }
  painter->restore();
}

void CougWaypointsPlugin::paintPath(QPainter* painter, const std::vector<CougWaypoint>& wps,
                                    const QColor& color,
                                    const swri_transform_util::Transform& transform,
                                    int selected_index) {
  QVector<QPointF> points;
  for (const auto& cwp : wps) {
    points.push_back(waypointToMapGl(cwp, transform));
  }

  QPen pen(color, 2);
  painter->setPen(pen);
  painter->drawPolyline(points);
  for (int i = 0; i < points.size(); ++i) {
    if (i == selected_index) {
      painter->setPen(QPen(Qt::yellow, 20, Qt::SolidLine, Qt::RoundCap));
      painter->drawPoint(points[i]);
    } else if (color == Qt::blue) {
      painter->setPen(QPen(Qt::cyan, 20, Qt::SolidLine, Qt::RoundCap));
      painter->drawPoint(points[i]);
    } else {
      painter->setPen(QPen(Qt::gray, 20, Qt::SolidLine, Qt::RoundCap));
      painter->drawPoint(points[i]);
    }
  }
}

void CougWaypointsPlugin::paintLabels(QPainter* painter, const std::vector<CougWaypoint>& wps,
                                      const swri_transform_util::Transform& transform,
                                      const QColor& color) {
  for (size_t i = 0; i < wps.size(); i++) {
    QPointF gl_point = waypointToMapGl(wps[i], transform);

    painter->setPen(QPen(color));

    QPointF depth_text_corner(gl_point.x() - 50, gl_point.y() + 15);
    QRectF depth_text_rect(depth_text_corner, QSizeF(100, 20));
    QString depth_text = wps[i].position.altitude > 0.0
                             ? "ALT " + QString::number(wps[i].position.altitude, 'f', 1) + "m"
                             : QString::number(wps[i].position.altitude, 'f', 1) + "m";
    painter->drawText(depth_text_rect, Qt::AlignHCenter | Qt::AlignTop, depth_text);

    QPointF speed_text_corner(gl_point.x() - 50, gl_point.y() + 33);
    QRectF speed_text_rect(speed_text_corner, QSizeF(100, 20));
    QString speed_text = QString::number(wps[i].speed_rpm, 'f', 0) + " RPM";
    painter->drawText(speed_text_rect, Qt::AlignHCenter | Qt::AlignTop, speed_text);

    painter->setPen(QPen(color == Qt::white ? Qt::black : color));
    QPointF num_corner(gl_point.x() - 20, gl_point.y() - 20);
    QRectF num_rect(num_corner, QSizeF(40, 40));
    painter->drawText(num_rect, Qt::AlignHCenter | Qt::AlignVCenter, QString::number(i + 1));
  }
}

void CougWaypointsPlugin::LatChanged(double value) {
  if (selected_point_ < 0) return;
  if (auto* wp = manager_.getWaypointMutable(current_agent_, selected_point_)) {
    wp->position.latitude = value;
    map_canvas_->update();
  }
}

void CougWaypointsPlugin::LonChanged(double value) {
  if (selected_point_ < 0) return;
  if (auto* wp = manager_.getWaypointMutable(current_agent_, selected_point_)) {
    wp->position.longitude = value;
    map_canvas_->update();
  }
}

void CougWaypointsPlugin::DepthChanged(double value) {
  if (selected_point_ < 0) return;
  if (auto* wp = manager_.getWaypointMutable(current_agent_, selected_point_)) {
    wp->position.altitude = value;
    map_canvas_->update();
  }
}

void CougWaypointsPlugin::SpeedChanged(double value) {
  if (selected_point_ < 0) return;
  if (auto* wp = manager_.getWaypointMutable(current_agent_, selected_point_)) {
    wp->speed_rpm = value;
    map_canvas_->update();
  }
}

void CougWaypointsPlugin::AltitudeModeChanged(bool checked) {
  if (selected_point_ < 0) return;
  auto* wp = manager_.getWaypointMutable(current_agent_, selected_point_);
  if (!wp) return;

  if (checked) {
    ui_.depth_editor->setMinimum(0.0);
    ui_.depth_editor->setMaximum(9999.99);
  } else {
    ui_.depth_editor->setMinimum(-9999.99);
    ui_.depth_editor->setMaximum(0.0);
  }
  double new_val = checked ? std::abs(wp->position.altitude) : -std::abs(wp->position.altitude);
  wp->position.altitude = new_val;
  ui_.depth_editor->blockSignals(true);
  ui_.depth_editor->setValue(new_val);
  ui_.depth_editor->blockSignals(false);
  map_canvas_->update();
}

bool CougWaypointsPlugin::eventFilter(QObject* object, QEvent* event) {
  (void)object;
  switch (event->type()) {
    case QEvent::MouseButtonPress:
      return handleMousePress(dynamic_cast<QMouseEvent*>(event));
    case QEvent::MouseButtonRelease:
      return handleMouseRelease(dynamic_cast<QMouseEvent*>(event));
    case QEvent::MouseMove:
      return handleMouseMove(dynamic_cast<QMouseEvent*>(event));
    default:
      return false;
  }
}

int CougWaypointsPlugin::getClosestPoint(const QPointF& point, double& distance) {
  swri_transform_util::Transform transform;
  if (!tf_manager_->GetTransform(target_frame_, swri_transform_util::_wgs84_frame, transform)) {
    return -1;
  }

  int closest = -1;
  distance = std::numeric_limits<double>::max();
  auto wps = manager_.getWaypoints(current_agent_);

  for (size_t i = 0; i < wps.size(); i++) {
    QPointF transformed = waypointToMapGl(wps[i], transform);

    double d = QLineF(transformed, point).length();
    if (d < distance) {
      distance = d;
      closest = static_cast<int>(i);
    }
  }
  return closest;
}

bool CougWaypointsPlugin::handleMousePress(QMouseEvent* event) {
  if (current_agent_.empty()) {
    return false;
  }

  dragged_point_ = -1;
  double distance = 0.0;
  int closest_point = getClosestPoint(event->localPos(), distance);

  if (event->button() == Qt::LeftButton) {
    mouse_down_pos_ = event->localPos();
    mouse_down_time_ = QDateTime::currentMSecsSinceEpoch();

    if (distance < kHitRadiusPx) {
      dragged_point_ = closest_point;
      return true;
    }
  } else if (event->button() == Qt::RightButton) {
    if (distance < kHitRadiusPx) {
      manager_.removeWaypoint(current_agent_, closest_point);

      if (selected_point_ == closest_point) {
        deselectWaypoint();
      }
      return true;
    }
  }
  return false;
}

bool CougWaypointsPlugin::handleMouseRelease(QMouseEvent* event) {
  if (current_agent_.empty()) {
    return false;
  }

  qreal distance = QLineF(mouse_down_pos_, event->localPos()).length();
  qint64 msecsDiff = QDateTime::currentMSecsSinceEpoch() - mouse_down_time_;

  // A click (tap) barely moves and is brief; anything else is a drag.
  bool is_click = distance <= kClickMaxDistPx && msecsDiff < kClickMaxDurationMs;

  if (dragged_point_ != -1) {
    if (is_click) {
      selected_point_ = dragged_point_;
      auto wps = manager_.getWaypoints(current_agent_);
      populateEditors(wps[selected_point_]);
    }
    dragged_point_ = -1;
    return true;
  }

  if (event->button() == Qt::LeftButton && is_click) {
    geographic_msgs::msg::GeoPoint geo;
    if (screenToGeo(event->localPos(), geo)) {
      CougWaypoint cwp;
      cwp.position = geo;
      cwp.position.altitude = ui_.depth_editor->value();
      cwp.speed_rpm = ui_.speed_editor->value();

      manager_.addWaypoint(current_agent_, cwp);
    }
  }

  dragged_point_ = -1;
  return false;
}

bool CougWaypointsPlugin::handleMouseMove(QMouseEvent* event) {
  if (current_agent_.empty()) {
    return false;
  }

  if (dragged_point_ >= 0) {
    if (selected_point_ != -1) {
      deselectWaypoint();
    }

    geographic_msgs::msg::GeoPoint geo;
    if (screenToGeo(event->localPos(), geo)) {
      if (auto* wp = manager_.getWaypointMutable(current_agent_, dragged_point_)) {
        wp->position.longitude = geo.longitude;
        wp->position.latitude = geo.latitude;
      }
    }
    return true;
  }
  return false;
}

QWidget* CougWaypointsPlugin::GetConfigWidget(QWidget* parent) {
  config_widget_->setParent(parent);
  return config_widget_;
}

void CougWaypointsPlugin::PrintError(const std::string& message) {
  PrintErrorHelper(ui_.status, message, 1.0);
}

void CougWaypointsPlugin::PrintInfo(const std::string& message) {
  PrintInfoHelper(ui_.status, message, 1.0);
}

void CougWaypointsPlugin::PrintWarning(const std::string& message) {
  PrintWarningHelper(ui_.status, message, 1.0);
}

}  // namespace coug_mapviz
