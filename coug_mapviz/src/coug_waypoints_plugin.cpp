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

CougWaypointsPlugin::CougWaypointsPlugin()
    : MapvizPlugin(),
      ui_(),
      config_widget_(new QWidget()),
      map_canvas_(nullptr),
      selected_point_(-1),
      dragged_point_(-1),
      is_mouse_down_(false),
      mouse_down_time_(0) {
  ui_.setupUi(config_widget_);

  QPalette p(config_widget_->palette());
  p.setColor(QPalette::Window, Qt::white);
  config_widget_->setPalette(p);
  QPalette p3(ui_.status->palette());
  p3.setColor(QPalette::Text, Qt::darkGreen);
  ui_.status->setPalette(p3);

  QObject::connect(ui_.agent_selector, SIGNAL(currentTextChanged(const QString&)), this,
                   SLOT(AgentChanged(const QString&)));
  QObject::connect(ui_.publish, SIGNAL(clicked()), this, SLOT(PublishWaypoints()));
  QObject::connect(ui_.clear, SIGNAL(clicked()), this, SLOT(Clear()));

  QObject::connect(ui_.start, SIGNAL(clicked()), this, SLOT(Start()));
  QObject::connect(ui_.stop, SIGNAL(clicked()), this, SLOT(Stop()));
  QObject::connect(ui_.surface, SIGNAL(clicked()), this, SLOT(Surface()));
  QObject::connect(ui_.home, SIGNAL(clicked()), this, SLOT(Home()));

  QObject::connect(ui_.save, SIGNAL(clicked()), this, SLOT(SaveWaypoints()));
  QObject::connect(ui_.load, SIGNAL(clicked()), this, SLOT(LoadWaypoints()));

  QObject::connect(this, SIGNAL(VisibleChanged(bool)), this, SLOT(VisibilityChanged(bool)));
  QObject::connect(ui_.depth_editor, SIGNAL(valueChanged(double)), this,
                   SLOT(DepthChanged(double)));
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

  try {
    node_->declare_parameter("agent_namespaces", std::vector<std::string>{});
    node_->declare_parameter("waypoint_topic", std::string("waypoints"));
    node_->declare_parameter("waypoints_map_topic", std::string("waypoints_map"));
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&) {
  }
  node_->get_parameter("agent_namespaces", agent_namespaces_);
  node_->get_parameter("waypoint_topic", waypoint_topic_);
  node_->get_parameter("waypoints_map_topic", waypoints_map_topic_);
  for (const auto& ns : agent_namespaces_) {
    ui_.agent_selector->addItem(QString::fromStdString(ns));
    publishers_[ns + "/" + waypoint_topic_] =
        node_->create_publisher<coug_interfaces::msg::WayPointList>(ns + "/" + waypoint_topic_,
                                                                    rclcpp::SystemDefaultsQoS());
    map_publishers_[ns + "/" + waypoints_map_topic_] =
        node_->create_publisher<geometry_msgs::msg::PoseArray>(ns + "/" + waypoints_map_topic_,
                                                               rclcpp::SystemDefaultsQoS());
    for (const auto& cmd : {"start", "stop", "surface", "home"}) {
      std::string service_name = ns + "/" + cmd;
      clients_[service_name] = node_->create_client<std_srvs::srv::Trigger>(service_name);
    }
  }

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

void CougWaypointsPlugin::AgentChanged(const QString& text) {
  current_agent_ = text.toStdString();
  selected_point_ = -1;
  ui_.depth_editor->setEnabled(false);
  ui_.altitude_mode->blockSignals(true);
  ui_.altitude_mode->setChecked(false);
  ui_.altitude_mode->setEnabled(false);
  ui_.altitude_mode->blockSignals(false);
  ui_.depth_editor->setMinimum(-9999.99);
  ui_.depth_editor->setMaximum(0.0);

  auto wps = manager_.getWaypoints(current_agent_);
  if (wps.empty()) {
    PrintInfo("Click to add waypoints");
  } else {
    PrintInfo(current_agent_ + " (" + std::to_string(wps.size()) + " waypoints)");
  }
}

void CougWaypointsPlugin::recordResult(std::shared_ptr<DispatchState> state, bool success,
                                       const std::string& agent) {
  std::lock_guard<std::mutex> lock(state->mutex);
  if (success)
    state->succeeded++;
  else
    state->failed.push_back(agent);
  if (++state->responded < state->total) return;

  int s = state->succeeded, t = state->total;
  std::string prefix = "[" + state->cmd + "] ";
  if (s == t) {
    PrintInfo(prefix + "All " + std::to_string(t) + " agent(s) confirmed");
  } else {
    std::string failed_str;
    for (const auto& f : state->failed) failed_str += " " + f;
    std::string msg =
        prefix + std::to_string(s) + "/" + std::to_string(t) + " confirmed; failed:" + failed_str;
    if (s == 0)
      PrintError(msg);
    else
      PrintWarning(msg);
  }
}

void CougWaypointsPlugin::callTrigger(const std::string& service_name, const std::string& agent,
                                      std::shared_ptr<DispatchState> state) {
  if (clients_.find(service_name) == clients_.end()) {
    if (state)
      recordResult(state, false, agent);
    else
      PrintError("Unknown service: " + service_name);
    return;
  }
  auto& client = clients_[service_name];
  if (!client->service_is_ready()) {
    if (state)
      recordResult(state, false, agent);
    else
      PrintError("Service not available: " + service_name);
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  client->async_send_request(
      request, [this, agent, state,
                service_name](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        auto response = future.get();
        if (state) {
          recordResult(state, response->success, agent);
        } else {
          std::string prefix = "[" + service_name.substr(service_name.rfind('/') + 1) + "] ";
          if (response->success)
            PrintInfo(prefix + response->message);
          else
            PrintWarning(prefix + response->message);
        }
      });
}

void CougWaypointsPlugin::dispatchCommand(const std::string& cmd) {
  if (ui_.apply_all->isChecked()) {
    if (agent_namespaces_.empty()) {
      PrintError("No agents configured");
      return;
    }
    auto state = std::make_shared<DispatchState>();
    state->total = static_cast<int>(agent_namespaces_.size());
    state->cmd = cmd;
    for (const auto& ns : agent_namespaces_) callTrigger(ns + "/" + cmd, ns, state);
  } else if (!current_agent_.empty()) {
    callTrigger(current_agent_ + "/" + cmd);
  } else {
    PrintError("No agent selected");
  }
}

void CougWaypointsPlugin::PublishWaypoints() {
  if (ui_.apply_all->isChecked()) {
    PublishAll();
  } else if (!current_agent_.empty()) {
    auto wps = manager_.getWaypoints(current_agent_);
    PublishAgent(current_agent_, wps);

    if (wps.empty()) {
      PrintWarning("Mission cleared");
    } else {
      PrintInfo("Published " + std::to_string(wps.size()) + " waypoint(s)");
    }
  } else {
    PrintError("No agent selected");
  }
}

void CougWaypointsPlugin::PublishAll() {
  int count = 0;
  bool any_waypoints = false;
  for (const auto& [agent, wps] : manager_.getAllWaypoints()) {
    PublishAgent(agent, wps);
    if (!wps.empty()) any_waypoints = true;
    count++;
  }
  if (!any_waypoints) {
    PrintWarning("Mission cleared");
  } else {
    PrintInfo("Published to " + std::to_string(count) + " agent(s)");
  }
}

void CougWaypointsPlugin::PublishAgent(const std::string& agent,
                                       const std::vector<geographic_msgs::msg::GeoPoint>& wps) {
  std::string topic = agent + "/" + waypoint_topic_;
  coug_interfaces::msg::WayPointList waypoint_list;
  waypoint_list.header.frame_id = swri_transform_util::_wgs84_frame;
  waypoint_list.header.stamp = node_->now();
  for (const auto& gp : wps) {
    geographic_msgs::msg::WayPoint wp;
    wp.position = gp;
    waypoint_list.waypoints.push_back(wp);
  }

  publishers_[topic]->publish(waypoint_list);

  std::string map_topic = agent + "/" + waypoints_map_topic_;
  swri_transform_util::Transform transform;
  if (tf_manager_->GetTransform(target_frame_, swri_transform_util::_wgs84_frame, transform)) {
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.frame_id = target_frame_;
    pose_array.header.stamp = node_->now();
    for (const auto& gp : wps) {
      tf2::Vector3 point(gp.longitude, gp.latitude, 0.0);
      point = transform * point;
      geometry_msgs::msg::Pose pose;
      pose.position.x = point.x();
      pose.position.y = point.y();
      pose.position.z = gp.altitude;
      pose.orientation.w = 1.0;
      pose_array.poses.push_back(pose);
    }
    map_publishers_[map_topic]->publish(pose_array);
  }
}

bool CougWaypointsPlugin::IsAgentKnown(const std::string& agent) {
  return std::find(agent_namespaces_.begin(), agent_namespaces_.end(), agent) !=
         agent_namespaces_.end();
}

void CougWaypointsPlugin::Clear() {
  if (ui_.apply_all->isChecked()) {
    manager_.clearAllWaypoints();
  } else if (!current_agent_.empty()) {
    manager_.clearWaypoints(current_agent_);
  }

  ui_.depth_editor->setValue(0.0);
  selected_point_ = -1;
  dragged_point_ = -1;
  ui_.depth_editor->setEnabled(false);
  ui_.altitude_mode->blockSignals(true);
  ui_.altitude_mode->setChecked(false);
  ui_.altitude_mode->setEnabled(false);
  ui_.altitude_mode->blockSignals(false);
  ui_.depth_editor->setMinimum(-9999.99);
  ui_.depth_editor->setMaximum(0.0);
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
      if (!IsAgentKnown(agent)) {
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
    if (agent != current_agent_ && IsAgentKnown(agent)) {
      DrawPath(wps, transform);
    }
  }
}

void CougWaypointsPlugin::DrawPath(const std::vector<geographic_msgs::msg::GeoPoint>& wps,
                                   const swri_transform_util::Transform& transform) {
  glColor4f(1.0, 1.0, 1.0, 1.0);
  glBegin(GL_LINE_STRIP);
  for (const auto& wp : wps) {
    tf2::Vector3 point(wp.longitude, wp.latitude, 0);
    point = transform * point;
    glVertex2d(point.x(), point.y());
  }
  glEnd();

  glColor4f(0.5, 0.5, 0.5, 1.0);
  glPointSize(20);
  glBegin(GL_POINTS);
  for (const auto& wp : wps) {
    tf2::Vector3 point(wp.longitude, wp.latitude, 0);
    point = transform * point;
    glVertex2d(point.x(), point.y());
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
    if (agent != current_agent_ && IsAgentKnown(agent)) {
      PaintLabels(painter, wps, transform, QColor(255, 255, 255, 200));
    }
  }

  if (!current_agent_.empty()) {
    auto wps = manager_.getWaypoints(current_agent_);
    if (!wps.empty()) {
      PaintPath(painter, wps, QColor(Qt::blue), transform, selected_point_);
      PaintLabels(painter, wps, transform, Qt::white);
    }
  }
  painter->restore();
}

void CougWaypointsPlugin::PaintPath(QPainter* painter,
                                    const std::vector<geographic_msgs::msg::GeoPoint>& wps,
                                    const QColor& color,
                                    const swri_transform_util::Transform& transform,
                                    int selected_index) {
  QVector<QPointF> points;
  for (const auto& wp : wps) {
    tf2::Vector3 point(wp.longitude, wp.latitude, 0);
    point = transform * point;
    points.push_back(map_canvas_->FixedFrameToMapGlCoord(QPointF(point.x(), point.y())));
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

void CougWaypointsPlugin::PaintLabels(QPainter* painter,
                                      const std::vector<geographic_msgs::msg::GeoPoint>& wps,
                                      const swri_transform_util::Transform& transform,
                                      const QColor& color) {
  for (size_t i = 0; i < wps.size(); i++) {
    tf2::Vector3 point(wps[i].longitude, wps[i].latitude, 0);
    point = transform * point;
    QPointF gl_point = map_canvas_->FixedFrameToMapGlCoord(QPointF(point.x(), point.y()));

    painter->setPen(QPen(color));

    QPointF depth_text_corner(gl_point.x() - 50, gl_point.y() + 15);
    QRectF depth_text_rect(depth_text_corner, QSizeF(100, 20));
    QString depth_text = wps[i].altitude > 0.0
                             ? "ALT " + QString::number(wps[i].altitude, 'f', 1) + "m"
                             : QString::number(wps[i].altitude, 'f', 1) + "m";
    painter->drawText(depth_text_rect, Qt::AlignHCenter | Qt::AlignTop, depth_text);

    painter->setPen(QPen(color == Qt::white ? Qt::black : color));
    QPointF num_corner(gl_point.x() - 20, gl_point.y() - 20);
    QRectF num_rect(num_corner, QSizeF(40, 40));
    painter->drawText(num_rect, Qt::AlignHCenter | Qt::AlignVCenter, QString::number(i + 1));
  }
}

void CougWaypointsPlugin::DepthChanged(double value) {
  if (current_agent_.empty()) {
    return;
  }

  auto wps = manager_.getWaypoints(current_agent_);
  if (selected_point_ >= 0 && static_cast<size_t>(selected_point_) < wps.size()) {
    wps[selected_point_].altitude = value;
    manager_.setWaypoints(current_agent_, wps);
  }
}

void CougWaypointsPlugin::AltitudeModeChanged(bool checked) {
  if (current_agent_.empty()) {
    return;
  }

  auto wps = manager_.getWaypoints(current_agent_);
  if (selected_point_ < 0 || static_cast<size_t>(selected_point_) >= wps.size()) {
    return;
  }

  double current_val = wps[selected_point_].altitude;
  if (checked) {
    ui_.depth_editor->setMinimum(0.0);
    ui_.depth_editor->setMaximum(9999.99);
    double new_val = std::abs(current_val);
    wps[selected_point_].altitude = new_val;
    ui_.depth_editor->blockSignals(true);
    ui_.depth_editor->setValue(new_val);
    ui_.depth_editor->blockSignals(false);
  } else {
    ui_.depth_editor->setMinimum(-9999.99);
    ui_.depth_editor->setMaximum(0.0);
    double new_val = -std::abs(current_val);
    wps[selected_point_].altitude = new_val;
    ui_.depth_editor->blockSignals(true);
    ui_.depth_editor->setValue(new_val);
    ui_.depth_editor->blockSignals(false);
  }
  manager_.setWaypoints(current_agent_, wps);
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

int CougWaypointsPlugin::GetClosestPoint(const QPointF& point, double& distance) {
  swri_transform_util::Transform transform;
  if (!tf_manager_->GetTransform(target_frame_, swri_transform_util::_wgs84_frame, transform)) {
    return -1;
  }

  int closest = -1;
  distance = std::numeric_limits<double>::max();
  auto wps = manager_.getWaypoints(current_agent_);

  for (size_t i = 0; i < wps.size(); i++) {
    tf2::Vector3 waypoint(wps[i].longitude, wps[i].latitude, 0);
    waypoint = transform * waypoint;
    QPointF transformed = map_canvas_->FixedFrameToMapGlCoord(QPointF(waypoint.x(), waypoint.y()));

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
  int closest_point = GetClosestPoint(event->localPos(), distance);

  if (event->button() == Qt::LeftButton) {
    is_mouse_down_ = true;
    mouse_down_pos_ = event->localPos();
    mouse_down_time_ = QDateTime::currentMSecsSinceEpoch();

    if (distance < 15) {
      dragged_point_ = closest_point;
      return true;
    }
  } else if (event->button() == Qt::RightButton) {
    if (distance < 15) {
      auto wps = manager_.getWaypoints(current_agent_);
      wps.erase(wps.begin() + closest_point);
      manager_.setWaypoints(current_agent_, wps);

      if (selected_point_ == closest_point) {
        selected_point_ = -1;
        ui_.depth_editor->setEnabled(false);
        ui_.altitude_mode->blockSignals(true);
        ui_.altitude_mode->setChecked(false);
        ui_.altitude_mode->setEnabled(false);
        ui_.altitude_mode->blockSignals(false);
        ui_.depth_editor->setMinimum(-9999.99);
        ui_.depth_editor->setMaximum(0.0);
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

  if (dragged_point_ != -1) {
    if (distance <= 5.0 && msecsDiff < 500) {
      selected_point_ = dragged_point_;
      auto wps = manager_.getWaypoints(current_agent_);
      bool is_altitude = wps[selected_point_].altitude > 0.0;

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

      ui_.depth_editor->setEnabled(true);
      ui_.depth_editor->blockSignals(true);
      ui_.depth_editor->setValue(wps[selected_point_].altitude);
      ui_.depth_editor->blockSignals(false);
    }
    dragged_point_ = -1;
    return true;
  }

  if (is_mouse_down_) {
    if (msecsDiff < 500 && distance <= 5.0) {
      QPointF transformed = map_canvas_->MapGlCoordToFixedFrame(event->localPos());
      swri_transform_util::Transform transform;
      if (tf_manager_->GetTransform(swri_transform_util::_wgs84_frame, target_frame_, transform)) {
        tf2::Vector3 position(transformed.x(), transformed.y(), 0.0);
        position = transform * position;

        geographic_msgs::msg::GeoPoint gp;
        gp.longitude = position.x();
        gp.latitude = position.y();
        gp.altitude = ui_.depth_editor->value();

        manager_.addWaypoint(current_agent_, gp);
      }
    }
  }

  is_mouse_down_ = false;
  dragged_point_ = -1;
  return false;
}

bool CougWaypointsPlugin::handleMouseMove(QMouseEvent* event) {
  if (current_agent_.empty()) {
    return false;
  }

  if (dragged_point_ >= 0) {
    if (selected_point_ != -1) {
      selected_point_ = -1;
      ui_.depth_editor->setEnabled(false);
      ui_.altitude_mode->blockSignals(true);
      ui_.altitude_mode->setChecked(false);
      ui_.altitude_mode->setEnabled(false);
      ui_.altitude_mode->blockSignals(false);
      ui_.depth_editor->setMinimum(-9999.99);
      ui_.depth_editor->setMaximum(0.0);
    }

    swri_transform_util::Transform transform;
    if (tf_manager_->GetTransform(swri_transform_util::_wgs84_frame, target_frame_, transform)) {
      QPointF transformed = map_canvas_->MapGlCoordToFixedFrame(event->localPos());
      tf2::Vector3 position(transformed.x(), transformed.y(), 0.0);
      position = transform * position;

      auto wps = manager_.getWaypoints(current_agent_);
      wps[dragged_point_].longitude = position.x();
      wps[dragged_point_].latitude = position.y();
      manager_.setWaypoints(current_agent_, wps);
    }
    return true;
  }
  return false;
}

void CougWaypointsPlugin::LoadConfig(const YAML::Node& node, const std::string& path) {
  (void)node;
  (void)path;
}

void CougWaypointsPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path) {
  (void)emitter;
  (void)path;
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
