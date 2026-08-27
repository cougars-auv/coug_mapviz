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

#include <swri_transform_util/frames.h>

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMouseEvent>
#include <QPainter>
#include <algorithm>
#include <cmath>
#include <coug_mapviz/coug_waypoints_plugin.hpp>
#include <coug_mapviz/utils/geo_conversions.hpp>
#include <cstdlib>
#include <pluginlib/class_list_macros.hpp>
#include <string>
#include <vector>

PLUGINLIB_EXPORT_CLASS(coug_mapviz::CougWaypointsPlugin, mapviz::MapvizPlugin)

namespace coug_mapviz {

using utils::AgentInterface;

namespace {

constexpr double kHitRadiusPx = 15.0;
constexpr double kClickMaxDistPx = 5.0;
constexpr qint64 kClickMaxDurationMs = 500;
constexpr double kDepthEditorLimit = 9999.99;

void setEditorValue(QDoubleSpinBox* editor, double value) {
  editor->blockSignals(true);
  editor->setValue(value);
  editor->blockSignals(false);
}

}  // namespace

CougWaypointsPlugin::CougWaypointsPlugin()
    : MapvizPlugin(),
      ui_(),
      config_widget_(new QWidget()),
      map_canvas_(nullptr),
      selected_idx_(-1),
      dragged_idx_(-1),
      mouse_down_time_(0) {
  ui_.setupUi(config_widget_);

  QPalette config_palette(config_widget_->palette());
  config_palette.setColor(QPalette::Window, Qt::white);
  config_widget_->setPalette(config_palette);
  QPalette status_palette(ui_.status->palette());
  status_palette.setColor(QPalette::Text, Qt::darkGreen);
  ui_.status->setPalette(status_palette);

  connect(ui_.agent_selector, &QComboBox::currentTextChanged, this,
          &CougWaypointsPlugin::AgentChanged);
  connect(ui_.publish, &QPushButton::clicked, this, &CougWaypointsPlugin::PublishWaypoints);
  connect(ui_.clear, &QPushButton::clicked, this, &CougWaypointsPlugin::Clear);
  connect(ui_.start, &QPushButton::clicked, this, &CougWaypointsPlugin::Start);
  connect(ui_.stop, &QPushButton::clicked, this, &CougWaypointsPlugin::Stop);
  connect(ui_.surface, &QPushButton::clicked, this, &CougWaypointsPlugin::Surface);
  connect(ui_.home, &QPushButton::clicked, this, &CougWaypointsPlugin::Home);
  connect(ui_.save, &QPushButton::clicked, this, &CougWaypointsPlugin::SaveWaypoints);
  connect(ui_.load, &QPushButton::clicked, this, &CougWaypointsPlugin::LoadWaypoints);

  const auto editor_changed = QOverload<double>::of(&QDoubleSpinBox::valueChanged);
  connect(ui_.lat_editor, editor_changed, this, &CougWaypointsPlugin::EditorChanged);
  connect(ui_.lon_editor, editor_changed, this, &CougWaypointsPlugin::EditorChanged);
  connect(ui_.depth_editor, editor_changed, this, &CougWaypointsPlugin::EditorChanged);
  connect(ui_.speed_editor, editor_changed, this, &CougWaypointsPlugin::EditorChanged);
  connect(ui_.capture_radius_editor, editor_changed, this, &CougWaypointsPlugin::EditorChanged);
  connect(ui_.capture_radius_z_editor, editor_changed, this, &CougWaypointsPlugin::EditorChanged);
  connect(ui_.slip_radius_editor, editor_changed, this, &CougWaypointsPlugin::EditorChanged);
  connect(ui_.slip_radius_z_editor, editor_changed, this, &CougWaypointsPlugin::EditorChanged);
  connect(ui_.altitude_mode, &QCheckBox::toggled, this, &CougWaypointsPlugin::AltitudeModeChanged);

  connect(this, &CougWaypointsPlugin::StatusUpdateRequested, this,
          &CougWaypointsPlugin::HandleStatusUpdate, Qt::QueuedConnection);
}

CougWaypointsPlugin::~CougWaypointsPlugin() {
  if (map_canvas_) {
    map_canvas_->removeEventFilter(this);
  }
}

bool CougWaypointsPlugin::Initialize(QGLWidget* canvas) {
  map_canvas_ = dynamic_cast<mapviz::MapCanvas*>(canvas);
  if (!map_canvas_) {
    return false;
  }
  map_canvas_->installEventFilter(this);
  renderer_ = std::make_unique<utils::WaypointRenderer>(map_canvas_);

  param_listener_ = std::make_shared<coug_waypoints::ParamListener>(node_);
  params_ = param_listener_->get_params();

  agent_namespaces_ = params_.agent_namespaces;
  for (const auto& agent_ns : agent_namespaces_) {
    ui_.agent_selector->addItem(QString::fromStdString(agent_ns));
  }

  default_waypoint_.speed_rpm = params_.default_speed_rpm;
  default_waypoint_.capture_radius = params_.default_capture_radius;
  default_waypoint_.capture_radius_z = params_.default_capture_radius_z;
  default_waypoint_.slip_radius = params_.default_slip_radius;
  default_waypoint_.slip_radius_z = params_.default_slip_radius_z;
  applyDefaultsToEditors();

  const AgentInterface::Config interface_config{
      params_.waypoint_topic,
      params_.waypoint_map_topic,
      {params_.start_service, params_.stop_service, params_.surface_service, params_.home_service},
  };
  interface_.initialize(node_, tf_manager_, agent_namespaces_, interface_config,
                        [this](AgentInterface::Status level, const std::string& message) {
                          Q_EMIT StatusUpdateRequested(static_cast<int>(level),
                                                       QString::fromStdString(message));
                        });

  initialized_ = true;
  return true;
}

void CougWaypointsPlugin::Draw(double x, double y, double scale) {
  (void)x;
  (void)y;
  (void)scale;
}

void CougWaypointsPlugin::Paint(QPainter* painter, double x, double y, double scale) {
  (void)x;
  (void)y;
  (void)scale;

  swri_transform_util::Transform fixed_T_wgs84;
  if (!tf_manager_->GetTransform(target_frame_, swri_transform_util::_wgs84_frame, fixed_T_wgs84)) {
    return;
  }

  painter->save();
  painter->resetTransform();

  painter->setFont(QFont("DejaVu Sans Mono", 10, QFont::Bold));
  for (const auto& [agent, waypoints] : waypoints_) {
    if (agent != current_agent_ && isAgentKnown(agent)) {
      renderer_->paintPath(painter, waypoints, QColor(200, 200, 200, 191), fixed_T_wgs84);
      renderer_->paintLabels(painter, waypoints, fixed_T_wgs84, QColor(255, 255, 255, 191));
    }
  }

  if (!current_agent_.empty()) {
    const auto& waypoints = agentWaypoints(current_agent_);
    if (!waypoints.empty()) {
      renderer_->paintCircles(painter, waypoints, fixed_T_wgs84);
      renderer_->paintPath(painter, waypoints, QColor(Qt::blue), fixed_T_wgs84, selected_idx_);
      renderer_->paintLabels(painter, waypoints, fixed_T_wgs84, Qt::white);
    }
  }
  painter->restore();
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

bool CougWaypointsPlugin::eventFilter(QObject* object, QEvent* event) {
  (void)object;
  if (!Visible()) {
    return false;
  }
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

bool CougWaypointsPlugin::handleMousePress(QMouseEvent* event) {
  if (current_agent_.empty()) {
    return false;
  }

  dragged_idx_ = -1;
  double distance = 0.0;
  const int closest_idx = findClosestWaypoint(event->localPos(), distance);

  if (event->button() == Qt::LeftButton) {
    mouse_down_pos_ = event->localPos();
    mouse_down_time_ = QDateTime::currentMSecsSinceEpoch();

    if (distance < kHitRadiusPx) {
      dragged_idx_ = closest_idx;
      return true;
    }
  } else if (event->button() == Qt::RightButton) {
    auto it = waypoints_.find(current_agent_);
    if (distance < kHitRadiusPx && it != waypoints_.end() && closest_idx >= 0 &&
        static_cast<size_t>(closest_idx) < it->second.size()) {
      it->second.erase(it->second.begin() + closest_idx);

      if (selected_idx_ == closest_idx) {
        deselectWaypoint();
      } else if (selected_idx_ > closest_idx) {
        --selected_idx_;
      }
      map_canvas_->update();
      return true;
    }
  }
  return false;
}

bool CougWaypointsPlugin::handleMouseRelease(QMouseEvent* event) {
  if (current_agent_.empty()) {
    return false;
  }

  const qreal distance = QLineF(mouse_down_pos_, event->localPos()).length();
  const qint64 press_duration_ms = QDateTime::currentMSecsSinceEpoch() - mouse_down_time_;

  const bool is_click = distance <= kClickMaxDistPx && press_duration_ms < kClickMaxDurationMs;

  if (dragged_idx_ != -1) {
    if (is_click) {
      const auto& waypoints = agentWaypoints(current_agent_);
      if (static_cast<size_t>(dragged_idx_) < waypoints.size()) {
        selected_idx_ = dragged_idx_;
        populateEditors(waypoints[selected_idx_]);
      }
    }
    dragged_idx_ = -1;
    return true;
  }

  if (event->button() == Qt::LeftButton && is_click) {
    geographic_msgs::msg::GeoPoint geo_point;
    if (glToWgs84(event->localPos(), geo_point)) {
      coug_interfaces::msg::WayPoint waypoint;
      waypoint.position = geo_point;
      waypoint.position.altitude = ui_.depth_editor->value();
      waypoint.mode = ui_.altitude_mode->isChecked() ? coug_interfaces::msg::WayPoint::ALTITUDE
                                                     : coug_interfaces::msg::WayPoint::DEPTH;
      waypoint.speed_rpm = ui_.speed_editor->value();
      waypoint.capture_radius = ui_.capture_radius_editor->value();
      waypoint.capture_radius_z = ui_.capture_radius_z_editor->value();
      waypoint.slip_radius = ui_.slip_radius_editor->value();
      waypoint.slip_radius_z = ui_.slip_radius_z_editor->value();

      waypoints_[current_agent_].push_back(waypoint);
      map_canvas_->update();
    }
  }

  dragged_idx_ = -1;
  return false;
}

bool CougWaypointsPlugin::handleMouseMove(QMouseEvent* event) {
  if (current_agent_.empty()) {
    return false;
  }

  if (dragged_idx_ >= 0) {
    if (selected_idx_ != -1) {
      deselectWaypoint();
    }

    geographic_msgs::msg::GeoPoint geo_point;
    auto it = waypoints_.find(current_agent_);
    if (glToWgs84(event->localPos(), geo_point) && it != waypoints_.end() &&
        static_cast<size_t>(dragged_idx_) < it->second.size()) {
      auto& waypoint = it->second[dragged_idx_];
      waypoint.position.longitude = geo_point.longitude;
      waypoint.position.latitude = geo_point.latitude;
      map_canvas_->update();
    }
    return true;
  }
  return false;
}

void CougWaypointsPlugin::HandleStatusUpdate(int level, const QString& message) {
  switch (static_cast<AgentInterface::Status>(level)) {
    case AgentInterface::Status::kInfo:
      PrintInfo(message.toStdString());
      break;
    case AgentInterface::Status::kWarning:
      PrintWarning(message.toStdString());
      break;
    case AgentInterface::Status::kError:
      PrintError(message.toStdString());
      break;
  }
}

void CougWaypointsPlugin::PublishWaypoints() {
  if (ui_.apply_all->isChecked()) {
    publishAll();
  } else if (!current_agent_.empty()) {
    const auto& waypoints = agentWaypoints(current_agent_);
    interface_.publishWaypoints(current_agent_, waypoints, target_frame_);
    PrintInfo("Published 1 agent(s).");
  } else {
    PrintError("No agent selected.");
  }
}

void CougWaypointsPlugin::Clear() {
  int count = 0;
  if (ui_.apply_all->isChecked()) {
    count = static_cast<int>(waypoints_.size());
    waypoints_.clear();
  } else if (!current_agent_.empty()) {
    waypoints_[current_agent_].clear();
    count = 1;
  } else {
    PrintError("No agent selected.");
    return;
  }

  setEditorValue(ui_.lat_editor, 0.0);
  setEditorValue(ui_.lon_editor, 0.0);
  setEditorValue(ui_.depth_editor, 0.0);
  dragged_idx_ = -1;
  deselectWaypoint();
  map_canvas_->update();
  PrintInfo("Cleared " + std::to_string(count) + " agent(s).");
}

void CougWaypointsPlugin::SaveWaypoints() {
  const QString directory = missionDirectory();
  QDir dir(directory);
  if (!dir.exists()) {
    dir.mkpath(".");
  }

  QString filename = QFileDialog::getSaveFileName(config_widget_, "Save Mission", directory,
                                                  "JSON Files (*.json)");
  if (filename.isEmpty()) {
    return;
  }
  if (!filename.endsWith(".json", Qt::CaseInsensitive)) {
    filename += ".json";
  }

  std::string agent_to_save;
  if (!resolveSelectedAgent(agent_to_save)) {
    return;
  }

  QJsonObject mission;
  for (const auto& [agent, waypoints] : waypoints_) {
    if (!agent_to_save.empty() && agent != agent_to_save) continue;
    QJsonArray serialized_waypoints;
    for (const auto& waypoint : waypoints) {
      serialized_waypoints.append(QJsonObject{{"lon", waypoint.position.longitude},
                                              {"lat", waypoint.position.latitude},
                                              {"z", waypoint.position.altitude},
                                              {"mode", static_cast<int>(waypoint.mode)},
                                              {"speed_rpm", waypoint.speed_rpm},
                                              {"capture_radius", waypoint.capture_radius},
                                              {"capture_radius_z", waypoint.capture_radius_z},
                                              {"slip_radius", waypoint.slip_radius},
                                              {"slip_radius_z", waypoint.slip_radius_z}});
    }
    mission[QString::fromStdString(agent)] = serialized_waypoints;
  }

  QFile file(filename);
  if (!file.open(QIODevice::WriteOnly) || file.write(QJsonDocument(mission).toJson()) < 0) {
    PrintError("Failed to save.");
    return;
  }
  const int count = agent_to_save.empty() ? static_cast<int>(waypoints_.size()) : 1;
  PrintInfo("Saved " + std::to_string(count) + " agent(s).");
}

void CougWaypointsPlugin::LoadWaypoints() {
  const QString filename = QFileDialog::getOpenFileName(config_widget_, "Load Mission",
                                                        missionDirectory(), "JSON Files (*.json)");
  if (filename.isEmpty()) {
    return;
  }

  std::string agent_to_load;
  if (!resolveSelectedAgent(agent_to_load)) {
    return;
  }

  QFile file(filename);
  if (!file.open(QIODevice::ReadOnly)) {
    PrintError("Failed to load.");
    return;
  }

  const QJsonDocument document = QJsonDocument::fromJson(file.readAll());
  if (!document.isObject()) {
    PrintError("Failed to load.");
    return;
  }

  int loaded_count = 0;
  const QJsonObject mission = document.object();
  for (const auto& agent_key : mission.keys()) {
    const std::string agent = agent_key.toStdString();
    if ((!agent_to_load.empty() && agent != agent_to_load) || !isAgentKnown(agent) ||
        !mission[agent_key].isArray()) {
      continue;
    }

    std::vector<coug_interfaces::msg::WayPoint> waypoints;
    for (const auto& value : mission[agent_key].toArray()) {
      const QJsonObject serialized_waypoint = value.toObject();
      if (!serialized_waypoint.contains("lat") || !serialized_waypoint.contains("lon")) continue;

      auto waypoint = default_waypoint_;
      waypoint.position.longitude = serialized_waypoint["lon"].toDouble();
      waypoint.position.latitude = serialized_waypoint["lat"].toDouble();
      waypoint.position.altitude = serialized_waypoint["z"].toDouble();
      waypoint.mode = static_cast<uint8_t>(serialized_waypoint["mode"].toInt());
      const auto assign_if_present = [&serialized_waypoint](const char* name, double& field) {
        if (serialized_waypoint.contains(name)) field = serialized_waypoint[name].toDouble();
      };
      assign_if_present("speed_rpm", waypoint.speed_rpm);
      assign_if_present("capture_radius", waypoint.capture_radius);
      assign_if_present("capture_radius_z", waypoint.capture_radius_z);
      assign_if_present("slip_radius", waypoint.slip_radius);
      assign_if_present("slip_radius_z", waypoint.slip_radius_z);
      waypoints.push_back(waypoint);
    }
    waypoints_[agent] = std::move(waypoints);
    ++loaded_count;
  }
  if (loaded_count == 0) {
    PrintError("Mission contains no configured agents.");
    return;
  }

  AgentChanged(QString::fromStdString(current_agent_));
  map_canvas_->update();
  PrintInfo("Loaded " + std::to_string(loaded_count) + " agent(s).");
}

void CougWaypointsPlugin::AgentChanged(const QString& text) {
  current_agent_ = text.toStdString();
  deselectWaypoint();

  const auto& waypoints = agentWaypoints(current_agent_);
  map_canvas_->update();
  if (waypoints.empty()) {
    PrintInfo("Click to add waypoints.");
  } else {
    PrintInfo(current_agent_ + " (" + std::to_string(waypoints.size()) + " waypoints).");
  }
}

void CougWaypointsPlugin::EditorChanged(double value) {
  if (selected_idx_ < 0) return;

  auto it = waypoints_.find(current_agent_);
  if (it == waypoints_.end() || static_cast<size_t>(selected_idx_) >= it->second.size()) {
    return;
  }
  auto& waypoint = it->second[selected_idx_];

  const QObject* editor = sender();
  if (editor == ui_.lat_editor) {
    waypoint.position.latitude = value;
  } else if (editor == ui_.lon_editor) {
    waypoint.position.longitude = value;
  } else if (editor == ui_.depth_editor) {
    waypoint.position.altitude = value;
  } else if (editor == ui_.speed_editor) {
    waypoint.speed_rpm = value;
  } else if (editor == ui_.capture_radius_editor) {
    waypoint.capture_radius = value;
  } else if (editor == ui_.capture_radius_z_editor) {
    waypoint.capture_radius_z = value;
  } else if (editor == ui_.slip_radius_editor) {
    waypoint.slip_radius = value;
  } else if (editor == ui_.slip_radius_z_editor) {
    waypoint.slip_radius_z = value;
  } else {
    return;
  }
  map_canvas_->update();
}

void CougWaypointsPlugin::AltitudeModeChanged(bool checked) {
  if (selected_idx_ < 0) return;

  auto it = waypoints_.find(current_agent_);
  if (it == waypoints_.end() || static_cast<size_t>(selected_idx_) >= it->second.size()) return;

  auto& waypoint = it->second[selected_idx_];
  waypoint.mode =
      checked ? coug_interfaces::msg::WayPoint::ALTITUDE : coug_interfaces::msg::WayPoint::DEPTH;
  const double new_altitude =
      checked ? std::abs(waypoint.position.altitude) : -std::abs(waypoint.position.altitude);
  waypoint.position.altitude = new_altitude;

  setDepthEditorRange(checked);
  setEditorValue(ui_.depth_editor, new_altitude);
  map_canvas_->update();
}

const std::vector<coug_interfaces::msg::WayPoint>& CougWaypointsPlugin::agentWaypoints(
    const std::string& agent) const {
  static const std::vector<coug_interfaces::msg::WayPoint> kNoWaypoints;

  auto it = waypoints_.find(agent);
  return it != waypoints_.end() ? it->second : kNoWaypoints;
}

bool CougWaypointsPlugin::isAgentKnown(const std::string& agent) const {
  return std::find(agent_namespaces_.begin(), agent_namespaces_.end(), agent) !=
         agent_namespaces_.end();
}

bool CougWaypointsPlugin::resolveSelectedAgent(std::string& agent) {
  if (ui_.apply_all->isChecked()) {
    agent.clear();
    return true;
  }
  if (current_agent_.empty()) {
    PrintError("No agent selected.");
    return false;
  }
  agent = current_agent_;
  return true;
}

void CougWaypointsPlugin::applyDefaultsToEditors() {
  setEditorValue(ui_.lat_editor, default_waypoint_.position.latitude);
  setEditorValue(ui_.lon_editor, default_waypoint_.position.longitude);
  setEditorValue(ui_.depth_editor, default_waypoint_.position.altitude);
  setEditorValue(ui_.speed_editor, default_waypoint_.speed_rpm);
  setEditorValue(ui_.capture_radius_editor, default_waypoint_.capture_radius);
  setEditorValue(ui_.capture_radius_z_editor, default_waypoint_.capture_radius_z);
  setEditorValue(ui_.slip_radius_editor, default_waypoint_.slip_radius);
  setEditorValue(ui_.slip_radius_z_editor, default_waypoint_.slip_radius_z);
}

void CougWaypointsPlugin::setWaypointEditorsEnabled(bool enabled) {
  ui_.lat_editor->setEnabled(enabled);
  ui_.lon_editor->setEnabled(enabled);
  ui_.depth_editor->setEnabled(enabled);
  ui_.speed_editor->setEnabled(enabled);
  ui_.capture_radius_editor->setEnabled(enabled);
  ui_.capture_radius_z_editor->setEnabled(enabled);
  ui_.slip_radius_editor->setEnabled(enabled);
  ui_.slip_radius_z_editor->setEnabled(enabled);
}

void CougWaypointsPlugin::setDepthEditorRange(bool altitude_mode) {
  ui_.depth_editor->setRange(altitude_mode ? 0.0 : -kDepthEditorLimit,
                             altitude_mode ? kDepthEditorLimit : 0.0);
}

void CougWaypointsPlugin::populateEditors(const coug_interfaces::msg::WayPoint& waypoint) {
  const bool is_altitude = waypoint.mode == coug_interfaces::msg::WayPoint::ALTITUDE;

  ui_.altitude_mode->blockSignals(true);
  ui_.altitude_mode->setChecked(is_altitude);
  ui_.altitude_mode->setEnabled(true);
  ui_.altitude_mode->blockSignals(false);

  setDepthEditorRange(is_altitude);
  setWaypointEditorsEnabled(true);

  setEditorValue(ui_.lat_editor, waypoint.position.latitude);
  setEditorValue(ui_.lon_editor, waypoint.position.longitude);
  setEditorValue(ui_.depth_editor, waypoint.position.altitude);
  setEditorValue(ui_.speed_editor, waypoint.speed_rpm);
  setEditorValue(ui_.capture_radius_editor, waypoint.capture_radius);
  setEditorValue(ui_.capture_radius_z_editor, waypoint.capture_radius_z);
  setEditorValue(ui_.slip_radius_editor, waypoint.slip_radius);
  setEditorValue(ui_.slip_radius_z_editor, waypoint.slip_radius_z);
}

void CougWaypointsPlugin::deselectWaypoint() {
  selected_idx_ = -1;

  setWaypointEditorsEnabled(false);

  ui_.altitude_mode->blockSignals(true);
  ui_.altitude_mode->setChecked(false);
  ui_.altitude_mode->setEnabled(false);
  ui_.altitude_mode->blockSignals(false);
  setDepthEditorRange(false);
}

void CougWaypointsPlugin::publishAll() {
  for (const auto& agent : agent_namespaces_) {
    interface_.publishWaypoints(agent, agentWaypoints(agent), target_frame_);
  }
  PrintInfo("Published " + std::to_string(agent_namespaces_.size()) + " agent(s).");
}

void CougWaypointsPlugin::callService(AgentInterface::Command command) {
  if (ui_.apply_all->isChecked()) {
    if (agent_namespaces_.empty()) {
      PrintError("No agents configured.");
      return;
    }
    interface_.callService(command, agent_namespaces_, true);
  } else if (!current_agent_.empty()) {
    interface_.callService(command, {current_agent_}, false);
  } else {
    PrintError("No agent selected.");
  }
}

QString CougWaypointsPlugin::missionDirectory() {
  if (const char* overlay_ws = std::getenv("OVERLAY_WS")) {
    return QString::fromUtf8(overlay_ws) + "/src/coug_mapviz/coug_mapviz/missions";
  }
  if (const char* home = std::getenv("HOME")) {
    return QString::fromUtf8(home);
  }
  return QDir::currentPath();
}

bool CougWaypointsPlugin::glToWgs84(const QPointF& gl_point,
                                    geographic_msgs::msg::GeoPoint& geo_point) {
  swri_transform_util::Transform wgs84_T_fixed;
  if (!tf_manager_->GetTransform(swri_transform_util::_wgs84_frame, target_frame_, wgs84_T_fixed)) {
    return false;
  }
  const QPointF fixed_point = map_canvas_->MapGlCoordToFixedFrame(gl_point);
  geo_point = utils::fixedToWgs84(fixed_point.x(), fixed_point.y(), wgs84_T_fixed);
  return true;
}

int CougWaypointsPlugin::findClosestWaypoint(const QPointF& point, double& distance) {
  swri_transform_util::Transform fixed_T_wgs84;
  if (!tf_manager_->GetTransform(target_frame_, swri_transform_util::_wgs84_frame, fixed_T_wgs84)) {
    return -1;
  }

  int closest_idx = -1;
  distance = std::numeric_limits<double>::max();
  const auto& waypoints = agentWaypoints(current_agent_);

  for (size_t i = 0; i < waypoints.size(); i++) {
    const QPointF transformed = renderer_->waypointToGl(waypoints[i], fixed_T_wgs84);

    double dist = QLineF(transformed, point).length();
    if (dist < distance) {
      distance = dist;
      closest_idx = static_cast<int>(i);
    }
  }
  return closest_idx;
}

}  // namespace coug_mapviz
