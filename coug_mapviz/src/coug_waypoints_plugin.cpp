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

#include <mapviz/mapviz_plugin.h>
#include <qcheckbox.h>
#include <qcolor.h>
#include <qcombobox.h>
#include <qcoreevent.h>
#include <qdatetime.h>
#include <qdir.h>
#include <qevent.h>
#include <qfiledevice.h>
#include <qfiledialog.h>
#include <qfileinfo.h>
#include <qgl.h>
#include <qglobal.h>
#include <qjsonarray.h>
#include <qjsonobject.h>
#include <qnamespace.h>
#include <qobject.h>
#include <qobjectdefs.h>
#include <qpainter.h>
#include <qpalette.h>
#include <qpushbutton.h>
#include <qspinbox.h>
#include <qwidget.h>
#include <swri_transform_util/frames.h>
#include <swri_transform_util/transform.h>

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMouseEvent>
#include <QPainter>
#include <QSignalBlocker>
#include <cmath>
#include <coug_mapviz/coug_waypoints_plugin.hpp>
#include <cstdint>
#include <cstdlib>
#include <map>
#include <memory>
#include <pluginlib/class_list_macros.hpp>
#include <string>
#include <tf2/LinearMath/Vector3.hpp>
#include <utility>
#include <vector>

#include "coug_interfaces/msg/way_point.hpp"
#include "coug_mapviz/coug_waypoints_parameters.hpp"
#include "coug_mapviz/utils/fleet_interface.hpp"
#include "coug_mapviz/utils/waypoint_renderer.hpp"

PLUGINLIB_EXPORT_CLASS(coug_mapviz::CougWaypointsPlugin, mapviz::MapvizPlugin)

namespace coug_mapviz {

using coug_interfaces::msg::WayPoint;
using utils::FleetInterface;
using utils::WaypointRenderer;

namespace {

constexpr double kHitRadiusPx = 15.0;
constexpr double kClickMaxDistPx = 5.0;
constexpr qint64 kClickMaxDurationMs = 500;
constexpr double kDepthEditorLimit = 9999.99;

const QColor kConfigBackgroundColor(Qt::white);
const QColor kStatusTextColor(Qt::darkGreen);

void setEditorValue(QDoubleSpinBox* editor, double value) {
  editor->blockSignals(true);
  editor->setValue(value);
  editor->blockSignals(false);
}

}  // namespace

CougWaypointsPlugin::CougWaypointsPlugin()
    : ui_(),
      config_widget_(new QWidget()),
      map_canvas_(nullptr),
      selected_idx_(-1),
      dragged_idx_(-1),
      mouse_down_time_(0) {
  ui_.setupUi(config_widget_);

  QPalette config_palette(config_widget_->palette());
  config_palette.setColor(QPalette::Window, kConfigBackgroundColor);
  config_widget_->setPalette(config_palette);
  QPalette status_palette(ui_.status->palette());
  status_palette.setColor(QPalette::Text, kStatusTextColor);
  ui_.status->setPalette(status_palette);

  connect(ui_.agent_selector, &QComboBox::currentTextChanged, this,
          &CougWaypointsPlugin::AgentChanged);
  connect(ui_.publish, &QPushButton::clicked, this, &CougWaypointsPlugin::PublishWaypoints);
  connect(ui_.clear, &QPushButton::clicked, this, &CougWaypointsPlugin::ClearWaypoints);
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
          &CougWaypointsPlugin::UpdateStatus, Qt::QueuedConnection);
}

CougWaypointsPlugin::~CougWaypointsPlugin() {
  if (map_canvas_ != nullptr) {
    map_canvas_->removeEventFilter(this);
  }
}

auto CougWaypointsPlugin::Initialize(QGLWidget* canvas) -> bool {
  map_canvas_ = dynamic_cast<mapviz::MapCanvas*>(canvas);
  if (map_canvas_ == nullptr) {
    return false;
  }
  map_canvas_->installEventFilter(this);
  renderer_ = std::make_unique<WaypointRenderer>(map_canvas_);

  param_listener_ = std::make_shared<coug_waypoints::ParamListener>(node_);
  params_ = param_listener_->get_params();

  for (const auto& agent_ns : params_.agent_list) {
    ui_.agent_selector->addItem(QString::fromStdString(agent_ns));
  }

  setEditorValue(ui_.speed_editor, params_.default_speed_rpm);
  setEditorValue(ui_.capture_radius_editor, params_.default_capture_radius);
  setEditorValue(ui_.capture_radius_z_editor, params_.default_capture_radius_z);
  setEditorValue(ui_.slip_radius_editor, params_.default_slip_radius);
  setEditorValue(ui_.slip_radius_z_editor, params_.default_slip_radius_z);

  interface_.initialize(
      node_, params_, [this](FleetInterface::Status level, const std::string& message) {
        Q_EMIT StatusUpdateRequested(static_cast<int>(level), QString::fromStdString(message));
      });

  initialized_ = true;
  return true;
}

void CougWaypointsPlugin::Paint(QPainter* painter, double /*unused*/, double /*unused*/,
                                double /*unused*/) {
  painter->save();
  painter->resetTransform();

  painter->setFont(QFont("DejaVu Sans Mono", 10, QFont::Bold));
  for (const auto& [agent, waypoints] : waypoints_) {
    if (agent != current_agent_) {
      renderer_->paintWaypoints(painter, waypoints, false);
    }
  }

  if (!current_agent_.empty()) {
    const auto& waypoints = waypointsForAgent(current_agent_);
    if (!waypoints.empty()) {
      renderer_->paintWaypoints(painter, waypoints, true, selected_idx_);
    }
  }
  painter->restore();
}

void CougWaypointsPlugin::PrintError(const std::string& message) {
  PrintErrorHelper(ui_.status, message);
}

void CougWaypointsPlugin::PrintInfo(const std::string& message) {
  PrintInfoHelper(ui_.status, message);
}

void CougWaypointsPlugin::PrintWarning(const std::string& message) {
  PrintWarningHelper(ui_.status, message);
}

auto CougWaypointsPlugin::eventFilter(QObject* /*watched*/, QEvent* event) -> bool {
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

auto CougWaypointsPlugin::handleMousePress(QMouseEvent* event) -> bool {
  if (current_agent_.empty()) {
    return false;
  }

  dragged_idx_ = -1;
  const int closest_idx = findWaypointAt(event->localPos());

  if (event->button() == Qt::LeftButton) {
    mouse_down_pos_ = event->localPos();
    mouse_down_time_ = QDateTime::currentMSecsSinceEpoch();

    if (closest_idx >= 0) {
      dragged_idx_ = closest_idx;
      return true;
    }
  } else if (event->button() == Qt::RightButton) {
    auto* waypoints = currentWaypoints();
    if ((waypoints != nullptr) && closest_idx >= 0 &&
        static_cast<size_t>(closest_idx) < waypoints->size()) {
      waypoints->erase(waypoints->begin() + closest_idx);

      if (selected_idx_ == closest_idx) {
        clearWaypointSelection();
      } else if (selected_idx_ > closest_idx) {
        --selected_idx_;
      }
      map_canvas_->update();
      return true;
    }
  }
  return false;
}

auto CougWaypointsPlugin::handleMouseRelease(QMouseEvent* event) -> bool {
  if (current_agent_.empty()) {
    return false;
  }

  const qreal distance = QLineF(mouse_down_pos_, event->localPos()).length();
  const qint64 press_duration_ms = QDateTime::currentMSecsSinceEpoch() - mouse_down_time_;

  const bool is_click = distance <= kClickMaxDistPx && press_duration_ms < kClickMaxDurationMs;

  if (dragged_idx_ != -1) {
    if (is_click) {
      const auto& waypoints = waypointsForAgent(current_agent_);
      if (static_cast<size_t>(dragged_idx_) < waypoints.size()) {
        selected_idx_ = dragged_idx_;
        populateEditors(waypoints[selected_idx_]);
      }
    }
    dragged_idx_ = -1;
    return true;
  }

  if (event->button() == Qt::LeftButton && is_click) {
    WayPoint waypoint;
    const QPointF fixed_point = map_canvas_->MapGlCoordToFixedFrame(event->localPos());
    waypoint.position.x = fixed_point.x();
    waypoint.position.y = fixed_point.y();
    waypoint.position.z = ui_.depth_editor->value();
    waypoint.mode = ui_.altitude_mode->isChecked() ? WayPoint::ALTITUDE : WayPoint::DEPTH;
    waypoint.speed_rpm = ui_.speed_editor->value();
    waypoint.capture_radius = ui_.capture_radius_editor->value();
    waypoint.capture_radius_z = ui_.capture_radius_z_editor->value();
    waypoint.slip_radius = ui_.slip_radius_editor->value();
    waypoint.slip_radius_z = ui_.slip_radius_z_editor->value();

    waypoints_[current_agent_].push_back(waypoint);
    map_canvas_->update();
  }

  dragged_idx_ = -1;
  return false;
}

auto CougWaypointsPlugin::handleMouseMove(QMouseEvent* event) -> bool {
  if (current_agent_.empty()) {
    return false;
  }

  if (dragged_idx_ >= 0) {
    if (selected_idx_ != -1) {
      clearWaypointSelection();
    }

    auto* waypoints = currentWaypoints();
    if ((waypoints != nullptr) && static_cast<size_t>(dragged_idx_) < waypoints->size()) {
      auto& waypoint = (*waypoints)[dragged_idx_];
      const QPointF fixed_point = map_canvas_->MapGlCoordToFixedFrame(event->localPos());
      waypoint.position.x = fixed_point.x();
      waypoint.position.y = fixed_point.y();
      map_canvas_->update();
    }
    return true;
  }
  return false;
}

void CougWaypointsPlugin::UpdateStatus(int level, const QString& message) {
  switch (static_cast<FleetInterface::Status>(level)) {
    case FleetInterface::Status::kInfo:
      PrintInfo(message.toStdString());
      break;
    case FleetInterface::Status::kWarning:
      PrintWarning(message.toStdString());
      break;
    case FleetInterface::Status::kError:
      PrintError(message.toStdString());
      break;
  }
}

void CougWaypointsPlugin::AgentChanged(const QString& text) {
  current_agent_ = text.toStdString();
  clearWaypointSelection();

  const auto& waypoints = waypointsForAgent(current_agent_);
  map_canvas_->update();
  if (waypoints.empty()) {
    PrintInfo("Click to add waypoints.");
  } else {
    PrintInfo(current_agent_ + " (" + std::to_string(waypoints.size()) + " waypoints).");
  }
}

void CougWaypointsPlugin::EditorChanged(double value) {
  auto* waypoint = selectedWaypoint();
  if (waypoint == nullptr) {
    return;
  }

  const QObject* editor = sender();
  if (editor == ui_.lat_editor || editor == ui_.lon_editor) {
    QPointF map_point;
    if (!wgs84ToMap(ui_.lat_editor->value(), ui_.lon_editor->value(), map_point)) {
      return;
    }
    waypoint->position.x = map_point.x();
    waypoint->position.y = map_point.y();
  } else if (editor == ui_.depth_editor) {
    waypoint->position.z = value;
  } else if (editor == ui_.speed_editor) {
    waypoint->speed_rpm = value;
  } else if (editor == ui_.capture_radius_editor) {
    waypoint->capture_radius = value;
  } else if (editor == ui_.capture_radius_z_editor) {
    waypoint->capture_radius_z = value;
  } else if (editor == ui_.slip_radius_editor) {
    waypoint->slip_radius = value;
  } else if (editor == ui_.slip_radius_z_editor) {
    waypoint->slip_radius_z = value;
  } else {
    return;
  }
  map_canvas_->update();
}

void CougWaypointsPlugin::AltitudeModeChanged(bool checked) {
  auto* waypoint = selectedWaypoint();
  if (waypoint == nullptr) {
    return;
  }

  waypoint->mode = checked ? WayPoint::ALTITUDE : WayPoint::DEPTH;
  const double new_altitude =
      checked ? std::abs(waypoint->position.z) : -std::abs(waypoint->position.z);
  waypoint->position.z = new_altitude;

  setDepthEditorRange(checked);
  setEditorValue(ui_.depth_editor, new_altitude);
  map_canvas_->update();
}

void CougWaypointsPlugin::PublishWaypoints() {
  const auto agents = targetAgents();
  if (agents.empty()) {
    return;
  }

  for (const auto& agent : agents) {
    interface_.publishWaypoints(agent, waypointsForAgent(agent));
  }
  PrintInfo("Published to " + std::to_string(agents.size()) + " agent(s).");
}

void CougWaypointsPlugin::ClearWaypoints() {
  const auto agents = targetAgents();
  if (agents.empty()) {
    return;
  }

  for (const auto& agent : agents) {
    waypoints_[agent].clear();
  }

  setEditorValue(ui_.lat_editor, 0.0);
  setEditorValue(ui_.lon_editor, 0.0);
  setEditorValue(ui_.depth_editor, 0.0);
  dragged_idx_ = -1;
  clearWaypointSelection();
  map_canvas_->update();
  PrintInfo("Cleared " + std::to_string(agents.size()) + " agent(s).");
}

void CougWaypointsPlugin::SaveWaypoints() {
  const auto agents = targetAgents();
  if (agents.empty()) {
    return;
  }

  const QString directory = missionDirectory();
  if (directory.isEmpty()) {
    PrintError("CONFIG_DIR is not set.");
  } else {
    QDir const dir(directory);
    if (!dir.exists()) {
      dir.mkpath(".");
    }
  }

  QString filename = QFileDialog::getSaveFileName(config_widget_, "Save Mission", directory,
                                                  "JSON Files (*.json)");
  if (filename.isEmpty()) {
    return;
  }
  if (!filename.endsWith(".json", Qt::CaseInsensitive)) {
    filename += ".json";
  }

  QJsonObject mission;
  for (const auto& agent : agents) {
    const auto& waypoints = waypointsForAgent(agent);
    QJsonArray serialized_waypoints;
    for (const auto& waypoint : waypoints) {
      QPointF lat_lon;
      if (!mapToWgs84(QPointF(waypoint.position.x, waypoint.position.y), lat_lon)) {
        PrintError("No transform between " + params_.map_frame + " and WGS84.");
        return;
      }
      serialized_waypoints.append(QJsonObject{{"lat", lat_lon.x()},
                                              {"lon", lat_lon.y()},
                                              {"z", waypoint.position.z},
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
  PrintInfo("Saved " + std::to_string(agents.size()) + " agent(s).");
}

void CougWaypointsPlugin::LoadWaypoints() {
  const auto agents = targetAgents();
  if (agents.empty()) {
    return;
  }

  const QString directory = missionDirectory();
  if (directory.isEmpty()) {
    PrintError("CONFIG_DIR is not set.");
  }

  const QString filename = QFileDialog::getOpenFileName(config_widget_, "Load Mission", directory,
                                                        "JSON Files (*.json)");
  if (filename.isEmpty()) {
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
  std::map<std::string, std::vector<WayPoint>> loaded_waypoints;
  for (const auto& agent : agents) {
    const QJsonValue serialized_waypoints = mission[QString::fromStdString(agent)];
    if (!serialized_waypoints.isArray()) {
      continue;
    }

    std::vector<WayPoint> waypoints;
    for (const auto& value : serialized_waypoints.toArray()) {
      const QJsonObject serialized_waypoint = value.toObject();
      WayPoint waypoint;
      QPointF map_point;
      if (!wgs84ToMap(serialized_waypoint["lat"].toDouble(), serialized_waypoint["lon"].toDouble(),
                      map_point)) {
        PrintError("No transform between WGS84 and " + params_.map_frame + ".");
        return;
      }
      waypoint.position.x = map_point.x();
      waypoint.position.y = map_point.y();
      waypoint.position.z = serialized_waypoint["z"].toDouble();
      waypoint.mode = static_cast<uint8_t>(serialized_waypoint["mode"].toInt());
      waypoint.speed_rpm = serialized_waypoint["speed_rpm"].toDouble();
      waypoint.capture_radius = serialized_waypoint["capture_radius"].toDouble();
      waypoint.capture_radius_z = serialized_waypoint["capture_radius_z"].toDouble();
      waypoint.slip_radius = serialized_waypoint["slip_radius"].toDouble();
      waypoint.slip_radius_z = serialized_waypoint["slip_radius_z"].toDouble();
      waypoints.push_back(waypoint);
    }
    loaded_waypoints[agent] = std::move(waypoints);
    ++loaded_count;
  }
  if (loaded_count == 0) {
    PrintError("No matched agents.");
    return;
  }

  for (auto& [agent, waypoints] : loaded_waypoints) {
    waypoints_[agent] = std::move(waypoints);
  }

  AgentChanged(QString::fromStdString(current_agent_));
  PrintInfo("Loaded " + std::to_string(loaded_count) + " agent(s).");
}

auto CougWaypointsPlugin::waypointsForAgent(const std::string& agent) const
    -> const std::vector<WayPoint>& {
  static const std::vector<WayPoint> kNoWaypoints;

  auto it = waypoints_.find(agent);
  return it != waypoints_.end() ? it->second : kNoWaypoints;
}

auto CougWaypointsPlugin::targetAgents() -> std::vector<std::string> {
  if (ui_.apply_all->isChecked()) {
    if (params_.agent_list.empty()) {
      PrintError("No agents configured.");
    }
    return params_.agent_list;
  }
  if (current_agent_.empty()) {
    PrintError("No agent selected.");
    return {};
  }
  return {current_agent_};
}

auto CougWaypointsPlugin::currentWaypoints() -> std::vector<WayPoint>* {
  auto it = waypoints_.find(current_agent_);
  return it != waypoints_.end() ? &it->second : nullptr;
}

auto CougWaypointsPlugin::selectedWaypoint() -> WayPoint* {
  auto* waypoints = currentWaypoints();
  if ((waypoints == nullptr) || selected_idx_ < 0 ||
      static_cast<size_t>(selected_idx_) >= waypoints->size()) {
    return nullptr;
  }
  return &(*waypoints)[selected_idx_];
}

auto CougWaypointsPlugin::findWaypointAt(const QPointF& point) -> int {
  int closest_idx = -1;
  double closest_distance = kHitRadiusPx;
  const auto& waypoints = waypointsForAgent(current_agent_);

  for (size_t i = 0; i < waypoints.size(); i++) {
    const QPointF transformed =
        renderer_->fixedToGl(QPointF(waypoints[i].position.x, waypoints[i].position.y));
    const double distance = QLineF(transformed, point).length();
    if (distance < closest_distance) {
      closest_distance = distance;
      closest_idx = static_cast<int>(i);
    }
  }
  return closest_idx;
}

void CougWaypointsPlugin::clearWaypointSelection() {
  selected_idx_ = -1;

  setEditorsEnabled(false);

  ui_.altitude_mode->blockSignals(true);
  ui_.altitude_mode->setChecked(false);
  ui_.altitude_mode->setEnabled(false);
  ui_.altitude_mode->blockSignals(false);
  setDepthEditorRange(false);
}

void CougWaypointsPlugin::setEditorsEnabled(bool enabled) {
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
  QSignalBlocker const blocker(ui_.depth_editor);
  ui_.depth_editor->setRange(altitude_mode ? 0.0 : -kDepthEditorLimit,
                             altitude_mode ? kDepthEditorLimit : 0.0);
}

void CougWaypointsPlugin::populateEditors(const WayPoint& waypoint) {
  const bool is_altitude = waypoint.mode == WayPoint::ALTITUDE;

  ui_.altitude_mode->blockSignals(true);
  ui_.altitude_mode->setChecked(is_altitude);
  ui_.altitude_mode->setEnabled(true);
  ui_.altitude_mode->blockSignals(false);

  setDepthEditorRange(is_altitude);
  setEditorsEnabled(true);

  QPointF lat_lon;
  if (mapToWgs84(QPointF(waypoint.position.x, waypoint.position.y), lat_lon)) {
    setEditorValue(ui_.lat_editor, lat_lon.x());
    setEditorValue(ui_.lon_editor, lat_lon.y());
  }
  setEditorValue(ui_.depth_editor, waypoint.position.z);
  setEditorValue(ui_.speed_editor, waypoint.speed_rpm);
  setEditorValue(ui_.capture_radius_editor, waypoint.capture_radius);
  setEditorValue(ui_.capture_radius_z_editor, waypoint.capture_radius_z);
  setEditorValue(ui_.slip_radius_editor, waypoint.slip_radius);
  setEditorValue(ui_.slip_radius_z_editor, waypoint.slip_radius_z);
}

auto CougWaypointsPlugin::missionDirectory() -> QString {
  if (const char* config_dir = std::getenv("CONFIG_DIR")) {
    return QString::fromUtf8(config_dir) + "/missions";
  }
  return {};
}

auto CougWaypointsPlugin::wgs84ToMap(double latitude, double longitude, QPointF& map_point) const
    -> bool {
  swri_transform_util::Transform map_T_wgs84;
  if (!tf_manager_->GetTransform(params_.map_frame, swri_transform_util::_wgs84_frame,
                                 map_T_wgs84)) {
    return false;
  }
  const tf2::Vector3 wgs84_point(longitude, latitude, 0.0);
  const tf2::Vector3 map_coordinates = map_T_wgs84 * wgs84_point;
  map_point = QPointF(map_coordinates.x(), map_coordinates.y());
  return true;
}

auto CougWaypointsPlugin::mapToWgs84(const QPointF& map_point, QPointF& lat_lon) const -> bool {
  swri_transform_util::Transform wgs84_T_map;
  if (!tf_manager_->GetTransform(swri_transform_util::_wgs84_frame, params_.map_frame,
                                 wgs84_T_map)) {
    return false;
  }
  const tf2::Vector3 wgs84_point = wgs84_T_map * tf2::Vector3(map_point.x(), map_point.y(), 0.0);
  lat_lon = QPointF(wgs84_point.y(), wgs84_point.x());
  return true;
}

void CougWaypointsPlugin::callFleetService(FleetInterface::Service service) {
  const auto agents = targetAgents();
  if (!agents.empty()) {
    interface_.callService(service, agents);
  }
}

}  // namespace coug_mapviz
