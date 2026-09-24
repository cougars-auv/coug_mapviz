// Copyright 2026 BYU FROST Lab
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
#include <qiodevice.h>
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
#include <QFileInfo>
#include <QIODevice>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMouseEvent>
#include <QPainter>
#include <QSignalBlocker>
#include <QTimer>
#include <QtGlobal>
#include <cmath>
#include <coug_mapviz/coug_waypoints_plugin.hpp>
#include <cstddef>
#include <cstdint>
#include <geometry_msgs/msg/point.hpp>
#include <map>
#include <mapviz/mapviz_plugin.hpp>
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
  const QSignalBlocker blocker(editor);
  editor->setValue(value);
}

void setEditorValue(QSpinBox* editor, int value, bool enabled) {
  const QSignalBlocker blocker(editor);
  editor->setValue(value);
  editor->setEnabled(enabled);
}

void setSelectorValue(QComboBox* selector, int index, bool enabled) {
  const QSignalBlocker blocker(selector);
  selector->setCurrentIndex(index);
  selector->setEnabled(enabled);
}

void setToggleValue(QCheckBox* toggle, bool checked, bool enabled) {
  const QSignalBlocker blocker(toggle);
  toggle->setChecked(checked);
  toggle->setEnabled(enabled);
}

auto toLatLon(const swri_transform_util::Transform& wgs84_T_map, const QPointF& map_point)
    -> QPointF {
  const tf2::Vector3 wgs84_point = wgs84_T_map * tf2::Vector3(map_point.x(), map_point.y(), 0.0);
  return {wgs84_point.y(), wgs84_point.x()};
}

auto toMapPoint(const swri_transform_util::Transform& map_T_wgs84, double latitude,
                double longitude) -> QPointF {
  const tf2::Vector3 map_coordinates = map_T_wgs84 * tf2::Vector3(longitude, latitude, 0.0);
  return {map_coordinates.x(), map_coordinates.y()};
}

}  // namespace

CougWaypointsPlugin::CougWaypointsPlugin() : ui_(), config_widget_(new QWidget()) {
  ui_.setupUi(config_widget_);

  QPalette config_palette(config_widget_->palette());
  config_palette.setColor(QPalette::Window, kConfigBackgroundColor);
  config_widget_->setPalette(config_palette);
  QPalette status_palette(ui_.status->palette());
  status_palette.setColor(QPalette::Text, kStatusTextColor);
  ui_.status->setPalette(status_palette);
  ui_.status->installEventFilter(this);
  config_widget_->installEventFilter(this);

  connect(ui_.agent_selector, &QComboBox::currentTextChanged, this,
          &CougWaypointsPlugin::AgentChanged);
  connect(ui_.lat_editor, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &CougWaypointsPlugin::EditorChanged);
  connect(ui_.lon_editor, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &CougWaypointsPlugin::EditorChanged);
  connect(ui_.type_selector, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          &CougWaypointsPlugin::TypeChanged);
  connect(ui_.tag_editor, QOverload<int>::of(&QSpinBox::valueChanged), this,
          &CougWaypointsPlugin::TagChanged);
  connect(ui_.flash_toggle, &QCheckBox::toggled, this, &CougWaypointsPlugin::FlashChanged);
  connect(ui_.depth_editor, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &CougWaypointsPlugin::EditorChanged);
  connect(ui_.altitude_mode, &QCheckBox::toggled, this, &CougWaypointsPlugin::AltitudeModeChanged);
  connect(ui_.speed_editor, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &CougWaypointsPlugin::EditorChanged);
  connect(ui_.capture_radius_editor, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &CougWaypointsPlugin::EditorChanged);
  connect(ui_.capture_radius_z_editor, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &CougWaypointsPlugin::EditorChanged);
  connect(ui_.slip_radius_editor, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &CougWaypointsPlugin::EditorChanged);
  connect(ui_.slip_radius_z_editor, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &CougWaypointsPlugin::EditorChanged);
  connect(ui_.publish, &QPushButton::clicked, this, &CougWaypointsPlugin::PublishWaypoints);
  connect(ui_.clear, &QPushButton::clicked, this, &CougWaypointsPlugin::ClearWaypoints);
  connect(ui_.load, &QPushButton::clicked, this, &CougWaypointsPlugin::LoadWaypoints);
  connect(ui_.save, &QPushButton::clicked, this, &CougWaypointsPlugin::SaveWaypoints);
  connect(ui_.start, &QPushButton::clicked, this, &CougWaypointsPlugin::Start);
  connect(ui_.stop, &QPushButton::clicked, this, &CougWaypointsPlugin::Stop);
  connect(ui_.home, &QPushButton::clicked, this, &CougWaypointsPlugin::Home);
  connect(ui_.surface, &QPushButton::clicked, this, &CougWaypointsPlugin::Surface);

  connect(this, &CougWaypointsPlugin::StatusUpdateRequested, this,
          &CougWaypointsPlugin::UpdateStatus, Qt::QueuedConnection);
}

CougWaypointsPlugin::~CougWaypointsPlugin() {
  if (map_canvas_ != nullptr) {
    map_canvas_->removeEventFilter(this);
  }
}

auto CougWaypointsPlugin::Initialize(QOpenGLWidget* canvas) -> bool {
  map_canvas_ = dynamic_cast<mapviz::MapCanvas*>(canvas);
  if (map_canvas_ == nullptr) {
    return false;
  }
  map_canvas_->installEventFilter(this);
  renderer_ = std::make_unique<WaypointRenderer>(map_canvas_);

  param_listener_ = std::make_shared<coug_waypoints::ParamListener>(NodeUnsafe());
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
      NodeUnsafe(), params_, [this](FleetInterface::Status level, const std::string& message) {
        Q_EMIT StatusUpdateRequested(static_cast<int>(level), QString::fromStdString(message));
      });

  initialized_ = true;
  return true;
}

void CougWaypointsPlugin::Paint(QPainter* painter, double /*x*/, double /*y*/, double /*scale*/) {
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
      renderer_->paintWaypoints(painter, waypoints, true, selected_waypoint_idx_);
    }
  }
  painter->restore();
}

void CougWaypointsPlugin::PrintError(const std::string& message) {
  PrintErrorHelper(ui_.status, message);
  updateStatusHeight();
}

void CougWaypointsPlugin::PrintInfo(const std::string& message) {
  PrintInfoHelper(ui_.status, message);
  updateStatusHeight();
}

void CougWaypointsPlugin::PrintWarning(const std::string& message) {
  PrintWarningHelper(ui_.status, message);
  updateStatusHeight();
}

void CougWaypointsPlugin::updateStatusHeight() {
  const int width = ui_.status->width();
  if (width <= 0) {
    return;
  }
  const int old_height = ui_.status->maximumHeight();
  ui_.status->setMinimumHeight(0);
  const int height = ui_.status->heightForWidth(width);
  ui_.status->setFixedHeight(height);
  if (height != old_height) {
    QTimer::singleShot(0, this, [this] { Q_EMIT SizeChanged(); });
  }
}

auto CougWaypointsPlugin::eventFilter(QObject* watched, QEvent* event) -> bool {
  MAPVIZ_ASSERT_GUI_THREAD();
  if (watched == ui_.status || watched == config_widget_) {
    if (event->type() == QEvent::Resize) {
      updateStatusHeight();
    }
    return false;
  }
  if (!Visible()) {
    return false;
  }
  switch (event->type()) {
    case QEvent::MouseButtonPress:
      // NOLINTNEXTLINE(cppcoreguidelines-pro-type-static-cast-downcast)
      return handleMousePress(static_cast<QMouseEvent*>(event));
    case QEvent::MouseButtonRelease:
      // NOLINTNEXTLINE(cppcoreguidelines-pro-type-static-cast-downcast)
      return handleMouseRelease(static_cast<QMouseEvent*>(event));
    case QEvent::MouseMove:
      // NOLINTNEXTLINE(cppcoreguidelines-pro-type-static-cast-downcast)
      return handleMouseMove(static_cast<QMouseEvent*>(event));
    default:
      return false;
  }
}

auto CougWaypointsPlugin::handleMousePress(QMouseEvent* event) -> bool {
  if (current_agent_.empty()) {
    return false;
  }

  dragged_hit_ = {};
  const WaypointHit hit = findHitAt(event->localPos());

  if (event->button() == Qt::LeftButton) {
    mouse_down_pos_ = event->localPos();
    mouse_down_time_ = QDateTime::currentMSecsSinceEpoch();

    if (hit.valid()) {
      dragged_hit_ = hit;
      return true;
    }
  } else if (event->button() == Qt::RightButton && eraseHit(hit)) {
    map_canvas_->update();
    return true;
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

  if (dragged_hit_.valid()) {
    const WaypointHit hit = dragged_hit_;
    dragged_hit_ = {};
    if (!is_click || hit.subwaypoint_idx >= 0) {
      return true;
    }

    if (selected_waypoint_idx_ == hit.waypoint_idx) {
      clearWaypointSelection();
    } else {
      const auto& waypoints = waypointsForAgent(current_agent_);
      if (static_cast<size_t>(hit.waypoint_idx) < waypoints.size()) {
        selected_waypoint_idx_ = hit.waypoint_idx;
        populateEditors(waypoints[selected_waypoint_idx_]);
      }
    }
    map_canvas_->update();
    return true;
  }

  if (event->button() == Qt::LeftButton && is_click) {
    const QPointF fixed_point = map_canvas_->MapGlCoordToFixedFrame(event->localPos());
    auto* selected = selectedWaypoint();

    if ((selected != nullptr) && selected->type == WayPoint::ARUCO) {
      geometry_msgs::msg::Point subwaypoint;
      subwaypoint.x = fixed_point.x();
      subwaypoint.y = fixed_point.y();
      selected->subwaypoints.push_back(subwaypoint);
    } else {
      WayPoint waypoint;
      waypoint.position.x = fixed_point.x();
      waypoint.position.y = fixed_point.y();
      waypoint.position.z = ui_.depth_editor->value();

      waypoint.speed_rpm = ui_.speed_editor->value();
      waypoint.capture_radius = ui_.capture_radius_editor->value();
      waypoint.capture_radius_z = ui_.capture_radius_z_editor->value();
      waypoint.slip_radius = ui_.slip_radius_editor->value();
      waypoint.slip_radius_z = ui_.slip_radius_z_editor->value();

      waypoint.mode = ui_.altitude_mode->isChecked() ? WayPoint::ALTITUDE : WayPoint::DEPTH;
      waypoint.type = WayPoint::GPS;

      waypoints_[current_agent_].push_back(waypoint);
      if (selected_waypoint_idx_ != -1) {
        clearWaypointSelection();
      }
    }
    map_canvas_->update();
  }
  return false;
}

auto CougWaypointsPlugin::handleMouseMove(QMouseEvent* event) -> bool {
  if (current_agent_.empty()) {
    return false;
  }

  if (!dragged_hit_.valid()) {
    return false;
  }

  auto* waypoints = currentWaypoints();
  if ((waypoints == nullptr) ||
      static_cast<size_t>(dragged_hit_.waypoint_idx) >= waypoints->size()) {
    return true;
  }

  auto& waypoint = (*waypoints)[dragged_hit_.waypoint_idx];
  const QPointF fixed_point = map_canvas_->MapGlCoordToFixedFrame(event->localPos());

  if (dragged_hit_.subwaypoint_idx >= 0) {
    if (static_cast<size_t>(dragged_hit_.subwaypoint_idx) < waypoint.subwaypoints.size()) {
      auto& subwaypoint = waypoint.subwaypoints[dragged_hit_.subwaypoint_idx];
      subwaypoint.x = fixed_point.x();
      subwaypoint.y = fixed_point.y();
    }
  } else {
    if (selected_waypoint_idx_ != -1) {
      clearWaypointSelection();
    }
    moveWaypoint(waypoint, fixed_point);
  }
  map_canvas_->update();
  return true;
}

void CougWaypointsPlugin::UpdateStatus(int level, const QString& message) {
  MAPVIZ_ASSERT_GUI_THREAD();
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
    PrintInfo("'" + current_agent_ + "' has " + std::to_string(waypoints.size()) + " waypoint(s).");
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
    moveWaypoint(*waypoint, map_point);
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

void CougWaypointsPlugin::TypeChanged(int index) {
  auto* waypoint = selectedWaypoint();
  if (waypoint == nullptr) {
    return;
  }

  if (index == WayPoint::ARUCO) {
    waypoint->type = WayPoint::ARUCO;
    waypoint->arrival_flash = true;
    if (waypoint->subwaypoints.empty()) {
      waypoint->subwaypoints = buildSearchPattern(*waypoint);
    }
  } else {
    waypoint->type = WayPoint::GPS;
    waypoint->arrival_flash = false;
    waypoint->tag_id = 0;
    waypoint->subwaypoints.clear();
  }
  setEditorValue(ui_.tag_editor, waypoint->tag_id, waypoint->type == WayPoint::ARUCO);
  setToggleValue(ui_.flash_toggle, waypoint->arrival_flash, waypoint->type == WayPoint::GPS);
  map_canvas_->update();
}

void CougWaypointsPlugin::TagChanged(int value) {
  auto* waypoint = selectedWaypoint();
  if (waypoint == nullptr) {
    return;
  }
  waypoint->tag_id = static_cast<uint16_t>(value);
}

void CougWaypointsPlugin::FlashChanged(bool checked) {
  auto* waypoint = selectedWaypoint();
  if (waypoint == nullptr) {
    return;
  }
  waypoint->arrival_flash = checked;
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

  size_t waypoint_count = 0;
  for (const auto& agent : agents) {
    interface_.publishWaypoints(agent, waypointsForAgent(agent));
    waypoint_count += waypointsForAgent(agent).size();
  }
  PrintInfo("Published " + std::to_string(waypoint_count) + " waypoint(s) to " +
            std::to_string(agents.size()) + " agent(s).");
}

void CougWaypointsPlugin::ClearWaypoints() {
  const auto agents = targetAgents();
  if (agents.empty()) {
    return;
  }

  size_t waypoint_count = 0;
  for (const auto& agent : agents) {
    waypoint_count += waypoints_[agent].size();
    waypoints_[agent].clear();
  }

  setEditorValue(ui_.lat_editor, 0.0);
  setEditorValue(ui_.lon_editor, 0.0);
  setEditorValue(ui_.depth_editor, 0.0);
  dragged_hit_ = {};
  clearWaypointSelection();
  map_canvas_->update();
  PrintInfo("Cleared " + std::to_string(waypoint_count) + " waypoint(s) from " +
            std::to_string(agents.size()) + " agent(s).");
}

void CougWaypointsPlugin::LoadWaypoints() {
  const auto agents = targetAgents();
  if (agents.empty()) {
    return;
  }

  const QString directory = missionDirectory();
  if (directory.isEmpty()) {
    PrintWarning("CONFIG_DIR is not set, using the default directory.");
  }

  const QString filename = QFileDialog::getOpenFileName(config_widget_, "Load Mission", directory,
                                                        "JSON Files (*.json)");
  if (filename.isEmpty()) {
    return;
  }

  QFile file(filename);
  if (!file.open(QIODevice::ReadOnly)) {
    PrintError("Failed to open '" + QFileInfo(filename).fileName().toStdString() + "'.");
    return;
  }

  const QJsonDocument document = QJsonDocument::fromJson(file.readAll());
  if (!document.isObject()) {
    PrintError("Invalid mission file: '" + QFileInfo(filename).fileName().toStdString() + "'.");
    return;
  }

  swri_transform_util::Transform map_T_wgs84;
  if (!tf_manager_->GetTransform(params_.map_frame, swri_transform_util::_wgs84_frame,
                                 map_T_wgs84)) {
    PrintError("No transform between '" + swri_transform_util::_wgs84_frame + "' and '" +
               params_.map_frame + "'.");
    return;
  }

  int loaded_count = 0;
  size_t waypoint_count = 0;
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
      const QPointF map_point = toMapPoint(map_T_wgs84, serialized_waypoint["lat"].toDouble(),
                                           serialized_waypoint["lon"].toDouble());
      waypoint.position.x = map_point.x();
      waypoint.position.y = map_point.y();
      waypoint.position.z = serialized_waypoint["z"].toDouble();

      waypoint.speed_rpm = serialized_waypoint["speed_rpm"].toDouble();
      waypoint.capture_radius = serialized_waypoint["capture_radius"].toDouble();
      waypoint.capture_radius_z = serialized_waypoint["capture_radius_z"].toDouble();
      waypoint.slip_radius = serialized_waypoint["slip_radius"].toDouble();
      waypoint.slip_radius_z = serialized_waypoint["slip_radius_z"].toDouble();

      waypoint.mode = static_cast<uint8_t>(serialized_waypoint["mode"].toInt());

      for (const auto& sub_value : serialized_waypoint["subwaypoints"].toArray()) {
        const QJsonObject serialized_subwaypoint = sub_value.toObject();
        const QPointF sub_map_point =
            toMapPoint(map_T_wgs84, serialized_subwaypoint["lat"].toDouble(),
                       serialized_subwaypoint["lon"].toDouble());
        geometry_msgs::msg::Point subwaypoint;
        subwaypoint.x = sub_map_point.x();
        subwaypoint.y = sub_map_point.y();
        waypoint.subwaypoints.push_back(subwaypoint);
      }
      waypoint.tag_id = static_cast<uint16_t>(serialized_waypoint["tag_id"].toInt());
      waypoint.arrival_flash = serialized_waypoint["arrival_flash"].toBool() ||
                               serialized_waypoint["type"].toInt() == WayPoint::ARUCO;

      waypoint.type = static_cast<uint8_t>(serialized_waypoint["type"].toInt());

      waypoints.push_back(waypoint);
    }
    waypoint_count += waypoints.size();
    loaded_waypoints[agent] = std::move(waypoints);
    ++loaded_count;
  }
  if (loaded_count == 0) {
    PrintError("No agents in the mission file match the selection.");
    return;
  }

  for (auto& [agent, waypoints] : loaded_waypoints) {
    waypoints_[agent] = std::move(waypoints);
  }

  AgentChanged(QString::fromStdString(current_agent_));
  PrintInfo("Loaded " + std::to_string(waypoint_count) + " waypoint(s) for " +
            std::to_string(loaded_count) + " agent(s) from '" +
            QFileInfo(filename).fileName().toStdString() + "'.");
}

void CougWaypointsPlugin::SaveWaypoints() {
  const auto agents = targetAgents();
  if (agents.empty()) {
    return;
  }

  const QString directory = missionDirectory();
  if (directory.isEmpty()) {
    PrintWarning("CONFIG_DIR is not set, using the default directory.");
  } else {
    const QDir dir(directory);
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

  swri_transform_util::Transform wgs84_T_map;
  if (!tf_manager_->GetTransform(swri_transform_util::_wgs84_frame, params_.map_frame,
                                 wgs84_T_map)) {
    PrintError("No transform between '" + params_.map_frame + "' and '" +
               swri_transform_util::_wgs84_frame + "'.");
    return;
  }

  QJsonObject mission;
  size_t waypoint_count = 0;
  for (const auto& agent : agents) {
    const auto& waypoints = waypointsForAgent(agent);
    waypoint_count += waypoints.size();
    QJsonArray serialized_waypoints;
    for (const auto& waypoint : waypoints) {
      const QPointF lat_lon =
          toLatLon(wgs84_T_map, QPointF(waypoint.position.x, waypoint.position.y));

      QJsonArray serialized_subwaypoints;
      for (const auto& subwaypoint : waypoint.subwaypoints) {
        const QPointF sub_lat_lon = toLatLon(wgs84_T_map, QPointF(subwaypoint.x, subwaypoint.y));
        serialized_subwaypoints.append(
            QJsonObject{{"lat", sub_lat_lon.x()}, {"lon", sub_lat_lon.y()}});
      }

      serialized_waypoints.append(QJsonObject{{"lat", lat_lon.x()},
                                              {"lon", lat_lon.y()},
                                              {"z", waypoint.position.z},
                                              {"speed_rpm", waypoint.speed_rpm},
                                              {"capture_radius", waypoint.capture_radius},
                                              {"capture_radius_z", waypoint.capture_radius_z},
                                              {"slip_radius", waypoint.slip_radius},
                                              {"slip_radius_z", waypoint.slip_radius_z},
                                              {"mode", static_cast<int>(waypoint.mode)},
                                              {"subwaypoints", serialized_subwaypoints},
                                              {"tag_id", static_cast<int>(waypoint.tag_id)},
                                              {"arrival_flash", waypoint.arrival_flash},
                                              {"type", static_cast<int>(waypoint.type)}});
    }
    mission[QString::fromStdString(agent)] = serialized_waypoints;
  }

  QFile file(filename);
  if (!file.open(QIODevice::WriteOnly) || file.write(QJsonDocument(mission).toJson()) < 0) {
    PrintError("Failed to save '" + QFileInfo(filename).fileName().toStdString() + "'.");
    return;
  }
  PrintInfo("Saved " + std::to_string(waypoint_count) + " waypoint(s) for " +
            std::to_string(agents.size()) + " agent(s) to '" +
            QFileInfo(filename).fileName().toStdString() + "'.");
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
  if ((waypoints == nullptr) || selected_waypoint_idx_ < 0 ||
      static_cast<size_t>(selected_waypoint_idx_) >= waypoints->size()) {
    return nullptr;
  }
  return &(*waypoints)[selected_waypoint_idx_];
}

auto CougWaypointsPlugin::findHitAt(const QPointF& point) -> WaypointHit {
  WaypointHit closest;
  double closest_distance = kHitRadiusPx;
  const auto& waypoints = waypointsForAgent(current_agent_);

  const auto consider = [&](const QPointF& fixed_point, int waypoint_idx, int subwaypoint_idx) {
    const double distance = QLineF(renderer_->fixedToGl(fixed_point), point).length();
    if (distance < closest_distance) {
      closest_distance = distance;
      closest = WaypointHit{waypoint_idx, subwaypoint_idx};
    }
  };

  for (size_t i = 0; i < waypoints.size(); i++) {
    const auto waypoint_idx = static_cast<int>(i);
    consider(QPointF(waypoints[i].position.x, waypoints[i].position.y), waypoint_idx, -1);
    for (size_t j = 0; j < waypoints[i].subwaypoints.size(); j++) {
      const auto& subwaypoint = waypoints[i].subwaypoints[j];
      consider(QPointF(subwaypoint.x, subwaypoint.y), waypoint_idx, static_cast<int>(j));
    }
  }
  return closest;
}

auto CougWaypointsPlugin::eraseHit(const WaypointHit& hit) -> bool {
  auto* waypoints = currentWaypoints();
  if ((waypoints == nullptr) || !hit.valid() ||
      static_cast<size_t>(hit.waypoint_idx) >= waypoints->size()) {
    return false;
  }

  if (hit.subwaypoint_idx >= 0) {
    auto& subwaypoints = (*waypoints)[hit.waypoint_idx].subwaypoints;
    if (static_cast<size_t>(hit.subwaypoint_idx) >= subwaypoints.size()) {
      return false;
    }
    subwaypoints.erase(subwaypoints.begin() + hit.subwaypoint_idx);
    return true;
  }

  waypoints->erase(waypoints->begin() + hit.waypoint_idx);
  if (selected_waypoint_idx_ == hit.waypoint_idx) {
    clearWaypointSelection();
  } else if (selected_waypoint_idx_ > hit.waypoint_idx) {
    --selected_waypoint_idx_;
  }
  return true;
}

auto CougWaypointsPlugin::buildSearchPattern(const WayPoint& waypoint) const
    -> std::vector<geometry_msgs::msg::Point> {
  const auto point_count = static_cast<int>(params_.default_search_points);
  const auto& radii = params_.default_search_radii;
  const double angle_step = 2.0 * M_PI / point_count;

  std::vector<geometry_msgs::msg::Point> pattern;
  pattern.reserve(radii.size() * static_cast<size_t>(point_count));
  for (size_t ring = 0; ring < radii.size(); ++ring) {
    const double offset = (ring % 2 == 0) ? 0.0 : 0.5 * angle_step;
    for (int i = 0; i < point_count; ++i) {
      const double angle = (angle_step * i) + offset;
      geometry_msgs::msg::Point subwaypoint;
      subwaypoint.x = waypoint.position.x + radii[ring] * std::cos(angle);
      subwaypoint.y = waypoint.position.y + radii[ring] * std::sin(angle);
      pattern.push_back(subwaypoint);
    }
  }
  return pattern;
}

void CougWaypointsPlugin::moveWaypoint(WayPoint& waypoint, const QPointF& map_point) {
  const double delta_x = map_point.x() - waypoint.position.x;
  const double delta_y = map_point.y() - waypoint.position.y;
  waypoint.position.x = map_point.x();
  waypoint.position.y = map_point.y();

  for (auto& subwaypoint : waypoint.subwaypoints) {
    subwaypoint.x += delta_x;
    subwaypoint.y += delta_y;
  }
}

void CougWaypointsPlugin::clearWaypointSelection() {
  selected_waypoint_idx_ = -1;

  setEditorsEnabled(false);
  setSelectorValue(ui_.type_selector, WayPoint::GPS, false);
  setEditorValue(ui_.tag_editor, 0, false);
  setToggleValue(ui_.flash_toggle, false, false);
}

void CougWaypointsPlugin::setEditorsEnabled(bool enabled) {
  ui_.lat_editor->setEnabled(enabled);
  ui_.lon_editor->setEnabled(enabled);
  ui_.depth_editor->setEnabled(enabled);
  ui_.altitude_mode->setEnabled(enabled);
  ui_.speed_editor->setEnabled(enabled);
  ui_.capture_radius_editor->setEnabled(enabled);
  ui_.capture_radius_z_editor->setEnabled(enabled);
  ui_.slip_radius_editor->setEnabled(enabled);
  ui_.slip_radius_z_editor->setEnabled(enabled);
}

void CougWaypointsPlugin::setDepthEditorRange(bool altitude_mode) {
  const QSignalBlocker blocker(ui_.depth_editor);
  ui_.depth_editor->setRange(altitude_mode ? 0.0 : -kDepthEditorLimit,
                             altitude_mode ? kDepthEditorLimit : 0.0);
}

void CougWaypointsPlugin::populateEditors(const WayPoint& waypoint) {
  const bool is_altitude = waypoint.mode == WayPoint::ALTITUDE;

  setSelectorValue(ui_.type_selector, waypoint.type, true);
  setEditorValue(ui_.tag_editor, waypoint.tag_id, waypoint.type == WayPoint::ARUCO);
  setToggleValue(ui_.flash_toggle, waypoint.arrival_flash, waypoint.type == WayPoint::GPS);
  setDepthEditorRange(is_altitude);
  setToggleValue(ui_.altitude_mode, is_altitude, true);
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
  const QString config_dir = qEnvironmentVariable("CONFIG_DIR");
  if (config_dir.isEmpty()) {
    return {};
  }
  return config_dir + "/missions";
}

auto CougWaypointsPlugin::wgs84ToMap(double latitude, double longitude, QPointF& map_point) const
    -> bool {
  swri_transform_util::Transform map_T_wgs84;
  if (!tf_manager_->GetTransform(params_.map_frame, swri_transform_util::_wgs84_frame,
                                 map_T_wgs84)) {
    return false;
  }
  map_point = toMapPoint(map_T_wgs84, latitude, longitude);
  return true;
}

auto CougWaypointsPlugin::mapToWgs84(const QPointF& map_point, QPointF& lat_lon) const -> bool {
  swri_transform_util::Transform wgs84_T_map;
  if (!tf_manager_->GetTransform(swri_transform_util::_wgs84_frame, params_.map_frame,
                                 wgs84_T_map)) {
    return false;
  }
  lat_lon = toLatLon(wgs84_T_map, map_point);
  return true;
}

void CougWaypointsPlugin::callFleetService(FleetInterface::Service service) {
  const auto agents = targetAgents();
  if (!agents.empty()) {
    interface_.callService(service, agents);
  }
}

}  // namespace coug_mapviz
