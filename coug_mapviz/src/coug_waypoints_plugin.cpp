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
#include <QFileDialog>
#include <QMouseEvent>
#include <QPainter>
#include <algorithm>
#include <array>
#include <cmath>
#include <coug_mapviz/coug_waypoints_plugin.hpp>
#include <coug_mapviz/utils/geo_conversions.hpp>
#include <coug_mapviz/utils/mission_io.hpp>
#include <cstdlib>
#include <pluginlib/class_list_macros.hpp>
#include <string>
#include <type_traits>
#include <vector>

PLUGINLIB_EXPORT_CLASS(coug_mapviz::CougWaypointsPlugin, mapviz::MapvizPlugin)

namespace coug_mapviz {

using utils::AgentInterface;

static constexpr double kHitRadiusPx = 15.0;
static constexpr double kClickMaxDistPx = 5.0;
static constexpr qint64 kClickMaxDurationMs = 500;
static constexpr int kCircleSegments = 48;
static constexpr double kDepthEditorLimit = 9999.99;

namespace {

struct EditorBinding {
  QDoubleSpinBox* Ui::coug_waypoints_config::* editor;
  double (*get)(const coug_interfaces::msg::WayPoint&);
  void (*set)(coug_interfaces::msg::WayPoint&, double);
};

constexpr std::array<EditorBinding, 8> kEditorBindings = {{
    {&Ui::coug_waypoints_config::lat_editor,
     [](const coug_interfaces::msg::WayPoint& waypoint) { return waypoint.position.latitude; },
     [](coug_interfaces::msg::WayPoint& waypoint, double value) {
       waypoint.position.latitude = value;
     }},
    {&Ui::coug_waypoints_config::lon_editor,
     [](const coug_interfaces::msg::WayPoint& waypoint) { return waypoint.position.longitude; },
     [](coug_interfaces::msg::WayPoint& waypoint, double value) {
       waypoint.position.longitude = value;
     }},
    {&Ui::coug_waypoints_config::depth_editor,
     [](const coug_interfaces::msg::WayPoint& waypoint) { return waypoint.position.altitude; },
     [](coug_interfaces::msg::WayPoint& waypoint, double value) {
       waypoint.position.altitude = value;
     }},
    {&Ui::coug_waypoints_config::speed_editor,
     [](const coug_interfaces::msg::WayPoint& waypoint) { return waypoint.speed_rpm; },
     [](coug_interfaces::msg::WayPoint& waypoint, double value) { waypoint.speed_rpm = value; }},
    {&Ui::coug_waypoints_config::capture_radius_editor,
     [](const coug_interfaces::msg::WayPoint& waypoint) { return waypoint.capture_radius; },
     [](coug_interfaces::msg::WayPoint& waypoint, double value) {
       waypoint.capture_radius = value;
     }},
    {&Ui::coug_waypoints_config::capture_radius_z_editor,
     [](const coug_interfaces::msg::WayPoint& waypoint) { return waypoint.capture_radius_z; },
     [](coug_interfaces::msg::WayPoint& waypoint, double value) {
       waypoint.capture_radius_z = value;
     }},
    {&Ui::coug_waypoints_config::slip_radius_editor,
     [](const coug_interfaces::msg::WayPoint& waypoint) { return waypoint.slip_radius; },
     [](coug_interfaces::msg::WayPoint& waypoint, double value) { waypoint.slip_radius = value; }},
    {&Ui::coug_waypoints_config::slip_radius_z_editor,
     [](const coug_interfaces::msg::WayPoint& waypoint) { return waypoint.slip_radius_z; },
     [](coug_interfaces::msg::WayPoint& waypoint, double value) {
       waypoint.slip_radius_z = value;
     }},
}};

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
  QObject::connect(ui_.lat_editor, SIGNAL(valueChanged(double)), this, SLOT(EditorChanged(double)));
  QObject::connect(ui_.lon_editor, SIGNAL(valueChanged(double)), this, SLOT(EditorChanged(double)));
  QObject::connect(ui_.depth_editor, SIGNAL(valueChanged(double)), this,
                   SLOT(EditorChanged(double)));
  QObject::connect(ui_.speed_editor, SIGNAL(valueChanged(double)), this,
                   SLOT(EditorChanged(double)));
  QObject::connect(ui_.capture_radius_editor, SIGNAL(valueChanged(double)), this,
                   SLOT(EditorChanged(double)));
  QObject::connect(ui_.capture_radius_z_editor, SIGNAL(valueChanged(double)), this,
                   SLOT(EditorChanged(double)));
  QObject::connect(ui_.slip_radius_editor, SIGNAL(valueChanged(double)), this,
                   SLOT(EditorChanged(double)));
  QObject::connect(ui_.slip_radius_z_editor, SIGNAL(valueChanged(double)), this,
                   SLOT(EditorChanged(double)));
  QObject::connect(ui_.altitude_mode, SIGNAL(toggled(bool)), this, SLOT(AltitudeModeChanged(bool)));

  // --- Status Updates ---
  QObject::connect(this, SIGNAL(StatusUpdateRequested(int, const QString&)), this,
                   SLOT(HandleStatusUpdate(int, const QString&)), Qt::QueuedConnection);
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

  auto get_or_declare = [this](const std::string& name, const auto& default_value) {
    if (!node_->has_parameter(name)) {
      node_->declare_parameter(name, default_value);
    }
    std::decay_t<decltype(default_value)> value = default_value;
    node_->get_parameter(name, value);
    return value;
  };

  agent_namespaces_ = get_or_declare("agent_namespaces", std::vector<std::string>{});
  const std::string waypoint_topic = get_or_declare("waypoint_topic", std::string("waypoints"));
  const std::string waypoint_map_topic =
      get_or_declare("waypoint_map_topic", std::string("waypoints_map"));
  const std::map<std::string, std::string> services = {
      {"start", get_or_declare("start_service", std::string("base/start"))},
      {"stop", get_or_declare("stop_service", std::string("base/stop"))},
      {"surface", get_or_declare("surface_service", std::string("base/surface"))},
      {"home", get_or_declare("home_service", std::string("base/home"))},
  };
  known_agents_ = std::set<std::string>(agent_namespaces_.begin(), agent_namespaces_.end());
  for (const auto& agent_ns : agent_namespaces_) {
    ui_.agent_selector->addItem(QString::fromStdString(agent_ns));
  }

  default_waypoint_.speed_rpm = get_or_declare("default_speed_rpm", ui_.speed_editor->value());
  default_waypoint_.capture_radius =
      get_or_declare("default_capture_radius", ui_.capture_radius_editor->value());
  default_waypoint_.capture_radius_z =
      get_or_declare("default_capture_radius_z", ui_.capture_radius_z_editor->value());
  default_waypoint_.slip_radius =
      get_or_declare("default_slip_radius", ui_.slip_radius_editor->value());
  default_waypoint_.slip_radius_z =
      get_or_declare("default_slip_radius_z", ui_.slip_radius_z_editor->value());
  applyDefaultsToEditors();

  interface_.initialize(node_, tf_manager_, agent_namespaces_, waypoint_topic, waypoint_map_topic,
                        services, [this](AgentInterface::Status level, const std::string& message) {
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

  swri_transform_util::Transform fixed_T_wgs84;
  if (!tf_manager_->GetTransform(target_frame_, swri_transform_util::_wgs84_frame, fixed_T_wgs84)) {
    return;
  }

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  if (!curr_agent_.empty()) {
    drawWaypointCircles(store_.getWaypoints(curr_agent_), fixed_T_wgs84);
  }

  glLineWidth(2);

  for (const auto& [agent, waypoints] : store_.getAllWaypoints()) {
    if (agent != curr_agent_ && isAgentKnown(agent)) {
      drawPath(waypoints, fixed_T_wgs84);
    }
  }
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
  for (const auto& [agent, waypoints] : store_.getAllWaypoints()) {
    if (agent != curr_agent_ && isAgentKnown(agent)) {
      paintLabels(painter, waypoints, fixed_T_wgs84, QColor(255, 255, 255, 191));
    }
  }

  if (!curr_agent_.empty()) {
    const auto& waypoints = store_.getWaypoints(curr_agent_);
    if (!waypoints.empty()) {
      paintPath(painter, waypoints, QColor(Qt::blue), fixed_T_wgs84, selected_idx_);
      paintLabels(painter, waypoints, fixed_T_wgs84, Qt::white);
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
  if (curr_agent_.empty()) {
    return false;
  }

  dragged_idx_ = -1;
  double distance = 0.0;
  int closest_idx = findClosestWaypoint(event->localPos(), distance);

  if (event->button() == Qt::LeftButton) {
    mouse_down_pos_ = event->localPos();
    mouse_down_time_ = QDateTime::currentMSecsSinceEpoch();

    if (distance < kHitRadiusPx) {
      dragged_idx_ = closest_idx;
      return true;
    }
  } else if (event->button() == Qt::RightButton) {
    if (distance < kHitRadiusPx) {
      store_.removeWaypoint(curr_agent_, closest_idx);

      if (selected_idx_ == closest_idx) {
        deselectWaypoint();
      }
      return true;
    }
  }
  return false;
}

bool CougWaypointsPlugin::handleMouseRelease(QMouseEvent* event) {
  if (curr_agent_.empty()) {
    return false;
  }

  qreal distance = QLineF(mouse_down_pos_, event->localPos()).length();
  qint64 press_duration_ms = QDateTime::currentMSecsSinceEpoch() - mouse_down_time_;

  bool is_click = distance <= kClickMaxDistPx && press_duration_ms < kClickMaxDurationMs;

  if (dragged_idx_ != -1) {
    if (is_click) {
      const auto& waypoints = store_.getWaypoints(curr_agent_);
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

      store_.addWaypoint(curr_agent_, waypoint);
    }
  }

  dragged_idx_ = -1;
  return false;
}

bool CougWaypointsPlugin::handleMouseMove(QMouseEvent* event) {
  if (curr_agent_.empty()) {
    return false;
  }

  if (dragged_idx_ >= 0) {
    if (selected_idx_ != -1) {
      deselectWaypoint();
    }

    geographic_msgs::msg::GeoPoint geo_point;
    if (glToWgs84(event->localPos(), geo_point)) {
      store_.modifyWaypoint(curr_agent_, dragged_idx_,
                            [&geo_point](coug_interfaces::msg::WayPoint& waypoint) {
                              waypoint.position.longitude = geo_point.longitude;
                              waypoint.position.latitude = geo_point.latitude;
                            });
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
  } else if (!curr_agent_.empty()) {
    const auto& waypoints = store_.getWaypoints(curr_agent_);
    interface_.publishWaypoints(curr_agent_, waypoints, target_frame_);
    PrintInfo("Published 1 agent(s).");
  } else {
    PrintError("No agent selected.");
  }
}

void CougWaypointsPlugin::Clear() {
  int count = 0;
  if (ui_.apply_all->isChecked()) {
    count = static_cast<int>(store_.getAllWaypoints().size());
    store_.clearAllWaypoints();
  } else if (!curr_agent_.empty()) {
    store_.clearWaypoints(curr_agent_);
    count = 1;
  } else {
    PrintError("No agent selected.");
    return;
  }

  ui_.lat_editor->blockSignals(true);
  ui_.lat_editor->setValue(0.0);
  ui_.lat_editor->blockSignals(false);
  ui_.lon_editor->blockSignals(true);
  ui_.lon_editor->setValue(0.0);
  ui_.lon_editor->blockSignals(false);
  ui_.depth_editor->setValue(0.0);
  dragged_idx_ = -1;
  deselectWaypoint();
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

  if (utils::saveMission(filename.toStdString(), store_.getAllWaypoints(), agent_to_save)) {
    PrintInfo("Saved " + std::to_string(affectedAgentCount(agent_to_save)) + " agent(s).");
  } else {
    PrintError("Failed to save.");
  }
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

  utils::WaypointMap loaded_waypoints;
  if (!utils::loadMission(filename.toStdString(), default_waypoint_, loaded_waypoints,
                          agent_to_load)) {
    PrintError("Failed to load.");
    return;
  }

  for (const auto& [agent, waypoints] : loaded_waypoints) {
    if (isAgentKnown(agent)) {
      store_.setWaypoints(agent, waypoints);
    }
  }

  AgentChanged(QString::fromStdString(curr_agent_));
  PrintInfo("Loaded " + std::to_string(affectedAgentCount(agent_to_load)) + " agent(s).");
}

void CougWaypointsPlugin::AgentChanged(const QString& text) {
  curr_agent_ = text.toStdString();
  deselectWaypoint();

  const auto& waypoints = store_.getWaypoints(curr_agent_);
  if (waypoints.empty()) {
    PrintInfo("Click to add waypoints.");
  } else {
    PrintInfo(curr_agent_ + " (" + std::to_string(waypoints.size()) + " waypoints).");
  }
}

void CougWaypointsPlugin::EditorChanged(double value) {
  if (selected_idx_ < 0) return;

  const QObject* editor = sender();
  const auto binding = std::find_if(
      kEditorBindings.begin(), kEditorBindings.end(),
      [this, editor](const EditorBinding& candidate) { return ui_.*candidate.editor == editor; });
  if (binding == kEditorBindings.end()) {
    return;
  }

  if (store_.modifyWaypoint(
          curr_agent_, selected_idx_,
          [&](coug_interfaces::msg::WayPoint& waypoint) { binding->set(waypoint, value); })) {
    map_canvas_->update();
  }
}

void CougWaypointsPlugin::AltitudeModeChanged(bool checked) {
  if (selected_idx_ < 0) return;

  double new_altitude = 0.0;
  const bool changed = store_.modifyWaypoint(
      curr_agent_, selected_idx_, [&](coug_interfaces::msg::WayPoint& waypoint) {
        waypoint.mode = checked ? coug_interfaces::msg::WayPoint::ALTITUDE
                                : coug_interfaces::msg::WayPoint::DEPTH;
        new_altitude =
            checked ? std::abs(waypoint.position.altitude) : -std::abs(waypoint.position.altitude);
        waypoint.position.altitude = new_altitude;
      });
  if (!changed) return;

  if (checked) {
    ui_.depth_editor->setMinimum(0.0);
    ui_.depth_editor->setMaximum(kDepthEditorLimit);
  } else {
    ui_.depth_editor->setMinimum(-kDepthEditorLimit);
    ui_.depth_editor->setMaximum(0.0);
  }
  ui_.depth_editor->blockSignals(true);
  ui_.depth_editor->setValue(new_altitude);
  ui_.depth_editor->blockSignals(false);
  map_canvas_->update();
}

bool CougWaypointsPlugin::isAgentKnown(const std::string& agent) const {
  return known_agents_.count(agent) > 0;
}

bool CougWaypointsPlugin::resolveSelectedAgent(std::string& agent) {
  if (ui_.apply_all->isChecked()) {
    agent.clear();
    return true;
  }
  if (curr_agent_.empty()) {
    PrintError("No agent selected.");
    return false;
  }
  agent = curr_agent_;
  return true;
}

int CougWaypointsPlugin::affectedAgentCount(const std::string& agent) const {
  return agent.empty() ? static_cast<int>(store_.getAllWaypoints().size()) : 1;
}

void CougWaypointsPlugin::applyDefaultsToEditors() {
  for (const auto& binding : kEditorBindings) {
    setEditorValue(ui_.*binding.editor, binding.get(default_waypoint_));
  }
}

void CougWaypointsPlugin::populateEditors(const coug_interfaces::msg::WayPoint& waypoint) {
  const bool is_altitude = waypoint.mode == coug_interfaces::msg::WayPoint::ALTITUDE;

  ui_.altitude_mode->blockSignals(true);
  ui_.altitude_mode->setChecked(is_altitude);
  ui_.altitude_mode->setEnabled(true);
  ui_.altitude_mode->blockSignals(false);

  if (is_altitude) {
    ui_.depth_editor->setMinimum(0.0);
    ui_.depth_editor->setMaximum(kDepthEditorLimit);
  } else {
    ui_.depth_editor->setMinimum(-kDepthEditorLimit);
    ui_.depth_editor->setMaximum(0.0);
  }

  for (const auto& binding : kEditorBindings) {
    QDoubleSpinBox* editor = ui_.*binding.editor;
    editor->setEnabled(true);
    setEditorValue(editor, binding.get(waypoint));
  }
}

void CougWaypointsPlugin::deselectWaypoint() {
  selected_idx_ = -1;

  for (const auto& binding : kEditorBindings) {
    (ui_.*binding.editor)->setEnabled(false);
  }

  ui_.altitude_mode->blockSignals(true);
  ui_.altitude_mode->setChecked(false);
  ui_.altitude_mode->setEnabled(false);
  ui_.altitude_mode->blockSignals(false);
  ui_.depth_editor->setMinimum(-kDepthEditorLimit);
  ui_.depth_editor->setMaximum(0.0);
}

void CougWaypointsPlugin::publishAll() {
  for (const auto& agent : agent_namespaces_) {
    interface_.publishWaypoints(agent, store_.getWaypoints(agent), target_frame_);
  }
  PrintInfo("Published to " + std::to_string(agent_namespaces_.size()) + " agent(s).");
}

void CougWaypointsPlugin::callService(const std::string& cmd) {
  if (ui_.apply_all->isChecked()) {
    if (agent_namespaces_.empty()) {
      PrintError("No agents configured.");
      return;
    }
    interface_.callService(cmd, agent_namespaces_, true);
  } else if (!curr_agent_.empty()) {
    interface_.callService(cmd, {curr_agent_}, false);
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

QPointF CougWaypointsPlugin::wgs84ToFixedPoint(
    const coug_interfaces::msg::WayPoint& waypoint,
    const swri_transform_util::Transform& fixed_T_wgs84) {
  const tf2::Vector3 point = utils::wgs84ToFixed(waypoint, fixed_T_wgs84);
  return QPointF(point.x(), point.y());
}

QPointF CougWaypointsPlugin::wgs84ToGl(const coug_interfaces::msg::WayPoint& waypoint,
                                       const swri_transform_util::Transform& fixed_T_wgs84) {
  return map_canvas_->FixedFrameToMapGlCoord(wgs84ToFixedPoint(waypoint, fixed_T_wgs84));
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
  const auto& waypoints = store_.getWaypoints(curr_agent_);

  for (size_t i = 0; i < waypoints.size(); i++) {
    QPointF transformed = wgs84ToGl(waypoints[i], fixed_T_wgs84);

    double dist = QLineF(transformed, point).length();
    if (dist < distance) {
      distance = dist;
      closest_idx = static_cast<int>(i);
    }
  }
  return closest_idx;
}

void CougWaypointsPlugin::drawPath(const std::vector<coug_interfaces::msg::WayPoint>& waypoints,
                                   const swri_transform_util::Transform& fixed_T_wgs84) {
  std::vector<QPointF> points;
  points.reserve(waypoints.size());
  for (const auto& waypoint : waypoints) {
    points.push_back(wgs84ToFixedPoint(waypoint, fixed_T_wgs84));
  }

  glColor4f(1.0, 1.0, 1.0, 0.75);
  glBegin(GL_LINE_STRIP);
  for (const auto& point : points) {
    glVertex2d(point.x(), point.y());
  }
  glEnd();

  glColor4f(0.5, 0.5, 0.5, 0.75);
  glPointSize(20);
  glBegin(GL_POINTS);
  for (const auto& point : points) {
    glVertex2d(point.x(), point.y());
  }
  glEnd();
}

void CougWaypointsPlugin::drawWaypointCircles(
    const std::vector<coug_interfaces::msg::WayPoint>& waypoints,
    const swri_transform_util::Transform& fixed_T_wgs84) {
  for (const auto& waypoint : waypoints) {
    QPointF center = wgs84ToFixedPoint(waypoint, fixed_T_wgs84);
    double center_x = center.x();
    double center_y = center.y();
    double capture_radius = waypoint.capture_radius;
    double slip_radius = waypoint.slip_radius;

    drawFilledCircle(center_x, center_y, slip_radius, 0.118f, 0.565f, 1.0f, 0.12f);
    drawCircleOutline(center_x, center_y, slip_radius, 0.118f, 0.565f, 1.0f, 0.65f);
    drawFilledCircle(center_x, center_y, capture_radius, 1.0f, 0.55f, 0.0f, 0.18f);
    drawCircleOutline(center_x, center_y, capture_radius, 1.0f, 0.55f, 0.0f, 0.75f);
  }
}

void CougWaypointsPlugin::drawFilledCircle(double center_x, double center_y, double radius,
                                           float red, float green, float blue, float alpha) {
  glColor4f(red, green, blue, alpha);
  glBegin(GL_TRIANGLE_FAN);
  glVertex2d(center_x, center_y);
  for (int i = 0; i <= kCircleSegments; ++i) {
    double theta = 2.0 * M_PI * static_cast<double>(i) / kCircleSegments;
    glVertex2d(center_x + radius * std::cos(theta), center_y + radius * std::sin(theta));
  }
  glEnd();
}

void CougWaypointsPlugin::drawCircleOutline(double center_x, double center_y, double radius,
                                            float red, float green, float blue, float alpha) {
  glColor4f(red, green, blue, alpha);
  glBegin(GL_LINE_LOOP);
  for (int i = 0; i < kCircleSegments; ++i) {
    double theta = 2.0 * M_PI * static_cast<double>(i) / kCircleSegments;
    glVertex2d(center_x + radius * std::cos(theta), center_y + radius * std::sin(theta));
  }
  glEnd();
}

void CougWaypointsPlugin::paintLabels(QPainter* painter,
                                      const std::vector<coug_interfaces::msg::WayPoint>& waypoints,
                                      const swri_transform_util::Transform& fixed_T_wgs84,
                                      const QColor& color) {
  for (size_t i = 0; i < waypoints.size(); i++) {
    QPointF gl_point = wgs84ToGl(waypoints[i], fixed_T_wgs84);

    painter->setPen(QPen(color));

    QPointF depth_text_corner(gl_point.x() - 50, gl_point.y() + 15);
    QRectF depth_text_rect(depth_text_corner, QSizeF(100, 20));
    QString depth_text =
        waypoints[i].mode == coug_interfaces::msg::WayPoint::ALTITUDE
            ? "ALT " + QString::number(waypoints[i].position.altitude, 'f', 1) + "m"
            : QString::number(waypoints[i].position.altitude, 'f', 1) + "m";
    painter->drawText(depth_text_rect, Qt::AlignHCenter | Qt::AlignTop, depth_text);

    QPointF speed_text_corner(gl_point.x() - 50, gl_point.y() + 33);
    QRectF speed_text_rect(speed_text_corner, QSizeF(100, 20));
    QString speed_text = QString::number(waypoints[i].speed_rpm, 'f', 0) + "RPM";
    painter->drawText(speed_text_rect, Qt::AlignHCenter | Qt::AlignTop, speed_text);

    painter->setPen(QPen(color == Qt::white ? Qt::black : color));
    QPointF num_corner(gl_point.x() - 20, gl_point.y() - 20);
    QRectF num_rect(num_corner, QSizeF(40, 40));
    painter->drawText(num_rect, Qt::AlignHCenter | Qt::AlignVCenter, QString::number(i + 1));
  }
}

void CougWaypointsPlugin::paintPath(QPainter* painter,
                                    const std::vector<coug_interfaces::msg::WayPoint>& waypoints,
                                    const QColor& color,
                                    const swri_transform_util::Transform& fixed_T_wgs84,
                                    int selected_idx) {
  QVector<QPointF> points;
  for (const auto& waypoint : waypoints) {
    points.push_back(wgs84ToGl(waypoint, fixed_T_wgs84));
  }

  QPen pen(color, 2);
  painter->setPen(pen);
  painter->drawPolyline(points);
  for (int i = 0; i < points.size(); ++i) {
    if (i == selected_idx) {
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

}  // namespace coug_mapviz
