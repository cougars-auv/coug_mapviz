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
 * @file coug_waypoints_plugin.hpp
 * @brief MapViz plugin for multi-agent waypoint mission planning.
 * @author Nelson Durrant
 * @date May 2026
 */

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
#include <coug_mapviz/coug_comms_client.hpp>
#include <coug_mapviz/coug_waypoint_manager.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace coug_mapviz {

/**
 * @class CougWaypointsPlugin
 * @brief A MapViz plugin for multi-agent waypoint mission planning.
 */
class CougWaypointsPlugin : public mapviz::MapvizPlugin {
  Q_OBJECT

 public:
  // --- Lifecycle & Mapviz Interface ---
  CougWaypointsPlugin();
  ~CougWaypointsPlugin() override;

  /**
   * @brief Initializes the plugin with the map canvas.
   * @param canvas The OpenGL widget to draw on.
   * @return True if initialization succeeds.
   */
  bool Initialize(QGLWidget* canvas) override;

  /**
   * @brief Shuts down the plugin and releases resources.
   */
  void Shutdown() override {}

  /**
   * @brief Draws OpenGL content in the map frame.
   * @param x Camera X position.
   * @param y Camera Y position.
   * @param scale Map scale factor.
   */
  void Draw(double x, double y, double scale) override;

  /**
   * @brief Paints 2D overlays.
   * @param painter The QPainter instance.
   * @param x Camera X position.
   * @param y Camera Y position.
   * @param scale Map scale factor.
   */
  void Paint(QPainter* painter, double x, double y, double scale) override;

  /**
   * @brief Handles coordinate transformations (unused).
   */
  void Transform() override {}

  // --- Configuration ---
  /**
   * @brief Loads plugin configuration from YAML (unused).
   * @param node The YAML node containing configuration.
   * @param path Path to the configuration file.
   */
  void LoadConfig(const YAML::Node& node, const std::string& path) override {
    (void)node;
    (void)path;
  }

  /**
   * @brief Saves plugin configuration to YAML (unused).
   * @param emitter The YAML emitter.
   * @param path Path to the configuration file.
   */
  void SaveConfig(YAML::Emitter& emitter, const std::string& path) override {
    (void)emitter;
    (void)path;
  }

  /**
   * @brief Creates and returns the configuration widget.
   * @param parent Parent widget.
   * @return Pointer to the configuration widget.
   */
  QWidget* GetConfigWidget(QWidget* parent) override;

  /**
   * @brief Indicates if the plugin uses QPainter.
   * @return Always true.
   */
  bool SupportsPainting() override { return true; }

 protected:
  // --- Error Handling ---
  /**
   * @brief Displays an error message in the status widget.
   * @param message The error message.
   */
  void PrintError(const std::string& message) override;

  /**
   * @brief Displays an info message in the status widget.
   * @param message The info message.
   */
  void PrintInfo(const std::string& message) override;

  /**
   * @brief Displays a warning message in the status widget.
   * @param message The warning message.
   */
  void PrintWarning(const std::string& message) override;

  // --- Event Handling ---
  /**
   * @brief Filters Qt events for mouse interaction.
   * @param object The object receiving the event.
   * @param event The event being processed.
   * @return True if the event was handled.
   */
  bool eventFilter(QObject* object, QEvent* event) override;

  /**
   * @brief Handles mouse press events for adding/selecting waypoints.
   * @param event The mouse event.
   * @return True if handled.
   */
  bool handleMousePress(QMouseEvent* event);

  /**
   * @brief Handles mouse release events for placing/modifying waypoints.
   * @param event The mouse event.
   * @return True if handled.
   */
  bool handleMouseRelease(QMouseEvent* event);

  /**
   * @brief Handles mouse move events for dragging waypoints.
   * @param event The mouse event.
   * @return True if handled.
   */
  bool handleMouseMove(QMouseEvent* event);

 protected Q_SLOTS:
  // --- UI Slots ---
  /**
   * @brief Publishes the selected agent's waypoints, or all if apply_all is set.
   */
  void PublishWaypoints();

  /**
   * @brief Clears the selected agent's waypoints, or all if apply_all is set.
   */
  void Clear();

  /**
   * @brief Opens a dialog to save waypoints to a file.
   */
  void SaveWaypoints();

  /**
   * @brief Opens a dialog to load waypoints from a file.
   */
  void LoadWaypoints();

  /**
   * @brief Updates the active agent and resets interaction state.
   * @param text The newly selected agent namespace.
   */
  void AgentChanged(const QString& text);

  // --- Mission Control ---
  /**
   * @brief Starts the mission on the selected agent, or all if apply_all is set.
   */
  void Start() { callService("start"); }

  /**
   * @brief Stops the mission on the selected agent, or all if apply_all is set.
   */
  void Stop() { callService("stop"); }

  /**
   * @brief Surfaces the selected agent, or all if apply_all is set.
   */
  void Surface() { callService("surface"); }

  /**
   * @brief Sends the selected agent home, or all if apply_all is set.
   */
  void Home() { callService("home"); }

  /**
   * @brief Shows or hides waypoints when the visibility checkbox changes.
   * @param visible True to show, false to hide.
   */
  void VisibilityChanged(bool visible);

  /**
   * @brief Updates the latitude of the selected waypoint.
   * @param value New latitude in degrees.
   */
  void LatChanged(double value);

  /**
   * @brief Updates the longitude of the selected waypoint.
   * @param value New longitude in degrees.
   */
  void LonChanged(double value);

  /**
   * @brief Updates the depth of the selected waypoint.
   * @param value New depth value.
   */
  void DepthChanged(double value);

  /**
   * @brief Updates the thruster speed of the selected waypoint.
   * @param value New speed in RPM.
   */
  void SpeedChanged(double value);

  /**
   * @brief Toggles altitude (ALT) mode and flips the depth spinbox range.
   * @param checked True for altitude (ALT) mode, false for depth mode.
   */
  void AltitudeModeChanged(bool checked);

 private:
  // --- Components ---
  Ui::coug_waypoints_config ui_;
  QWidget* config_widget_;
  mapviz::MapCanvas* map_canvas_;

  // --- ROS Interface ---
  CougCommsClient comms_;
  CougWaypointManager manager_;
  std::string current_agent_;
  std::vector<std::string> agent_namespaces_;

  // --- Interaction State ---
  int selected_point_;
  int dragged_point_;
  QPointF mouse_down_pos_;
  qint64 mouse_down_time_;

  // --- Helpers ---
  /**
   * @brief Clears the selection and disables the lat/lon/depth/speed editors.
   */
  void deselectWaypoint();

  /**
   * @brief Enables and populates the lat/lon/depth/speed editors from a waypoint.
   * @param wp The waypoint whose values should fill the editors.
   */
  void populateEditors(const CougWaypoint& wp);

  /**
   * @brief Publishes waypoints for all agents currently in the manager.
   */
  void publishAll();

  /**
   * @brief Calls cmd on the current agent, or all agents if apply_all is checked.
   * @param cmd The command string to call (e.g. "start", "stop", "surface", "home").
   */
  void callService(const std::string& cmd);

  /**
   * @brief Returns true if agent is in the agent_namespaces_ list.
   * @param agent The agent namespace to look up.
   * @return True if the agent is known, false otherwise.
   */
  bool isAgentKnown(const std::string& agent);

  /**
   * @brief Projects a waypoint's geographic position into the map's fixed frame.
   * @param wp The waypoint to project.
   * @param transform Transform from the WGS84 frame to the fixed frame.
   * @return The point in fixed-frame coordinates.
   */
  QPointF waypointToFixedFrame(const CougWaypoint& wp,
                               const swri_transform_util::Transform& transform);

  /**
   * @brief Projects a waypoint's geographic position into screen (map GL) coordinates.
   * @param wp The waypoint to project.
   * @param transform Transform from the WGS84 frame to the fixed frame.
   * @return The point in screen-space coordinates.
   */
  QPointF waypointToMapGl(const CougWaypoint& wp, const swri_transform_util::Transform& transform);

  /**
   * @brief Converts a screen-space point to a geographic position.
   * @param screen_point The point in screen (map GL) coordinates.
   * @param geo Output: the corresponding geographic position.
   * @return True if the transform was available, false otherwise.
   */
  bool screenToGeo(const QPointF& screen_point, geographic_msgs::msg::GeoPoint& geo);

  /**
   * @brief Returns the index of the waypoint within 15px of point, or -1 if none.
   * @param point The screen-space position to test.
   * @param distance Output: distance in pixels to the closest waypoint.
   * @return Index of the closest waypoint, or -1.
   */
  int getClosestPoint(const QPointF& point, double& distance);

  /**
   * @brief Draws non-current agent paths in white using OpenGL.
   * @param wps The waypoint list to draw.
   * @param transform Transform from the waypoint frame to the map frame.
   */
  void drawPath(const std::vector<CougWaypoint>& wps,
                const swri_transform_util::Transform& transform);

  /**
   * @brief Paints waypoint index numbers, depth, and speed labels using QPainter.
   * @param painter The QPainter instance.
   * @param wps The waypoint list to label.
   * @param transform Transform from the waypoint frame to screen space.
   * @param color Label color.
   */
  void paintLabels(QPainter* painter, const std::vector<CougWaypoint>& wps,
                   const swri_transform_util::Transform& transform, const QColor& color);

  /**
   * @brief Paints path lines and dots; highlights the selected waypoint in yellow.
   * @param painter The QPainter instance.
   * @param wps The waypoint list to paint.
   * @param color Base color for the path.
   * @param transform Transform from the waypoint frame to screen space.
   * @param selected_index Index of the selected waypoint to highlight, or -1 for none.
   */
  void paintPath(QPainter* painter, const std::vector<CougWaypoint>& wps, const QColor& color,
                 const swri_transform_util::Transform& transform, int selected_index = -1);
};

}  // namespace coug_mapviz
