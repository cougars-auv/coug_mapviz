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
#include <coug_interfaces/msg/way_point_list.hpp>
#include <coug_mapviz/coug_waypoint_manager.hpp>
#include <geographic_msgs/msg/geo_point.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
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
   * @brief Loads plugin configuration from YAML.
   * @param node The YAML node containing configuration.
   * @param path Path to the configuration file.
   */
  void LoadConfig(const YAML::Node& node, const std::string& path) override;

  /**
   * @brief Saves plugin configuration to YAML.
   * @param emitter The YAML emitter.
   * @param path Path to the configuration file.
   */
  void SaveConfig(YAML::Emitter& emitter, const std::string& path) override;

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
   * @brief Publishes waypoints for the current agent (or all agents if apply_all is checked).
   */
  void PublishWaypoints();

  /**
   * @brief Clears waypoints for the current agent (or all agents if apply_all is checked).
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
   * @brief Updates the active agent and resets interaction state when the selector changes.
   * @param text The newly selected agent namespace.
   */
  void AgentChanged(const QString& text);

  // --- Mission Control ---
  /**
   * @brief Starts mission execution on the current agent (or all agents if apply_all is checked).
   */
  void Start() { dispatchCommand("start"); }

  /**
   * @brief Stops mission execution on the current agent (or all agents if apply_all is checked).
   */
  void Stop() { dispatchCommand("stop"); }

  /**
   * @brief Commands the agent to surface at its current position (or all agents if apply_all is
   * checked).
   */
  void Surface() { dispatchCommand("surface"); }

  /**
   * @brief Commands the agent to navigate to its home waypoint (or all agents if apply_all is
   * checked).
   */
  void Home() { dispatchCommand("home"); }

  /**
   * @brief Shows or hides waypoints when the visibility checkbox changes.
   * @param visible True to show, false to hide.
   */
  void VisibilityChanged(bool visible);

  /**
   * @brief Updates the depth of the selected waypoint.
   * @param value New depth value.
   */
  void DepthChanged(double value);

 private:
  // --- Components ---
  Ui::coug_waypoints_config ui_;
  QWidget* config_widget_;
  mapviz::MapCanvas* map_canvas_;

  // --- ROS Interface ---
  std::map<std::string, rclcpp::Publisher<coug_interfaces::msg::WayPointList>::SharedPtr>
      publishers_;
  std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr>
      map_publishers_;
  std::map<std::string, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> clients_;
  CougWaypointManager manager_;
  std::string current_agent_;
  std::vector<std::string> agent_namespaces_;
  std::string waypoint_topic_;
  std::string waypoints_map_topic_;

  // --- Interaction State ---
  int selected_point_;
  int dragged_point_;
  bool is_mouse_down_;
  QPointF mouse_down_pos_;
  qint64 mouse_down_time_;

  // --- Helpers ---
  /**
   * @brief Publishes waypoints for all agents currently in the manager.
   */
  void PublishAll();

  /**
   * @brief Publishes waypoints for one agent; derives the topic as agent + "/waypoints".
   * @param agent The agent namespace.
   * @param wps The waypoints to publish.
   */
  void PublishAgent(const std::string& agent,
                    const std::vector<geographic_msgs::msg::GeoPoint>& wps);

  /**
   * @brief Calls a Trigger service, creating the client lazily on first use.
   * @param service_name The fully-qualified service name.
   */
  void callTrigger(const std::string& service_name);

  /**
   * @brief Calls cmd on the current agent, or all agents if apply_all is checked.
   * @param cmd The command string to dispatch (e.g. "start", "stop", "surface", "home").
   */
  void dispatchCommand(const std::string& cmd);

  /**
   * @brief Returns true if agent is in the agent_namespaces_ list.
   * @param agent The agent namespace to look up.
   * @return True if the agent is known, false otherwise.
   */
  bool IsAgentKnown(const std::string& agent);

  /**
   * @brief Returns the index of the waypoint within 15px of point, or -1 if none.
   * @param point The screen-space position to test.
   * @param distance Output: distance in pixels to the closest waypoint.
   * @return Index of the closest waypoint, or -1.
   */
  int GetClosestPoint(const QPointF& point, double& distance);

  /**
   * @brief Draws non-current agent paths in white using OpenGL.
   * @param wps The waypoint list to draw.
   * @param transform Transform from the waypoint frame to the map frame.
   */
  void DrawPath(const std::vector<geographic_msgs::msg::GeoPoint>& wps,
                const swri_transform_util::Transform& transform);

  /**
   * @brief Paints waypoint index numbers and depth labels using QPainter.
   * @param painter The QPainter instance.
   * @param wps The waypoint list to label.
   * @param transform Transform from the waypoint frame to screen space.
   * @param color Label color.
   */
  void PaintLabels(QPainter* painter, const std::vector<geographic_msgs::msg::GeoPoint>& wps,
                   const swri_transform_util::Transform& transform, const QColor& color);

  /**
   * @brief Paints path lines and dots; highlights selected_index in yellow using QPainter.
   * @param painter The QPainter instance.
   * @param wps The waypoint list to paint.
   * @param color Base color for the path.
   * @param transform Transform from the waypoint frame to screen space.
   * @param selected_index Index of the selected waypoint to highlight, or -1 for none.
   */
  void PaintPath(QPainter* painter, const std::vector<geographic_msgs::msg::GeoPoint>& wps,
                 const QColor& color, const swri_transform_util::Transform& transform,
                 int selected_index = -1);
};

}  // namespace coug_mapviz
