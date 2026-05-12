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
 * @file coug_utils_plugin.hpp
 * @brief MapViz plugin for per-agent utility commands.
 * @author Nelson Durrant
 * @date May 2026
 */

#pragma once

#include <mapviz/mapviz_plugin.h>
#include <ui_coug_utils_config.h>

#include <QGLWidget>
#include <QPainter>
#include <QWidget>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace coug_mapviz {

/**
 * @class CougUtilsPlugin
 * @brief A MapViz plugin that provides per-agent utility commands.
 */
class CougUtilsPlugin : public mapviz::MapvizPlugin {
  Q_OBJECT

 public:
  // --- Lifecycle & Mapviz Interface ---
  CougUtilsPlugin();
  ~CougUtilsPlugin() override = default;

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
   * @return Always false.
   */
  bool SupportsPainting() override { return false; }

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

 protected Q_SLOTS:
  // --- UI Slots ---
  /**
   * @brief Updates the active agent when the selector changes.
   * @param text The newly selected agent namespace.
   */
  void AgentChanged(const QString& text);

  /**
   * @brief Arms the selected agent.
   */
  void Arm();

  /**
   * @brief Disarms the selected agent.
   */
  void Disarm();

  /**
   * @brief Enables the DVL on the selected agent.
   */
  void DvlOn();

  /**
   * @brief Disables the DVL on the selected agent.
   */
  void DvlOff();

 private:
  // --- Components ---
  Ui::coug_utils_config ui_;
  QWidget* config_widget_;
  std::string current_agent_;
};

}  // namespace coug_mapviz
