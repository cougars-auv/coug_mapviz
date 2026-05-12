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
 * @file coug_utils_plugin.cpp
 * @brief Implementation of the CougUtilsPlugin.
 * @author Nelson Durrant
 * @date May 2026
 */

#include <coug_mapviz/coug_utils_plugin.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(coug_mapviz::CougUtilsPlugin, mapviz::MapvizPlugin)

namespace coug_mapviz {

CougUtilsPlugin::CougUtilsPlugin() : MapvizPlugin(), ui_(), config_widget_(new QWidget()) {
  ui_.setupUi(config_widget_);

  QPalette p(config_widget_->palette());
  p.setColor(QPalette::Window, Qt::white);
  config_widget_->setPalette(p);
  QPalette p3(ui_.status->palette());
  p3.setColor(QPalette::Text, Qt::darkGreen);
  ui_.status->setPalette(p3);

  QObject::connect(ui_.agent_selector, SIGNAL(currentTextChanged(const QString&)), this,
                   SLOT(AgentChanged(const QString&)));

  QObject::connect(ui_.arm, SIGNAL(clicked()), this, SLOT(Arm()));
  QObject::connect(ui_.disarm, SIGNAL(clicked()), this, SLOT(Disarm()));
  QObject::connect(ui_.dvl_on, SIGNAL(clicked()), this, SLOT(DvlOn()));
  QObject::connect(ui_.dvl_off, SIGNAL(clicked()), this, SLOT(DvlOff()));
}

bool CougUtilsPlugin::Initialize(QGLWidget* canvas) {
  (void)canvas;

  try {
    node_->declare_parameter("agent_namespaces", std::vector<std::string>{});
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&) {
  }
  std::vector<std::string> agent_namespaces;
  node_->get_parameter("agent_namespaces", agent_namespaces);
  for (const auto& ns : agent_namespaces) {
    ui_.agent_selector->addItem(QString::fromStdString(ns));
  }

  initialized_ = true;
  return true;
}

void CougUtilsPlugin::AgentChanged(const QString& text) { current_agent_ = text.toStdString(); }

void CougUtilsPlugin::Arm() {
  ui_.armed_indicator->setStyleSheet("background-color: #00cc00; border-radius: 6px;");
}

void CougUtilsPlugin::Disarm() {
  ui_.armed_indicator->setStyleSheet("background-color: #cc0000; border-radius: 6px;");
}

void CougUtilsPlugin::DvlOn() {
  ui_.dvl_indicator->setStyleSheet("background-color: #00cc00; border-radius: 6px;");
}

void CougUtilsPlugin::DvlOff() {
  ui_.dvl_indicator->setStyleSheet("background-color: #cc0000; border-radius: 6px;");
}

void CougUtilsPlugin::Draw(double x, double y, double scale) {
  (void)x;
  (void)y;
  (void)scale;
}

void CougUtilsPlugin::Paint(QPainter* painter, double x, double y, double scale) {
  (void)painter;
  (void)x;
  (void)y;
  (void)scale;
}

void CougUtilsPlugin::LoadConfig(const YAML::Node& node, const std::string& path) {
  (void)node;
  (void)path;
}

void CougUtilsPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path) {
  (void)emitter;
  (void)path;
}

QWidget* CougUtilsPlugin::GetConfigWidget(QWidget* parent) {
  config_widget_->setParent(parent);
  return config_widget_;
}

void CougUtilsPlugin::PrintError(const std::string& message) {
  PrintErrorHelper(ui_.status, message, 1.0);
}

void CougUtilsPlugin::PrintInfo(const std::string& message) {
  PrintInfoHelper(ui_.status, message, 1.0);
}

void CougUtilsPlugin::PrintWarning(const std::string& message) {
  PrintWarningHelper(ui_.status, message, 1.0);
}

}  // namespace coug_mapviz
