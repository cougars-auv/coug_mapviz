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

#pragma once

#include <swri_transform_util/transform.h>

#include <QColor>
#include <QPointF>
#include <coug_interfaces/msg/way_point.hpp>
#include <vector>

class QPainter;

namespace mapviz {
class MapCanvas;
}

namespace coug_mapviz::utils {

class WaypointRenderer {
 public:
  explicit WaypointRenderer(mapviz::MapCanvas* map_canvas);

  QPointF waypointToGl(const coug_interfaces::msg::WayPoint& waypoint,
                       const swri_transform_util::Transform& fixed_T_wgs84) const;

  void paintCircles(QPainter* painter, const std::vector<coug_interfaces::msg::WayPoint>& waypoints,
                    const swri_transform_util::Transform& fixed_T_wgs84) const;

  void paintLabels(QPainter* painter, const std::vector<coug_interfaces::msg::WayPoint>& waypoints,
                   const swri_transform_util::Transform& fixed_T_wgs84, const QColor& color) const;

  void paintPath(QPainter* painter, const std::vector<coug_interfaces::msg::WayPoint>& waypoints,
                 const QColor& color, const swri_transform_util::Transform& fixed_T_wgs84,
                 int selected_idx = -1) const;

 private:
  static QPointF waypointToFixedPoint(const coug_interfaces::msg::WayPoint& waypoint,
                                      const swri_transform_util::Transform& fixed_T_wgs84);

  mapviz::MapCanvas* map_canvas_;
};

}  // namespace coug_mapviz::utils
