// Copyright (c) 2026 BYU FROST Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

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
