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

#include <mapviz/map_canvas.h>
#include <qcolor.h>
#include <qnamespace.h>
#include <qobject.h>
#include <qpaintdevice.h>
#include <qpoint.h>
#include <qvector.h>

#include <QBrush>
#include <QPainter>
#include <QPen>
#include <QVector>
#include <coug_mapviz/utils/waypoint_renderer.hpp>
#include <cstddef>
#include <vector>

#include "coug_interfaces/msg/way_point.hpp"

namespace coug_mapviz::utils {

using coug_interfaces::msg::WayPoint;

namespace {

constexpr float kMarkerSizePx = 20.0F;
constexpr int kPathWidthPx = 2;

constexpr int kLabelWidthPx = 100;
constexpr int kLabelHeightPx = 20;
constexpr int kLabelOffsetXPx = 50;
constexpr int kDepthOffsetYPx = 15;
constexpr int kSpeedOffsetYPx = 33;
constexpr int kIndexSizePx = 40;

QColor const kSlipCircleColor(30, 144, 255, 166);
QColor const kSlipCircleFillColor(30, 144, 255, 30);
QColor const kCaptureCircleColor(255, 140, 0, 191);
QColor const kCaptureCircleFillColor(255, 140, 0, 46);
QColor const kActivePath(Qt::blue);
QColor const kActiveLabel(Qt::white);
QColor const kInactivePath(200, 200, 200, 191);
QColor const kInactiveLabel(255, 255, 255, 191);
QColor const kActiveMarker(Qt::cyan);
QColor const kInactiveMarker(Qt::gray);
QColor const kSelectedMarker(Qt::yellow);

auto labelRect(QPointF const& point, int y_offset) -> QRectF {
  return {QPointF(point.x() - kLabelOffsetXPx, point.y() + y_offset),
          QSizeF(kLabelWidthPx, kLabelHeightPx)};
}

}  // namespace

WaypointRenderer::WaypointRenderer(mapviz::MapCanvas* map_canvas) : map_canvas_(map_canvas) {}

auto WaypointRenderer::fixedToGl(QPointF const& fixed_point) const -> QPointF {
  return map_canvas_->FixedFrameToMapGlCoord(fixed_point);
}

void WaypointRenderer::paintWaypoints(QPainter* painter, std::vector<WayPoint> const& waypoints,
                                      bool active, int selected_idx) const {
  QColor const& path_color = active ? kActivePath : kInactivePath;
  QColor const& label_color = active ? kActiveLabel : kInactiveLabel;
  QVector<QPointF> points;
  for (auto const& waypoint : waypoints) {
    points.push_back(fixedToGl(QPointF(waypoint.position.x, waypoint.position.y)));
  }
  painter->setPen(QPen(path_color, kPathWidthPx));
  painter->drawPolyline(points);

  for (int i = 0; i < points.size(); ++i) {
    auto const& waypoint = waypoints[static_cast<size_t>(i)];
    if (active) {
      QPointF const center(waypoint.position.x, waypoint.position.y);
      auto const radius = [&](double meters) {
        return QLineF(points[i], fixedToGl(QPointF(center.x() + meters, center.y()))).length();
      };
      painter->setPen(QPen(kSlipCircleColor, 1.5));
      painter->setBrush(QBrush(kSlipCircleFillColor));
      painter->drawEllipse(points[i], radius(waypoint.slip_radius), radius(waypoint.slip_radius));
      painter->setPen(QPen(kCaptureCircleColor, 1.5));
      painter->setBrush(QBrush(kCaptureCircleFillColor));
      painter->drawEllipse(points[i], radius(waypoint.capture_radius),
                           radius(waypoint.capture_radius));
    }

    QColor marker = kInactiveMarker;
    if (i == selected_idx) {
      marker = kSelectedMarker;
    } else if (active) {
      marker = kActiveMarker;
    }
    painter->setPen(QPen(marker, kMarkerSizePx, Qt::SolidLine, Qt::RoundCap));
    painter->drawPoint(points[i]);

    painter->setPen(QPen(label_color));
    QString const depth = waypoint.mode == WayPoint::ALTITUDE
                              ? "ALT " + QString::number(waypoint.position.z, 'f', 1) + "m"
                              : QString::number(waypoint.position.z, 'f', 1) + "m";
    painter->drawText(labelRect(points[i], kDepthOffsetYPx), Qt::AlignHCenter | Qt::AlignTop,
                      depth);
    painter->drawText(labelRect(points[i], kSpeedOffsetYPx), Qt::AlignHCenter | Qt::AlignTop,
                      QString::number(waypoint.speed_rpm, 'f', 0) + "rpm");
    painter->setPen(QPen(label_color == Qt::white ? Qt::black : label_color));
    QRectF const index_rect(
        QPointF(points[i].x() - kIndexSizePx / 2.0, points[i].y() - kIndexSizePx / 2.0),
        QSizeF(kIndexSizePx, kIndexSizePx));
    painter->drawText(index_rect, Qt::AlignHCenter | Qt::AlignVCenter, QString::number(i + 1));
  }
}
}  // namespace coug_mapviz::utils
