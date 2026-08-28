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

#include <QBrush>
#include <QPainter>
#include <QPen>
#include <QVector>
#include <coug_mapviz/utils/waypoint_renderer.hpp>

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

const QColor kSlipCircleColor(30, 144, 255, 166);
const QColor kSlipCircleFillColor(30, 144, 255, 30);
const QColor kCaptureCircleColor(255, 140, 0, 191);
const QColor kCaptureCircleFillColor(255, 140, 0, 46);
const QColor kActivePath(Qt::blue);
const QColor kActiveLabel(Qt::white);
const QColor kInactivePath(200, 200, 200, 191);
const QColor kInactiveLabel(255, 255, 255, 191);
const QColor kActiveMarker(Qt::cyan);
const QColor kInactiveMarker(Qt::gray);
const QColor kSelectedMarker(Qt::yellow);

QRectF labelRect(const QPointF& point, int y_offset) {
  return {QPointF(point.x() - kLabelOffsetXPx, point.y() + y_offset),
          QSizeF(kLabelWidthPx, kLabelHeightPx)};
}

}  // namespace

WaypointRenderer::WaypointRenderer(mapviz::MapCanvas* map_canvas) : map_canvas_(map_canvas) {}

QPointF WaypointRenderer::fixedToGl(const QPointF& fixed_point) const {
  return map_canvas_->FixedFrameToMapGlCoord(fixed_point);
}

void WaypointRenderer::paintWaypoints(QPainter* painter, const std::vector<WayPoint>& waypoints,
                                      bool active, int selected_idx) const {
  const QColor& path_color = active ? kActivePath : kInactivePath;
  const QColor& label_color = active ? kActiveLabel : kInactiveLabel;
  QVector<QPointF> points;
  for (const auto& waypoint : waypoints) {
    points.push_back(fixedToGl(QPointF(waypoint.position.x, waypoint.position.y)));
  }
  painter->setPen(QPen(path_color, kPathWidthPx));
  painter->drawPolyline(points);

  for (int i = 0; i < points.size(); ++i) {
    const auto& waypoint = waypoints[static_cast<size_t>(i)];
    if (active) {
      const QPointF center(waypoint.position.x, waypoint.position.y);
      const auto radius = [&](double meters) {
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

    const QColor marker = i == selected_idx ? kSelectedMarker
                          : active          ? kActiveMarker
                                            : kInactiveMarker;
    painter->setPen(QPen(marker, kMarkerSizePx, Qt::SolidLine, Qt::RoundCap));
    painter->drawPoint(points[i]);

    painter->setPen(QPen(label_color));
    const QString depth = waypoint.mode == WayPoint::ALTITUDE
                              ? "ALT " + QString::number(waypoint.position.z, 'f', 1) + "m"
                              : QString::number(waypoint.position.z, 'f', 1) + "m";
    painter->drawText(labelRect(points[i], kDepthOffsetYPx), Qt::AlignHCenter | Qt::AlignTop,
                      depth);
    painter->drawText(labelRect(points[i], kSpeedOffsetYPx), Qt::AlignHCenter | Qt::AlignTop,
                      QString::number(waypoint.speed_rpm, 'f', 0) + "rpm");
    painter->setPen(QPen(label_color == Qt::white ? Qt::black : label_color));
    const QRectF index_rect(
        QPointF(points[i].x() - kIndexSizePx / 2.0, points[i].y() - kIndexSizePx / 2.0),
        QSizeF(kIndexSizePx, kIndexSizePx));
    painter->drawText(index_rect, Qt::AlignHCenter | Qt::AlignVCenter, QString::number(i + 1));
  }
}
}  // namespace coug_mapviz::utils
