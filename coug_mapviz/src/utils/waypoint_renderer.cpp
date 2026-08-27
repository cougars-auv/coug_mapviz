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
#include <coug_mapviz/utils/geo_conversions.hpp>
#include <coug_mapviz/utils/waypoint_renderer.hpp>

namespace coug_mapviz::utils {

namespace {

constexpr float kWaypointMarkerPx = 20.0F;
constexpr int kPathWidthPx = 2;
constexpr int kLabelWidthPx = 100;
constexpr int kLabelHeightPx = 20;
constexpr int kLabelOffsetXPx = 50;
constexpr int kDepthLabelOffsetYPx = 15;
constexpr int kSpeedLabelOffsetYPx = 33;
constexpr int kIndexLabelSizePx = 40;

const QColor kSlipCircleColor(30, 144, 255, 166);
const QColor kSlipCircleFillColor(30, 144, 255, 30);
const QColor kCaptureCircleColor(255, 140, 0, 191);
const QColor kCaptureCircleFillColor(255, 140, 0, 46);
const QColor kLightLabelColor(Qt::white);
const QColor kDarkLabelColor(Qt::black);
const QColor kActivePathColor(Qt::blue);
const QColor kSelectedWaypointColor(Qt::yellow);
const QColor kActiveWaypointColor(Qt::cyan);
const QColor kInactiveWaypointColor(Qt::gray);

}  // namespace

WaypointRenderer::WaypointRenderer(mapviz::MapCanvas* map_canvas) : map_canvas_(map_canvas) {}

QPointF WaypointRenderer::waypointToFixedPoint(
    const coug_interfaces::msg::WayPoint& waypoint,
    const swri_transform_util::Transform& fixed_T_wgs84) {
  const tf2::Vector3 point = wgs84ToFixed(waypoint, fixed_T_wgs84);
  return QPointF(point.x(), point.y());
}

QPointF WaypointRenderer::waypointToGl(const coug_interfaces::msg::WayPoint& waypoint,
                                       const swri_transform_util::Transform& fixed_T_wgs84) const {
  return map_canvas_->FixedFrameToMapGlCoord(waypointToFixedPoint(waypoint, fixed_T_wgs84));
}

void WaypointRenderer::paintCircles(QPainter* painter,
                                    const std::vector<coug_interfaces::msg::WayPoint>& waypoints,
                                    const swri_transform_util::Transform& fixed_T_wgs84) const {
  for (const auto& waypoint : waypoints) {
    const QPointF fixed_center = waypointToFixedPoint(waypoint, fixed_T_wgs84);
    const QPointF gl_center = map_canvas_->FixedFrameToMapGlCoord(fixed_center);
    const QPointF gl_slip_edge = map_canvas_->FixedFrameToMapGlCoord(
        QPointF(fixed_center.x() + waypoint.slip_radius, fixed_center.y()));
    const QPointF gl_capture_edge = map_canvas_->FixedFrameToMapGlCoord(
        QPointF(fixed_center.x() + waypoint.capture_radius, fixed_center.y()));

    const double slip_radius_px = QLineF(gl_center, gl_slip_edge).length();
    const double capture_radius_px = QLineF(gl_center, gl_capture_edge).length();

    painter->setPen(QPen(kSlipCircleColor, 1.5));
    painter->setBrush(QBrush(kSlipCircleFillColor));
    painter->drawEllipse(gl_center, slip_radius_px, slip_radius_px);

    painter->setPen(QPen(kCaptureCircleColor, 1.5));
    painter->setBrush(QBrush(kCaptureCircleFillColor));
    painter->drawEllipse(gl_center, capture_radius_px, capture_radius_px);
  }
}

void WaypointRenderer::paintLabels(QPainter* painter,
                                   const std::vector<coug_interfaces::msg::WayPoint>& waypoints,
                                   const swri_transform_util::Transform& fixed_T_wgs84,
                                   const QColor& color) const {
  for (size_t i = 0; i < waypoints.size(); ++i) {
    const QPointF gl_point = waypointToGl(waypoints[i], fixed_T_wgs84);
    painter->setPen(QPen(color));

    const QRectF depth_rect(
        QPointF(gl_point.x() - kLabelOffsetXPx, gl_point.y() + kDepthLabelOffsetYPx),
        QSizeF(kLabelWidthPx, kLabelHeightPx));
    const QString depth =
        waypoints[i].mode == coug_interfaces::msg::WayPoint::ALTITUDE
            ? "ALT " + QString::number(waypoints[i].position.altitude, 'f', 1) + "m"
            : QString::number(waypoints[i].position.altitude, 'f', 1) + "m";
    painter->drawText(depth_rect, Qt::AlignHCenter | Qt::AlignTop, depth);

    const QRectF speed_rect(
        QPointF(gl_point.x() - kLabelOffsetXPx, gl_point.y() + kSpeedLabelOffsetYPx),
        QSizeF(kLabelWidthPx, kLabelHeightPx));
    painter->drawText(speed_rect, Qt::AlignHCenter | Qt::AlignTop,
                      QString::number(waypoints[i].speed_rpm, 'f', 0) + "RPM");

    painter->setPen(QPen(color == kLightLabelColor ? kDarkLabelColor : color));
    const QRectF index_rect(
        QPointF(gl_point.x() - kIndexLabelSizePx / 2.0, gl_point.y() - kIndexLabelSizePx / 2.0),
        QSizeF(kIndexLabelSizePx, kIndexLabelSizePx));
    painter->drawText(index_rect, Qt::AlignHCenter | Qt::AlignVCenter, QString::number(i + 1));
  }
}

void WaypointRenderer::paintPath(QPainter* painter,
                                 const std::vector<coug_interfaces::msg::WayPoint>& waypoints,
                                 const QColor& color,
                                 const swri_transform_util::Transform& fixed_T_wgs84,
                                 int selected_idx) const {
  QVector<QPointF> points;
  for (const auto& waypoint : waypoints) points.push_back(waypointToGl(waypoint, fixed_T_wgs84));

  painter->setPen(QPen(color, kPathWidthPx));
  painter->drawPolyline(points);
  for (int i = 0; i < points.size(); ++i) {
    const QColor marker_color = i == selected_idx           ? kSelectedWaypointColor
                                : color == kActivePathColor ? kActiveWaypointColor
                                                            : kInactiveWaypointColor;
    painter->setPen(QPen(marker_color, kWaypointMarkerPx, Qt::SolidLine, Qt::RoundCap));
    painter->drawPoint(points[i]);
  }
}

}  // namespace coug_mapviz::utils
