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
#include <tf2/LinearMath/Vector3.h>

#include <coug_interfaces/msg/way_point.hpp>
#include <geographic_msgs/msg/geo_point.hpp>

namespace coug_mapviz::utils {

inline tf2::Vector3 wgs84ToFixed(const coug_interfaces::msg::WayPoint& waypoint,
                                 const swri_transform_util::Transform& fixed_T_wgs84) {
  const tf2::Vector3 wgs84_point(waypoint.position.longitude, waypoint.position.latitude, 0.0);
  return fixed_T_wgs84 * wgs84_point;
}

inline geographic_msgs::msg::GeoPoint fixedToWgs84(
    double x, double y, const swri_transform_util::Transform& wgs84_T_fixed) {
  const tf2::Vector3 wgs84_point = wgs84_T_fixed * tf2::Vector3(x, y, 0.0);

  geographic_msgs::msg::GeoPoint geo_point;
  geo_point.longitude = wgs84_point.x();
  geo_point.latitude = wgs84_point.y();
  return geo_point;
}

}  // namespace coug_mapviz::utils
