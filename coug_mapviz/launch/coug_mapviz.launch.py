# Copyright (c) 2026 BYU FROST Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import tempfile
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node


def launch_setup(context: LaunchContext, *args: Any, **kwargs: Any) -> list[Node]:
    use_sim_time = LaunchConfiguration("use_sim_time")
    agent_list_str = LaunchConfiguration("agent_list").perform(context)

    agent_namespaces = yaml.safe_load(agent_list_str)
    is_multiagent = len(agent_namespaces) > 1

    pkg_share = get_package_share_directory("coug_mapviz")
    fleet_params = PathJoinSubstitution(
        [
            EnvironmentVariable("CONFIG_DIR"),
            "fleet",
            "coug_mapviz_params.yaml",
        ]
    )
    full_template_path = os.path.join(pkg_share, "config", "mapviz_config.mvc.template")

    if is_multiagent:
        with open(full_template_path, "r") as f:
            base = yaml.safe_load(f.read())

        global_display_types = {
            "mapviz_plugins/tile_map",
            "coug_mapviz/coug_waypoints",
        }
        base["displays"] = [
            d for d in base["displays"] if d["type"] in global_display_types
        ]

        per_agent_template_path = os.path.join(
            pkg_share, "config", "multi_mapviz_config.mvc.template"
        )
        with open(per_agent_template_path, "r") as f:
            per_agent_template = f.read()

        for ns in agent_namespaces:
            per_agent = yaml.safe_load(per_agent_template.replace("AUV_NS", ns))
            base["displays"].extend(per_agent["displays"])

        config_content = yaml.safe_dump(base)
    else:
        with open(full_template_path, "r") as f:
            config_content = f.read().replace("AUV_NS", agent_namespaces[0])

    with tempfile.NamedTemporaryFile(
        mode="w", delete=False, suffix=".mvc"
    ) as temp_config:
        temp_config.write(config_content)
        mapviz_config_file = temp_config.name

    return [
        Node(
            package="mapviz",
            executable="mapviz",
            name="mapviz",
            parameters=[
                fleet_params,
                {
                    "config": mapviz_config_file,
                    "use_sim_time": use_sim_time,
                    "agent_namespaces": agent_namespaces,
                },
            ],
        ),
        Node(
            package="swri_transform_util",
            executable="initialize_origin.py",
            name="initialize_origin",
            remappings=[
                ("fix", "/origin"),
            ],
            parameters=[
                fleet_params,
                {"use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_origin_transform",
            arguments=[
                "--x",
                "0",
                "--y",
                "0",
                "--z",
                "0",
                "--yaw",
                "0",
                "--pitch",
                "0",
                "--roll",
                "0",
                "--frame-id",
                "map",
                "--child-frame-id",
                "origin",
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation/rosbag clock if true",
            ),
            DeclareLaunchArgument(
                "agent_list",
                default_value="[auv0]",
                description=(
                    "YAML list of agent namespaces "
                    "(e.g. '[coug1sim]' or '[coug1sim, coug2sim]')"
                ),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
