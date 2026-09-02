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
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node


def create_mapviz_config(agent_list: list[str], gui_dir: str) -> str:
    with open(os.path.join(gui_dir, "mapviz.mvc.template")) as template:
        content = template.read()

    if len(agent_list) == 1:
        config_content = content.replace("AGENT_NS", agent_list[0])
    else:
        config = yaml.safe_load(content)
        displays = config["displays"]
        displays[:] = [
            display
            for display in displays
            if display["type"]
            in {"mapviz_plugins/tile_map", "coug_mapviz/coug_waypoints"}
        ]
        with open(os.path.join(gui_dir, "multi_mapviz.mvc.template")) as template:
            agent_template = template.read()
        displays.extend(
            display
            for agent_ns in agent_list
            for display in yaml.safe_load(agent_template.replace("AGENT_NS", agent_ns))[
                "displays"
            ]
        )
        config_content = yaml.safe_dump(config)

    with tempfile.NamedTemporaryFile(
        mode="w", delete=False, suffix=".mvc"
    ) as rendered_config:
        rendered_config.write(config_content)
        return rendered_config.name


def launch_setup(context: LaunchContext, *args: Any, **kwargs: Any) -> list[Node]:
    use_sim_time = LaunchConfiguration("use_sim_time")
    agent_list_str = LaunchConfiguration("agent_list").perform(context)

    agent_list = yaml.safe_load(agent_list_str)
    config_dir = os.environ["CONFIG_DIR"]

    fleet_param_file = PathJoinSubstitution(
        [
            EnvironmentVariable("CONFIG_DIR"),
            "fleet",
            "coug_mapviz_params.yaml",
        ]
    )
    mapviz_config_file = create_mapviz_config(
        agent_list, os.path.join(config_dir, "gui")
    )

    return [
        Node(
            package="mapviz",
            executable="mapviz",
            name="mapviz",
            parameters=[
                fleet_param_file,
                {
                    "config": mapviz_config_file,
                    "use_sim_time": use_sim_time,
                    "map_frame": "map",
                    "agent_list": agent_list,
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
                fleet_param_file,
                {"use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_origin_transform",
            arguments=[
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
