# Copyright (c) 2022 OUXT Polaris
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

from ament_index_python.packages import get_package_share_path
import launch
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os
import yaml


def get_audio_capture_component(namespace: str, config_path: str):
    with open(config_path, "r") as f:
        params = yaml.safe_load(f)["audio_capture_node"]["ros__parameters"]
    component = ComposableNode(
        package="audio_capture",
        plugin="audio_capture::AudioCaptureNode",
        namespace=namespace,
        name="audio_capture_node",
        remappings=[],
        parameters=[params],
    )
    return component


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    audio_capture_config = LaunchConfiguration("audio_capture_config")
    return launch.LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="audio"),
            DeclareLaunchArgument(
                "audio_capture_config",
                default_value=os.path.join(
                    get_package_share_path("whisper_ros"),
                    "config",
                ),
            ),
            ComposableNodeContainer(
                name="whisper_container",
                namespace=namespace,
                package="rclcpp_components",
                executable="component_container_mt",
                composable_node_descriptions=[
                    get_audio_capture_component(
                        namespace=namespace, config_path=audio_capture_config
                    )
                ],
                output="screen",
            ),
        ]
    )
