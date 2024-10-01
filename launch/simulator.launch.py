# Copyright 2024 INRAE, French National Research Institute for Agriculture, Food and Environment
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

from launch import LaunchDescription

from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
    GroupAction,
    SetEnvironmentVariable,
)

from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from tirrex_demo import (
    get_log_directory,
    get_debug_directory,
    get_demo_timestamp,
)


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)
    demo = "hackathon_bringup"
    demo_timestamp = get_demo_timestamp()
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context)
    demo_config_directory = LaunchConfiguration("demo_config_directory").perform(context)
    robot_config_directory = os.path.join(demo_config_directory, 'robot')

    debug_directory = get_debug_directory(demo, demo_timestamp, False)
    log_directory = get_log_directory(demo, demo_timestamp, False)

    print(" demo_config_directory: ", demo_config_directory)
    print(" debug_directory: ", debug_directory)
    print(" log_directory: ", log_directory)

    actions = []

    actions.append(SetEnvironmentVariable("ROS_LOG_DIR", log_directory))

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("tirrex_demo"), "launch/core.launch.py")
            ),
            launch_arguments={
                "mode": mode,
                "demo_config_directory": demo_config_directory,
                "robot_config_directory": robot_config_directory,
                "robot_namespace": robot_namespace,
            }.items(),
        )
    )

    return [GroupAction(actions)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("demo_config_directory"),
        DeclareLaunchArgument("robot_namespace"),
        DeclareLaunchArgument("mode"),
        OpaqueFunction(function=launch_setup),
    ])
