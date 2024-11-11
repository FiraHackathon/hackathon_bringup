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
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def get_simulation_world(config_directory):
    simulation_file = os.path.join(config_directory, "simulation.yaml")
    with open(simulation_file) as f:
        data = yaml.safe_load(f)

    return os.path.join(
        get_package_share_directory(data["world_package"]), "worlds", data["world_name"]
    )


def launch_setup(context, *args, **kwargs):
    config_directory = LaunchConfiguration("demo_config_directory").perform(context)
    field_coverage_config = os.path.join(
        get_package_share_directory("hackathon_evaluation"), "config", "demo_hackathon.yaml"
    )
    world_path = get_simulation_world(config_directory)

    evaluation_data_path = os.path.join(
        get_package_share_directory("hackathon_evaluation"), "data"
    )

    parameters = {'world_file': world_path}
    for field in ["mixed_field", "sloping_field"]:
        csv_name = f"{field}_stem_positions.csv"
        parameters[f"{field}_data_file"] = os.path.join(evaluation_data_path, csv_name)

    actions = [
        Node(
            package="hackathon_evaluation",
            executable="evaluation_node",
            name="evaluation",
            exec_name="evaluation",
            parameters=[parameters],
            namespace="evaluation",
        ),

        Node(
            package="hackathon_evaluation",
            executable="field_coverage_node",
            name="field_coverage",
            exec_name="field_coverage",
            namespace="evaluation",
            parameters=[field_coverage_config]
        ),
    ]

    return [GroupAction(actions)]


def generate_launch_description():
    entities = [
        DeclareLaunchArgument(
            "robot_namespace",
            description="ROS namespace used for the robot",
        ),
        DeclareLaunchArgument(
            "demo_config_directory",
            description="directory containing main YAML files",
        ),
        OpaqueFunction(function=launch_setup),
    ]
    return LaunchDescription(entities)
