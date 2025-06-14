#!/usr/bin/env python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
import json
import os

def launch_setup(context, *args, **kwargs):
    count = int(context.perform_substitution(LaunchConfiguration('count')))

    json_file = os.path.expanduser("~/auspex_params/platform_properties/platform_properties.json")

    # Check if file exists
    if not os.path.exists(json_file):
        print(f"Error: JSON file not found at {json_file}")
        exit(1)
    # Read and parse JSON
    with open(json_file, "r") as file:
        data = json.load(file)

    # Extract platform_id
    platform_id = data.get("platform_id", None)
    # Validate platform_id
    if not platform_id:
        print("Error: platform_id not found in JSON!")
        exit(1)

    launch_array =  []

    for x in range(0, count):
        name_prefix = platform_id + '_' + str(x)
        launch_array.append(
            Node(
                package='auspex_obc',
                executable='auspex_obc',
                output='screen',
                shell=True,
                arguments=[name_prefix]
            )
        )
    return launch_array

# declare command line argument via appending count:=NUMBER when starting the launch file from terminal
def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument('count', default_value='1')] + [OpaqueFunction(function=launch_setup)]
    )
