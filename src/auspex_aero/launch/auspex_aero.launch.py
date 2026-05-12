#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
import yaml
import os

def launch_setup(context, *args, **kwargs):
    count = int(context.perform_substitution(LaunchConfiguration('count')))
    
    # Load YAML configuration file
    config_file_path = os.path.expanduser(
        "~/AUSPEX/params/platform_properties/platform_properties.yaml"
    )
    if not os.path.exists(config_file_path):
        print(f"Error: YAML file not found at {config_file_path}")
        raise FileNotFoundError(config_file_path)
    with open(config_file_path, "r") as file:
        config = yaml.safe_load(file)
    platform_prefix = config["general"]["platform_id"]
    if not platform_prefix:
        print("Error: platform_id not found in YAML!")
        raise RuntimeError("Error: platform_id not found in YAML!")
    
    launch_array = []
    delay_seconds = 0.5
    
    for i in range(count):
        if count > 1 or platform_prefix.startswith("sim"):
            platform_id = f"{platform_prefix}_{i}"
        else:
            platform_id = platform_prefix
        
        # Core Nodes
        nodes = [
            Node(
                package='auspex_aero',
                executable='offboard_controller',
                name=f'{platform_id}_offboard_controller',
                exec_name=f'{platform_id}_offboard_controller',
                output='screen',
                emulate_tty=True,
                parameters=[{
                    'platform_config_path': config_file_path,
                    'platform_id': platform_id
                }],
                ros_arguments=['--log-level', LaunchConfiguration('log_level')]
            ),
            Node(
                package='auspex_aero',
                executable='flight_manager',
                name=f'{platform_id}_flight_manager',
                exec_name=f'{platform_id}_flight_manager',
                output='screen',
                emulate_tty=True,
                parameters=[{
                    'platform_config_path': config_file_path,
                    'platform_id': platform_id,
                    'platform_number': i
                }],
                ros_arguments=['--log-level', LaunchConfiguration('log_level')]
            )
        ]
        
        # Sensors and Actuators
        # Parse into packages (Important for f.e. ZT30), standalone sensors/actuators will be a package of one.
        packages = {}
        need_rtsp_bridge = False
        for device in config.get("peripheral_devices", []):
            package_id = device.get("package_id")
            if not package_id:
                package_id = device["id"]
            if package_id not in packages:
                packages[package_id] = []
            packages[package_id].append(device)
            if "rtsp_publishing_settings" in device:
                need_rtsp_bridge = True
        # Add nodes for packages
        for package_id, components in packages.items():
            model = components[0].get("model")
            match model:
                case "RPI5":
                    nodes.append(Node(
                        package='auspex_aero',
                        executable='rpi5_camera',
                        name=f'{platform_id}_rpi5_camera',
                        exec_name=f'{platform_id}_rpi5_camera',
                        output='screen',
                        emulate_tty=True,
                        parameters=[{
                            'platform_config_path': config_file_path,
                            'platform_id': platform_id,
                            'package_id': package_id
                        }],
                        ros_arguments=['--log-level', LaunchConfiguration('log_level')]
                    ))
                case "ZT30":
                    print("[WARNING] ZT30 support not implemented yet")
                case "SIM_UE":
                    nodes.append(Node(
                        package='auspex_aero',
                        executable='sim_ue_camera',
                        name=f'{platform_id}_sim_ue_camera',
                        exec_name=f'{platform_id}_sim_ue_camera',
                        output='screen',
                        emulate_tty=True,
                        parameters=[{
                            'platform_config_path': config_file_path,
                            'platform_id': platform_id,
                            'package_id': package_id,
                            'platform_number': i
                        }],
                        ros_arguments=['--log-level', LaunchConfiguration('log_level')]
                    ))
                case "SIM_IS":
                    nodes.append(Node(
                        package='auspex_aero',
                        executable='sim_is_camera',
                        name=f'{platform_id}_sim_is_camera',
                        exec_name=f'{platform_id}_sim_is_camera',
                        output='screen',
                        emulate_tty=True,
                        parameters=[{
                            'platform_config_path': config_file_path,
                            'platform_id': platform_id,
                            'package_id': package_id,
                            'platform_number': i
                        }],
                        ros_arguments=['--log-level', LaunchConfiguration('log_level')]
                    ))
                case _:
                    print(f"[WARNING] Unknown model {model}")
        if need_rtsp_bridge:
            nodes.append(Node(
                package='auspex_aero',
                executable='rtsp_bridge',
                name=f'{platform_id}_rtsp_bridge',
                exec_name=f'{platform_id}_rtsp_bridge',
                output='screen',
                emulate_tty=True,
                parameters=[{
                    'platform_config_path': config_file_path,
                    'platform_id': platform_id,
                    'port_offset': i
                }],
                ros_arguments=['--log-level', LaunchConfiguration('log_level')]
            ))
        
        # Add nodes to launch array with delay
        for node in nodes:
            if i == 0:
                launch_array.append(node)
            else:
                launch_array.append(
                    TimerAction(
                        period=delay_seconds * i,
                        actions=[node]
                    )
                )
        
    return launch_array


def generate_launch_description():
    """Launch AUSPEX AERO."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'count',
            default_value='1',
            description='Number of AERO instances to launch'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level'
        ),
        OpaqueFunction(function=launch_setup)
    ])