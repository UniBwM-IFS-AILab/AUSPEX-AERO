# This module defines an base class for all peripheral devices.

import yaml

from rclpy.node import Node

#############################################################
# PeripheralDevice Base Class                               #
#############################################################

class PeripheralDevice(Node):
    def __init__(self) -> None:
        super().__init__('peripheral_device')
        # Declare parameters
        self.declare_parameter('platform_config_path', "")
        self.declare_parameter('platform_id', "")
        self.declare_parameter('package_id', "")
        
        self.platform_id = self.get_parameter('platform_id').get_parameter_value().string_value
        self.package_id = self.get_parameter('package_id').get_parameter_value().string_value
        
        # Load configuration
        platform_config_path = self.get_parameter('platform_config_path').get_parameter_value().string_value
        if not platform_config_path:
            raise RuntimeError("platform_config_path parameter not set!")
        with open(platform_config_path, 'r') as file:
            super_config = yaml.safe_load(file)
        # Trim configuration to only the relevant package/device
        self.config = {}
        for entry in super_config.get("peripheral_devices", []):
            package_id = entry.get("package_id")
            if not package_id:
                package_id = entry["id"]
            if package_id == self.package_id:
                self.config[entry["id"]] = entry
                break