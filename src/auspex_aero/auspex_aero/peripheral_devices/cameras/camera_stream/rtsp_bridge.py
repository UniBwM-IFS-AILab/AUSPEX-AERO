# This bridges the shared memory video to an RTSP server, allowing clients to connect and view the streams.
# Disclaimer: This is 100% vibe coded, i have no idea what is happening here, but it seems to work so who am i to judge :D

import yaml

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from .rtsp_server import ShmRtspBridge

#############################################################
# RtspBridge class                                          #
#############################################################

class RtspBridge(Node):
    def __init__(self) -> None:
        super().__init__('rtsp_bridge')
        # Declare parameters
        self.declare_parameter('platform_config_path', "")
        self.declare_parameter('platform_id', "")
        self.declare_parameter('port_offset', 0)
        
        self.get_logger().info(f"Starting RTSP Bridge...")
        
        self._start_rtsp_bridge()
        self._load_config()
        
        self.get_logger().info(f"RTSP Bridge ready.")
    
    def shutdown(self):
        self.server.stop()
    
    def _start_rtsp_bridge(self):
        self.port = 8554 + self.get_parameter('port_offset').get_parameter_value().integer_value
        self.server = ShmRtspBridge(rtsp_port=self.port)
        self.server.start()
    
    def _load_config(self):
        platform_config_path = self.get_parameter('platform_config_path').get_parameter_value().string_value
        if not platform_config_path:
            raise RuntimeError("platform_config_path parameter not set!")
        with open(platform_config_path, 'r') as file:
            config = yaml.safe_load(file)
        # Get ip address
        vpn_address = "127.0.0.1"
        for entry in config.get("network", []):
            if entry.get("id") == "vpn":
                vpn_address = entry.get("ip", "127.0.0.1")
                break

        platform_id = self.get_parameter('platform_id').get_parameter_value().string_value
        for device in config.get("peripheral_devices", []):
            if "rtsp_publishing_settings" in device:
                self.server.add_stream(
                    name=device["id"],
                    mount=f"/{platform_id}/{device['id']}",
                    socket_path=f"/tmp/{platform_id}/{device['id']}.sock",
                    in_width=device["shared_memory_publishing_settings"]["width"],
                    in_height=device["shared_memory_publishing_settings"]["height"],
                    in_fps=device["shared_memory_publishing_settings"]["fps"],
                    in_format="RGB",
                    out_width=device["rtsp_publishing_settings"]["width"],
                    out_height=device["rtsp_publishing_settings"]["height"],
                    out_fps=device["rtsp_publishing_settings"]["fps"],
                    bitrate_kbps=device["rtsp_publishing_settings"]["bitrate_kbps"],
                )
                self.get_logger().info(f"Added RTSP stream for device {device['id']} at rtsp://{vpn_address}:{self.port}/{platform_id}/{device['id']}")


#############################################################
# Entrypoint - Start here                                   #
#############################################################

def main(args=None):
    rclpy.init(args=args)
    node = RtspBridge()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()