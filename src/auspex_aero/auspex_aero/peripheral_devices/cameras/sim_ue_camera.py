import numpy as np

import airsim
import rclpy
from rclpy.executors import SingleThreadedExecutor

from .camera_stream.ipc_video_publisher import IPCVideoPublisher
from ..device import PeripheralDevice

#############################################################
# Image Grabber                                             #
#############################################################

class ImageGrabber:
    def __init__(
        self,
        width,
        height,
        fps,
        host,
        port,
        vehicle_name,
        camera_name,
        timeout = 15
    ):
        self.width = width
        self.height = height
        self.fps = fps

        self.host = host
        self.port = port
        self.vehicle_name = vehicle_name
        self.camera_name = camera_name
        self.timeout = timeout
        
        self._latest_frame = np.zeros((height, width, 3), dtype=np.uint8)
    
    def connect(self):
        self.client = airsim.MultirotorClient(self.host, self.port, self.timeout)
        self.client.confirmConnection()
    
    def get_latest_frame(self) -> np.ndarray:
        reqs = [
            airsim.ImageRequest(
                self.camera_name,
                airsim.ImageType.Scene,
                pixels_as_float=False,
                compress=False
            )
        ]
        responses = self.client.simGetImages(reqs, self.vehicle_name)
        
        if not responses or len(responses) == 0:
            return self._latest_frame
        
        for response in responses:
            if response.image_data_uint8 is None or len(response.image_data_uint8) == 0:
                continue
            
            w = response.width
            h = response.height
            
            img = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
            img = img.reshape((h, w, -1))
            if img.shape[2] == 4:
                img = img[:, :, :3]
            self._latest_frame = img
        
        return self._latest_frame
        

#############################################################
# ROS2 Node                                                 #
#############################################################

class SimUeCamera(PeripheralDevice):
    def __init__(self):
        super().__init__()
        self.declare_parameter('platform_number', 0)
        
        self.get_logger().info(f"Starting UnrealEngine Camera '{self.package_id}'...")
        self._get_config()
        self._setup_ipc_publisher()
        self._setup_image_grabber()
        
        self._timer = self.create_timer(1.0 / self.fps, self._grab_frames)
        
        self._start()
        self.get_logger().info(f"Camera '{self.package_id}' ready.")
    
    def _get_config(self):
        self.port = 41450 + self.get_parameter('platform_number').get_parameter_value().integer_value
        self.vehicle_name = f"Drone{self.get_parameter('platform_number').get_parameter_value().integer_value + 1}"
        
        first_key = next(iter(self.config))
        self.width = self.config[first_key]["shared_memory_publishing_settings"]["width"]
        self.height = self.config[first_key]["shared_memory_publishing_settings"]["height"]
        self.fps = self.config[first_key]["shared_memory_publishing_settings"]["fps"]
        self.camera_name = self.config[first_key]["sim_ue_camera_name"]
    
    def _setup_ipc_publisher(self):
        self._publisher = IPCVideoPublisher(
            socket_path=f"/tmp/{self.platform_id}/{self.package_id}.sock",
            width=self.width,
            height=self.height,
            fps=self.fps,
            format="RGB"
        )

    def _setup_image_grabber(self):
        self._grabber = ImageGrabber(
            width=self.width,
            height=self.height,
            fps=self.fps,
            host="127.0.0.1",
            port=self.port,
            vehicle_name=self.vehicle_name,
            camera_name=self.camera_name,
        )
    
    def _start(self):
        self._grabber.connect()
        self._publisher.start()

    def shutdown(self):
        self.get_logger().info("Shutting down UnrealEngine Camera...")
        self._timer.cancel()
        self._publisher.stop()
        self.get_logger().info("UnrealEngine Camera shutdown complete.")
    
    def _grab_frames(self):
        frame = self._grabber.get_latest_frame()
        self._publisher.push(frame)

#############################################################
# Entrypoint - Start here                                   #
#############################################################

def main(args=None):
    rclpy.init(args=args)
    node = SimUeCamera()
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