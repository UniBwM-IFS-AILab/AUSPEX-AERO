import numpy as np
import threading

import os
from picamera2 import Picamera2
import rclpy
from rclpy.executors import SingleThreadedExecutor

from .camera_stream.ipc_video_publisher import IPCVideoPublisher
from ..device import PeripheralDevice

#############################################################
# Image Grabber                                             #
#############################################################

class ImageGrabber:
    def __init__(self, width, height, fps):
        os.environ["LIBCAMERA_LOG_LEVELS"] = "*:2" # Prevents picam to spam console
        self._picam2 = Picamera2()

        config = self._picam2.create_video_configuration(
            main={"size": (width, height), "format": "RGB888"},
            controls={"FrameRate": fps}
        )
        self._picam2.configure(config)

        self._latest_frame = np.zeros((height, width, 3), dtype=np.uint8)

        self._callback = None
        self._lock = threading.Lock()
        self._running = False

    def start(self):
        if self._running:
            return
        self._picam2.post_callback = self._frame_callback
        self._picam2.start()
        self._running = True

    def stop(self):
        if not self._running:
            return

        self._picam2.stop()
        self._picam2.post_callback = None

        self._running = False
    
    def attach_callback(self, callback):
        """Callback signature: callback(frame: np.ndarray)"""
        self._callback = callback
    
    def get_latest_frame(self):
        with self._lock:
            return self._latest_frame

    def _frame_callback(self, request):
        frame = request.make_array("main")
        
        with self._lock:
            self._latest_frame = frame
        
        if self._callback is not None:
            self._callback(frame)


#############################################################
# ROS2 Node                                                 #
#############################################################

class Rpi5Camera(PeripheralDevice):
    def __init__(self):
        super().__init__()
        
        self.get_logger().info(f"Starting RPi5 Camera '{self.package_id}'...")
        self._get_config()
        self._setup_ipc_publisher()
        self._setup_image_grabber()
        self._start()
        self.get_logger().info(f"Camera '{self.package_id}' ready.")
    
    def _get_config(self):
        first_key = next(iter(self.config))
        self.width = self.config[first_key]["shared_memory_publishing_settings"]["width"]
        self.height = self.config[first_key]["shared_memory_publishing_settings"]["height"]
        self.fps = self.config[first_key]["shared_memory_publishing_settings"]["fps"]
    
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
            fps=self.fps
        )
    
    def _start(self):
        self._grabber.attach_callback(self._publisher.push)
        self._publisher.start()
        self._grabber.start()

    def shutdown(self):
        self.get_logger().info("Shutting down Rpi5Camera...")
        self._grabber.stop()
        self._publisher.stop()
        self.get_logger().info("Rpi5Camera shutdown complete.")

#############################################################
# Entrypoint - Start here                                   #
#############################################################

def main(args=None):
    rclpy.init(args=args)
    node = Rpi5Camera()
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