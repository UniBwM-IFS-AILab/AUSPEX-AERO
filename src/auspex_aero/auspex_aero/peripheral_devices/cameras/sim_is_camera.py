import time
import numpy as np
import threading
import zmq
import json
import cv2

import rclpy
from rclpy.executors import SingleThreadedExecutor

from .camera_stream.ipc_video_publisher import IPCVideoPublisher
from ..device import PeripheralDevice

#############################################################
# Image Grabber                                             #
#############################################################

class ImageGrabber:
    def __init__(self, address, port, width, height):
        self._address = address
        self._port = port
        self._width = width
        self._height = height

        # ZMQ setup
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.SUB)
        self._socket.connect(f"tcp://{address}:{port}")
        self._socket.setsockopt_string(zmq.SUBSCRIBE, "")  # subscribe to all topics

        # Internal state
        self._latest_frame = None
        self._callback = None
        self._lock = threading.Lock()
        self._running = False
        self._thread = None

    def start(self):
        if self._running:
            return

        self._running = True
        self._thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._thread.start()

    def stop(self):
        if not self._running:
            return

        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)

        self._socket.close()
        self._context.term()

    def attach_callback(self, callback):
        """Callback signature: callback(frame: np.ndarray)"""
        self._callback = callback

    def get_latest_frame(self):
        with self._lock:
            return self._latest_frame

    def _receive_loop(self):
        while self._running:
            try:
                # Receive multipart message (header + image)
                header_json = self._socket.recv_string()
                img_bytes = self._socket.recv()

                header = json.loads(header_json)

                # Decode JPEG
                np_arr = np.frombuffer(img_bytes, dtype=np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                if frame is None:
                    continue

                # Convert BGR -> RGB to match publisher intent
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                
                # Resize if needed (should match publisher settings)
                frame_shape = frame.shape
                frame = cv2.resize(frame, (self._width, self._height))

                # Store latest frame
                with self._lock:
                    self._latest_frame = frame

                # Trigger callback
                if self._callback is not None:
                    self._callback(frame)

            except Exception as e:
                print(f"ZMQ Subscriber error: {e}")
                time.sleep(0.01)
    

#############################################################
# ROS2 Node                                                 #
#############################################################

class SimIsCamera(PeripheralDevice):
    def __init__(self):
        super().__init__()
        self.declare_parameter('platform_number', 0)
        
        self.get_logger().info(f"Starting IsaacSim Camera '{self.package_id}'...")
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
            address="localhost",
            port=5555 + self.get_parameter('platform_number').get_parameter_value().integer_value,
            width=self.width,
            height=self.height,
        )
    
    def _start(self):
        self._grabber.attach_callback(self._publisher.push)
        self._publisher.start()
        self._grabber.start()

    def shutdown(self):
        self.get_logger().info("Shutting down IsaacSim Camera...")
        self._grabber.stop()
        self._publisher.stop()
        self.get_logger().info("IsaacSim Camera shutdown complete.")

#############################################################
# Entrypoint - Start here                                   #
#############################################################

def main(args=None):
    rclpy.init(args=args)
    node = SimIsCamera()
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