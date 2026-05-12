# Disclaimer: This is 100% vibe coded, i have no idea what is happening here, but it seems to work so who am i to judge :D

from __future__ import annotations
import threading
from dataclasses import dataclass

import numpy as np
import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst  # noqa: E402


@dataclass(frozen=True)
class Config:
    socket_path: str = "/tmp/gst_ipc.sock"
    width: int = 640
    height: int = 480
    fps: int = 30
    format: str = "RGB"  # "RGB", "BGR", "GRAY8" etc (must match Gst video format names)


class IPCVideoPublisher:
    """
    Black-box publisher:
      pub = ShmPublisher(cfg)
      pub.start()
      pub.push(frame_np_uint8)
      pub.stop()
    """

    def __init__(self, socket_path: str, width: int, height: int, fps: int, format: str):
        self.cfg = Config(
            socket_path=socket_path,
            width=width,
            height=height,
            fps=fps,
            format=format
        )
        self._lock = threading.Lock()
        self._started = False

        Gst.init(None)

        self._pipeline = Gst.parse_launch(self._build_pipeline_str(self.cfg))
        self._appsrc = self._pipeline.get_by_name("src")
        if not self._appsrc:
            raise RuntimeError("Failed to get appsrc element 'src' from pipeline")

        self._configure_appsrc()

        # Precompute frame size for quick validation
        self._expected_bytes = self._bytes_per_frame()

        self._timestamp_ns = 0
        self._duration_ns = int(1e9 / max(1, self.cfg.fps))

    @staticmethod
    def _build_pipeline_str(cfg: Config) -> str:
        # do-timestamp=false because we set PTS ourselves
        # is-live=true + format=time for live pushing
        # shmsink: wait-for-connection=false so publisher can start before consumer
        return (
            "appsrc name=src is-live=true format=time do-timestamp=false "
            "! queue leaky=downstream max-size-buffers=1 max-size-time=0 max-size-bytes=0 "
            "! video/x-raw,format={fmt},width={w},height={h},framerate={fps}/1 "
            "! shmsink socket-path={sock} wait-for-connection=false sync=false shm-size=33554432"
        ).format(fmt=cfg.format, w=cfg.width, h=cfg.height, fps=cfg.fps, sock=cfg.socket_path)

    def _configure_appsrc(self) -> None:
        caps = Gst.Caps.from_string(
            f"video/x-raw,format={self.cfg.format},width={self.cfg.width},height={self.cfg.height},"
            f"framerate={self.cfg.fps}/1"
        )
        self._appsrc.set_property("caps", caps)
        self._appsrc.set_property("stream-type", 0)  # GST_APP_STREAM_TYPE_STREAM
        self._appsrc.set_property("format", Gst.Format.TIME)

    def _bytes_per_frame(self) -> int:
        fmt = self.cfg.format.upper()
        if fmt in ("RGB", "BGR"):
            channels = 3
        elif fmt in ("RGBA", "BGRA"):
            channels = 4
        elif fmt in ("GRAY8", "GRAY"):
            channels = 1
        else:
            raise ValueError(f"Unsupported format for size calc: {self.cfg.format}")
        return self.cfg.width * self.cfg.height * channels

    def start(self) -> None:
        with self._lock:
            if self._started:
                return
            ret = self._pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                raise RuntimeError("Failed to set publisher pipeline to PLAYING")
            self._started = True

    def push(self, frame: np.ndarray) -> None:
        """
        Push a single frame (uint8 numpy array).
        Expected shapes:
          RGB/BGR: (H, W, 3)
          RGBA/BGRA: (H, W, 4)
          GRAY8: (H, W) or (H, W, 1)
        """
        with self._lock:
            if not self._started:
                raise RuntimeError("Publisher not started. Call start() first.")

            if frame.dtype != np.uint8:
                raise ValueError("Frame must be np.uint8")

            # Ensure contiguous bytes
            frame_c = np.ascontiguousarray(frame)
            data = frame_c.tobytes()

            if len(data) != self._expected_bytes:
                raise ValueError(
                    f"Frame byte size mismatch: got {len(data)} expected {self._expected_bytes}. "
                    f"Check width/height/format."
                )

            buf = Gst.Buffer.new_allocate(None, len(data), None)
            buf.fill(0, data)

            # timestamps
            buf.pts = self._timestamp_ns
            buf.dts = self._timestamp_ns
            buf.duration = self._duration_ns
            self._timestamp_ns += self._duration_ns

            # push
            flow_ret = self._appsrc.emit("push-buffer", buf)
            if flow_ret != Gst.FlowReturn.OK:
                raise RuntimeError(f"push-buffer failed: {flow_ret}")

    def stop(self) -> None:
        with self._lock:
            if not self._started:
                return
            try:
                self._appsrc.emit("end-of-stream")
            except Exception:
                pass
            self._pipeline.set_state(Gst.State.NULL)
            self._started = False