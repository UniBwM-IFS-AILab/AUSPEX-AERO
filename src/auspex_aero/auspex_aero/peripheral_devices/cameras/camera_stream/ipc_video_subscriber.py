# Disclaimer: This is 100% vibe coded, i have no idea what is happening here, but it seems to work so who am i to judge :D

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Callable, Optional, Tuple

import numpy as np

import gi
gi.require_version("Gst", "1.0")
gi.require_version("GstApp", "1.0")
from gi.repository import Gst, GstApp, GLib  # noqa: E402


@dataclass(frozen=True)
class Config:
    socket_path: str = "/tmp/gst_ipc.sock"
    width: int = 640
    height: int = 480
    format: str = "RGB"  # "RGB", "BGR", "GRAY8" etc (must match Gst video format names)
    enforce_caps: bool = True


class IPCVideoSubscriber:
    """
    Black-box consumer:
      con = IPCVideoSubscriber(cfg)
      con.start()

      frame = con.get_latest()  # returns a COPY of latest numpy frame or None
      con.set_callback(cb)      # cb(frame_np) called from internal GLib thread

      con.stop()
    """

    def __init__(self, socket_path: str, width: int, height: int, format: str, enforce_caps: bool = True):
        self.cfg = Config(
            socket_path=socket_path,
            width=width,
            height=height,
            format=format,
            enforce_caps=enforce_caps
        )
        self._lock = threading.Lock()
        self._started = False

        self._latest: Optional[np.ndarray] = None
        self._callback: Optional[Callable[[np.ndarray], None]] = None

        self._loop: Optional[GLib.MainLoop] = None
        self._thread: Optional[threading.Thread] = None

        Gst.init(None)

        self._pipeline = Gst.parse_launch(self._build_pipeline_str(self.cfg))
        self._appsink = self._pipeline.get_by_name("sink")
        if not self._appsink:
            raise RuntimeError("Failed to get appsink element 'sink' from pipeline")

        self._appsink.connect("new-sample", self._on_new_sample)

        self._expected_shape = self._frame_shape()

    @staticmethod
    def _build_pipeline_str(cfg: Config) -> str:
        # appsink drop=true max-buffers=1 => always latest only
        # sync=false => no clock sync delays
        # shmsrc do-timestamp not needed; we just read samples
        base = f"shmsrc socket-path={cfg.socket_path} is-live=true do-timestamp=true"
        if cfg.enforce_caps:
            caps = f"video/x-raw,format={cfg.format},width={cfg.width},height={cfg.height}"
            return (
                f"{base} ! queue leaky=downstream max-size-buffers=1 max-size-time=0 max-size-bytes=0 "
                f"! {caps} "
                f"! appsink name=sink emit-signals=true sync=false drop=true max-buffers=1"
            )
        return (
            f"{base} ! queue leaky=downstream max-size-buffers=1 max-size-time=0 max-size-bytes=0 "
            f"! appsink name=sink emit-signals=true sync=false drop=true max-buffers=1"
        )

    def _frame_shape(self) -> Tuple[int, ...]:
        fmt = self.cfg.format.upper()
        if fmt in ("RGB", "BGR"):
            return (self.cfg.height, self.cfg.width, 3)
        if fmt in ("RGBA", "BGRA"):
            return (self.cfg.height, self.cfg.width, 4)
        if fmt in ("GRAY8", "GRAY"):
            return (self.cfg.height, self.cfg.width)
        raise ValueError(f"Unsupported format: {self.cfg.format}")

    def start(self) -> None:
        with self._lock:
            if self._started:
                return

            self._loop = GLib.MainLoop()
            self._thread = threading.Thread(target=self._run_loop, daemon=True)
            self._thread.start()

            ret = self._pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                raise RuntimeError("Failed to set consumer pipeline to PLAYING")

            self._started = True

    def _run_loop(self) -> None:
        assert self._loop is not None
        self._loop.run()

    def set_callback(self, cb: Optional[Callable[[np.ndarray], None]]) -> None:
        with self._lock:
            self._callback = cb

    def get_latest(self) -> Optional[np.ndarray]:
        """
        Returns a COPY of the latest frame (so callers can safely mutate),
        or None if nothing received yet.
        """
        with self._lock:
            if self._latest is None:
                return None
            return self._latest.copy()

    def _on_new_sample(self, sink: GstApp.AppSink) -> Gst.FlowReturn:
        sample = sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.OK

        buf = sample.get_buffer()
        ok, mapinfo = buf.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.OK

        try:
            data = mapinfo.data  # bytes-like
            frame = np.frombuffer(data, dtype=np.uint8)

            # reshape
            frame = frame.reshape(self._expected_shape)

            # copy out of Gst buffer memory
            frame_copy = frame.copy()

            cb = None
            with self._lock:
                self._latest = frame_copy
                cb = self._callback

            if cb is not None:
                # callback runs in GLib thread; keep it fast!
                cb(frame_copy)

        finally:
            buf.unmap(mapinfo)

        return Gst.FlowReturn.OK

    def stop(self) -> None:
        with self._lock:
            if not self._started:
                return
            self._pipeline.set_state(Gst.State.NULL)
            if self._loop is not None:
                self._loop.quit()
            self._started = False

        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        self._loop = None