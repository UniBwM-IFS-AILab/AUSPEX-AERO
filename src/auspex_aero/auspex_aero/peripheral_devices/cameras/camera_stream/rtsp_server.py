from __future__ import annotations

import os
import threading
import time
from dataclasses import dataclass

import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstRtspServer", "1.0")

from gi.repository import GLib, Gst, GstRtspServer


@dataclass(frozen=True)
class StreamSpec:
    name: str
    mount: str
    socket_path: str

    in_width: int
    in_height: int
    in_fps: int
    in_format: str = "RGB"

    out_width: int = 320
    out_height: int = 240
    out_fps: int = 15
    bitrate_kbps: int = 800

    # internal "virtual channel" used between ingest and RTSP pipelines
    channel: str | None = None

    def resolved_channel(self) -> str:
        return self.channel or f"bridge_{self.name}"


class _IngestWorker:
    """
    Owns one shm->intervideosink pipeline and reconnect logic.
    Safe against the shm socket not existing yet.
    """

    def __init__(self, spec: StreamSpec, retry_interval_s: float = 1.0) -> None:
        self.spec = spec
        self.retry_interval_s = retry_interval_s

        self._pipeline: Gst.Pipeline | None = None
        self._running = False
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()

    def start(self) -> None:
        with self._lock:
            if self._running:
                return
            self._running = True
            self._thread = threading.Thread(target=self._run, name=f"ingest-{self.spec.name}", daemon=True)
            self._thread.start()

    def stop(self) -> None:
        with self._lock:
            self._running = False

        if self._thread is not None:
            self._thread.join(timeout=5.0)

        self._teardown_pipeline()

    def _run(self) -> None:
        while self._is_running():
            if not os.path.exists(self.spec.socket_path):
                self._teardown_pipeline()
                time.sleep(self.retry_interval_s)
                continue

            if self._pipeline is None:
                try:
                    self._pipeline = self._build_pipeline()
                    ret = self._pipeline.set_state(Gst.State.PLAYING)
                    if ret == Gst.StateChangeReturn.FAILURE:
                        self._teardown_pipeline()
                        time.sleep(self.retry_interval_s)
                        continue
                except Exception:
                    self._teardown_pipeline()
                    time.sleep(self.retry_interval_s)
                    continue

            # Watch the bus briefly. If the source disappears or errors, restart later.
            bus = self._pipeline.get_bus()
            msg = bus.timed_pop_filtered(
                500 * Gst.MSECOND,
                Gst.MessageType.ERROR | Gst.MessageType.EOS | Gst.MessageType.STATE_CHANGED,
            )

            if msg is None:
                # still healthy enough; keep looping
                continue

            if msg.type == Gst.MessageType.ERROR:
                self._teardown_pipeline()
                time.sleep(self.retry_interval_s)
                continue

            if msg.type == Gst.MessageType.EOS:
                self._teardown_pipeline()
                time.sleep(self.retry_interval_s)
                continue

            # If socket vanished underneath us, force reconnect logic.
            if not os.path.exists(self.spec.socket_path):
                self._teardown_pipeline()
                time.sleep(self.retry_interval_s)

        self._teardown_pipeline()

    def _is_running(self) -> bool:
        with self._lock:
            return self._running

    def _teardown_pipeline(self) -> None:
        if self._pipeline is not None:
            try:
                self._pipeline.set_state(Gst.State.NULL)
            except Exception:
                pass
            self._pipeline = None

    def _build_pipeline(self) -> Gst.Pipeline:
        channel = self.spec.resolved_channel()

        launch = f"""
            shmsrc socket-path={self.spec.socket_path} is-live=true do-timestamp=true !
            video/x-raw,format={self.spec.in_format},width={self.spec.in_width},height={self.spec.in_height},framerate={self.spec.in_fps}/1 !
            queue leaky=downstream max-size-buffers=2 max-size-bytes=0 max-size-time=0 !
            intervideosink channel={channel} sync=false async=false
        """

        element = Gst.parse_launch(" ".join(launch.split()))
        if not isinstance(element, Gst.Pipeline):
            # parse_launch can return a bin; wrap if needed
            pipeline = Gst.Pipeline.new(f"ingest_{self.spec.name}")
            pipeline.add(element)
            return pipeline
        return element


class _RtspFactory(GstRtspServer.RTSPMediaFactory):
    """
    RTSP side never touches shm directly.
    It always reads from intervideosrc.
    """

    def __init__(self, spec: StreamSpec) -> None:
        super().__init__()
        self.spec = spec
        self.set_shared(True)

    def do_create_element(self, _url: object) -> Gst.Element:
        channel = self.spec.resolved_channel()

        launch = f"""
            intervideosrc channel={channel} !
            queue leaky=downstream max-size-buffers=2 max-size-bytes=0 max-size-time=0 !
            videoconvert !
            videoscale !
            videorate !
            video/x-raw,format=I420,width={self.spec.out_width},height={self.spec.out_height},framerate={self.spec.out_fps}/1 !
            x264enc tune=zerolatency speed-preset=veryfast bitrate={self.spec.bitrate_kbps} key-int-max={max(1, self.spec.out_fps * 2)} !
            h264parse !
            rtph264pay name=pay0 pt=96 config-interval=1
        """
        return Gst.parse_launch(" ".join(launch.split()))


class ShmRtspBridge:
    """
    Public module-friendly API.
    Create the bridge, start it, then add streams whenever needed.
    """

    def __init__(
        self,
        streams: list[StreamSpec] | None = None,
        rtsp_port: int = 8554,
        retry_interval_s: float = 1.0,
    ) -> None:
        self.streams = list(streams or [])
        self.rtsp_port = rtsp_port
        self.retry_interval_s = retry_interval_s

        self._started = False
        self._main_loop: GLib.MainLoop | None = None
        self._main_loop_thread: threading.Thread | None = None
        self._server: GstRtspServer.RTSPServer | None = None
        self._workers: dict[str, _IngestWorker] = {}

        Gst.init(None)

    def add_stream(
        self,
        name: str,
        mount: str,
        socket_path: str,
        in_width: int,
        in_height: int,
        in_fps: int,
        in_format: str = "RGB",
        out_width: int = 320,
        out_height: int = 240,
        out_fps: int = 15,
        bitrate_kbps: int = 800,
        channel: str | None = None,
    ) -> None:
        spec = StreamSpec(
            name=name,
            mount=mount,
            socket_path=socket_path,
            in_width=in_width,
            in_height=in_height,
            in_fps=in_fps,
            in_format=in_format,
            out_width=out_width,
            out_height=out_height,
            out_fps=out_fps,
            bitrate_kbps=bitrate_kbps,
            channel=channel,
        )
        self.add_stream_spec(spec)

    def add_stream_spec(self, spec: StreamSpec) -> None:
        if any(existing.name == spec.name for existing in self.streams):
            raise ValueError(f"Stream name already exists: {spec.name}")

        if any(existing.mount == spec.mount for existing in self.streams):
            raise ValueError(f"RTSP mount already exists: {spec.mount}")

        self.streams.append(spec)

        if self._started:
            self._register_stream(spec)

    def _register_stream(self, spec: StreamSpec) -> None:
        if self._server is None:
            raise RuntimeError("RTSP server is not initialized")

        if spec.name in self._workers:
            return

        mounts = self._server.get_mount_points()
        factory = _RtspFactory(spec)
        mounts.add_factory(spec.mount, factory)

        worker = _IngestWorker(spec=spec, retry_interval_s=self.retry_interval_s)
        self._workers[spec.name] = worker
        worker.start()

    def start(self) -> None:
        if self._started:
            return

        self._server = GstRtspServer.RTSPServer()
        self._server.set_service(str(self.rtsp_port))

        self._main_loop = GLib.MainLoop()
        self._server.attach(None)

        self._main_loop_thread = threading.Thread(
            target=self._main_loop.run,
            name="rtsp-mainloop",
            daemon=True,
        )
        self._main_loop_thread.start()

        for spec in self.streams:
            self._register_stream(spec)

        self._started = True

    def stop(self) -> None:
        if not self._started:
            return

        for worker in self._workers.values():
            worker.stop()
        self._workers.clear()

        if self._main_loop is not None:
            self._main_loop.quit()

        if self._main_loop_thread is not None:
            self._main_loop_thread.join(timeout=5.0)

        self._main_loop = None
        self._main_loop_thread = None
        self._server = None
        self._started = False

    def rtsp_urls(self, host: str = "127.0.0.1") -> list[str]:
        return [f"rtsp://{host}:{self.rtsp_port}{spec.mount}" for spec in self.streams]