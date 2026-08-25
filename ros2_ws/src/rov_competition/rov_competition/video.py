"""摄像头、录像文件和 RTP/H.264 视频源的统一封装。"""

from __future__ import annotations

from typing import Any


class VideoSourceError(RuntimeError):
    """视频源无法打开、配置错误或读取中断。"""


def _validate_udp_port(port: int) -> None:
    """确认 UDP 端口处于系统允许的范围。"""

    if not 1 <= port <= 65535:
        raise VideoSourceError("AI 视频端口必须在 1..65535")


def build_udp_mpegts_url(port: int) -> str:
    """创建旧 MPEG-TS 链路可用的 OpenCV/FFmpeg 地址。

    这个入口只用于录像和旧外部视频兼容。比赛默认视频使用
    :func:`build_udp_rtp_h264_gstreamer_pipeline`，不会再把 RTP
    重新封装成 MPEG-TS。
    """

    _validate_udp_port(port)
    return f"udp://127.0.0.1:{port}?fifo_size=1000000&overrun_nonfatal=1"


def build_udp_rtp_h264_gstreamer_pipeline(
    port: int,
    *,
    payload_type: int = 96,
) -> str:
    """创建比赛默认的 RTP/H.264 单槽位解码管线。

    50 ms 抖动缓冲负责少量网络乱序和水密缆上的短时突发。泄漏队列位于 ``h264parse``
    之后，此处一个缓冲区已经是一张完整 H.264 access unit；因此慢一帧时
    丢弃的是完整旧帧，不会从一张图像中间随意丢 RTP 包而主动制造花屏。
    ``appsink`` 也只保留一帧，YOLO 每次读取当前可用的新画面。
    """

    _validate_udp_port(port)
    if not 0 <= payload_type <= 127:
        raise VideoSourceError("RTP payload type 必须在 0..127")
    return " ".join(
        [
            f"udpsrc port={port} buffer-size=262144",
            "caps=application/x-rtp,media=video,encoding-name=H264,"
            f"payload={payload_type},clock-rate=90000",
            "! rtpjitterbuffer latency=50 drop-on-latency=true do-lost=true",
            "! rtph264depay",
            "! h264parse",
            "! video/x-h264,stream-format=byte-stream,alignment=au",
            "! queue leaky=downstream max-size-buffers=1",
            "max-size-bytes=0 max-size-time=0",
            "! avdec_h264 max-threads=2",
            "! videoconvert",
            "! video/x-raw,format=BGR",
            "! appsink name=appsink emit-signals=false",
            "drop=true max-buffers=1 sync=false",
        ]
    )


def should_retry_live_video_interruption(
    *, live_stream: bool, mission_active: bool
) -> bool:
    """判断视频中断后能否只重连解码器而不进入永久故障。

    RTP 直播短时断帧时允许重建本地解码管线。活动自主任务仍由控制循环
    按帧年龄立即回中、冻结计时并在硬阈值后中止，因此重连不等于继续
    使用旧框运动。录像文件结束则不可重连。
    """

    del mission_active  # 保留参数以兼容调用点，并明确安全判断位于上层。
    return live_stream


class OpenCvVideoSource:
    """读取摄像头、录像文件或旧 MPEG-TS 视频。

    在录像结束或摄像头断流时返回 ``False``，由上层停止运动，而不是
    继续使用最后一帧。比赛 RTP 直播请使用 :class:`GStreamerVideoSource`。
    """

    def __init__(self, source: str | int, *, gstreamer: bool = False) -> None:
        """保存视频源参数，但在 :meth:`open` 前不占用设备。"""

        self.source = source
        self.gstreamer = gstreamer
        self._capture: Any = None

    def open(self) -> None:
        """打开视频源并验证其可读性。"""

        try:
            import cv2
        except ImportError as exc:
            raise VideoSourceError("缺少 opencv-python，无法打开视频") from exc
        backend = cv2.CAP_GSTREAMER if self.gstreamer else cv2.CAP_ANY
        self._capture = cv2.VideoCapture(self.source, backend)
        if not self._capture.isOpened():
            self.release()
            raise VideoSourceError(f"无法打开视频源: {self.source}")

    def read(self) -> tuple[bool, Any]:
        """读取下一帧；视频结束或断流时返回 ``(False, None)``。"""

        if self._capture is None:
            raise VideoSourceError("视频源尚未打开")
        ok, frame = self._capture.read()
        if not ok or frame is None or getattr(frame, "size", 0) == 0:
            return False, None
        return True, frame

    def release(self) -> None:
        """释放底层摄像头或解码器资源，可重复调用。"""

        if self._capture is not None:
            self._capture.release()
            self._capture = None

    def __enter__(self) -> OpenCvVideoSource:  # noqa: PYI034
        """进入上下文时打开视频。"""

        self.open()
        return self

    def __exit__(self, *_: object) -> None:
        """退出上下文时释放视频设备。"""

        self.release()


class GStreamerVideoSource:
    """通过系统 GStreamer ``appsink`` 直接读取 BGR 帧。

    现场安装的 PyPI OpenCV 没有 GStreamer 后端，因此这里直接使用
    PyGObject，而不是依赖 ``cv2.CAP_GSTREAMER``。此类不建立后台取帧
    线程，也不维护性能统计；单槽位丢旧帧由上面的管线完成。
    """

    def __init__(self, pipeline: str, *, sample_timeout_s: float = 3.0) -> None:
        """保存必须包含命名 ``appsink`` 的管线。"""

        if not isinstance(pipeline, str) or not pipeline.strip():
            raise VideoSourceError("GStreamer 管线不能为空")
        if "appsink" not in pipeline:
            raise VideoSourceError("GStreamer 管线必须包含 appsink")
        if sample_timeout_s <= 0.0:
            raise ValueError("sample_timeout_s 必须大于 0")
        self.pipeline_text = pipeline
        self.sample_timeout_s = sample_timeout_s
        self._gst: Any = None
        self._pipeline: Any = None
        self._appsink: Any = None

    def open(self) -> None:
        """解析管线并进入 PLAYING 状态。"""

        try:
            import gi

            gi.require_version("Gst", "1.0")
            gi.require_version("GstApp", "1.0")
            from gi.repository import Gst, GstApp  # noqa: F401
        except (ImportError, ValueError) as exc:
            raise VideoSourceError(
                "缺少 PyGObject/GstApp；请安装 python3-gi 和 "
                "gir1.2-gst-plugins-base-1.0"
            ) from exc

        self._gst = Gst
        Gst.init(None)
        try:
            self._pipeline = Gst.parse_launch(self.pipeline_text)
            self._appsink = self._pipeline.get_by_name("appsink")
            if self._appsink is None:
                raise VideoSourceError("GStreamer 管线未提供 name=appsink 的输出")
            self._appsink.set_property("emit-signals", False)
            self._appsink.set_property("sync", False)
            self._appsink.set_property("max-buffers", 1)
            self._appsink.set_property("drop", True)
            change = self._pipeline.set_state(Gst.State.PLAYING)
            if change == Gst.StateChangeReturn.FAILURE:
                raise VideoSourceError("GStreamer 管线无法进入 PLAYING")
            settled, _state, _pending = self._pipeline.get_state(5 * Gst.SECOND)
            if settled == Gst.StateChangeReturn.FAILURE:
                raise VideoSourceError("GStreamer 管线启动失败")
        except VideoSourceError:
            self.release()
            raise
        except Exception as exc:
            self.release()
            raise VideoSourceError(f"GStreamer 管线解析或启动失败: {exc}") from exc

    def read(self) -> tuple[bool, Any]:
        """在限定时间内读取一帧，并复制为 NumPy BGR 数组。"""

        if self._pipeline is None or self._appsink is None or self._gst is None:
            raise VideoSourceError("GStreamer 视频源尚未打开")
        Gst = self._gst
        sample = self._appsink.emit(
            "try-pull-sample", int(self.sample_timeout_s * Gst.SECOND)
        )
        if sample is None:
            event = self._pipeline.get_bus().timed_pop_filtered(
                0, Gst.MessageType.ERROR | Gst.MessageType.EOS
            )
            if event is not None and event.type == Gst.MessageType.ERROR:
                error, detail = event.parse_error()
                raise VideoSourceError(
                    f"GStreamer 视频错误: {error}; {detail or '无调试信息'}"
                )
            return False, None

        caps = sample.get_caps()
        structure = caps.get_structure(0)
        width = int(structure.get_value("width"))
        height = int(structure.get_value("height"))
        pixel_format = str(structure.get_value("format"))
        if width <= 0 or height <= 0 or pixel_format != "BGR":
            raise VideoSourceError(
                f"appsink 必须输出正尺寸 BGR，实际={width}x{height} {pixel_format}"
            )

        buffer = sample.get_buffer()
        mapped, info = buffer.map(Gst.MapFlags.READ)
        if not mapped:
            raise VideoSourceError("GStreamer 帧内存映射失败")
        try:
            import numpy as np

            raw = np.frombuffer(info.data, dtype=np.uint8)
            if raw.size % height != 0:
                raise VideoSourceError("GStreamer BGR 行步长无法解析")
            row_bytes = raw.size // height
            required_row_bytes = width * 3
            if row_bytes < required_row_bytes:
                raise VideoSourceError("GStreamer BGR 帧长度小于画面尺寸")
            frame = (
                raw.reshape(height, row_bytes)[:, :required_row_bytes]
                .reshape(height, width, 3)
                .copy()
            )
        finally:
            buffer.unmap(info)
        return True, frame

    def release(self) -> None:
        """停止管线并释放资源，可重复调用。"""

        if self._pipeline is not None and self._gst is not None:
            self._pipeline.set_state(self._gst.State.NULL)
        self._appsink = None
        self._pipeline = None

    def __enter__(self) -> GStreamerVideoSource:  # noqa: PYI034
        """进入上下文时打开视频。"""

        self.open()
        return self

    def __exit__(self, *_: object) -> None:
        """退出上下文时释放视频管线。"""

        self.release()
