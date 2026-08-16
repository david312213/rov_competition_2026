"""摄像头、录像文件和 GStreamer 视频源的统一封装。"""

from __future__ import annotations

from typing import Any


class VideoSourceError(RuntimeError):
    """视频源无法打开或读取中断。"""


def build_udp_mpegts_url(port: int) -> str:
    """创建 OpenCV/FFmpeg 可读取的本机 MPEG-TS UDP 地址。"""

    if not 1 <= port <= 65535:
        raise VideoSourceError("AI 视频端口必须在 1..65535")
    return f"udp://127.0.0.1:{port}?fifo_size=1000000&overrun_nonfatal=1"


class OpenCvVideoSource:
    """OpenCV 视频源上下文管理器。

    ``source`` 可以是摄像头编号、录像文件路径，或显式提供的 GStreamer 管线。
    在录像结束或摄像头断流时返回 ``False``，由上层立即停止运动而不是继续使用
    最后一帧。
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
