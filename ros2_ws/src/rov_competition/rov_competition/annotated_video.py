"""将带识别框的 OpenCV 画面编码为 H.264/RTP。"""

from __future__ import annotations

import shutil
import subprocess
from typing import Any


class AnnotatedVideoError(RuntimeError):
    """FFmpeg 不可用、编码器启动失败或视频管道断开。"""


def build_ffmpeg_rtp_command(
    host: str,
    port: int,
    *,
    width: int = 640,
    height: int = 480,
    fps: int = 20,
    bitrate_kbps: int = 2048,
) -> list[str]:
    """创建不经过 shell 的 FFmpeg 原始 BGR 到 H.264/RTP 参数。"""

    if not host or any(character in host for character in "/?&#"):
        raise AnnotatedVideoError("RTP 主机名为空或包含非法字符")
    if not 1 <= port <= 65535:
        raise AnnotatedVideoError("RTP 端口必须在 1..65535")
    if min(width, height, fps, bitrate_kbps) <= 0:
        raise AnnotatedVideoError("画面尺寸、帧率和码率必须大于 0")
    return [
        "ffmpeg",
        "-hide_banner",
        "-loglevel",
        "warning",
        "-f",
        "rawvideo",
        "-pixel_format",
        "bgr24",
        "-video_size",
        f"{width}x{height}",
        "-framerate",
        str(fps),
        "-i",
        "pipe:0",
        "-an",
        "-c:v",
        "libx264",
        "-preset",
        "ultrafast",
        "-tune",
        "zerolatency",
        "-b:v",
        f"{bitrate_kbps}k",
        "-g",
        str(max(1, fps)),
        "-f",
        "rtp",
        "-payload_type",
        "96",
        f"rtp://{host}:{port}?pkt_size=1200",
    ]


class AnnotatedRtpPublisher:
    """通过 FFmpeg 子进程发布固定尺寸的带框 RTP 画面。"""

    def __init__(
        self,
        host: str,
        port: int,
        *,
        width: int = 640,
        height: int = 480,
        fps: int = 20,
    ) -> None:
        """校验参数并启动编码器，失败时不创建半可用对象。"""

        command = build_ffmpeg_rtp_command(
            host,
            port,
            width=width,
            height=height,
            fps=fps,
        )
        if shutil.which("ffmpeg") is None:
            raise AnnotatedVideoError("未找到 ffmpeg，无法发布带框 RTP 画面")
        self.width = width
        self.height = height
        try:
            self._process = subprocess.Popen(command, stdin=subprocess.PIPE)
        except OSError as exc:
            raise AnnotatedVideoError(f"FFmpeg 启动失败: {exc}") from exc
        if self._process.stdin is None:
            self._process.terminate()
            raise AnnotatedVideoError("FFmpeg 标准输入管道创建失败")

    def write(self, frame: Any) -> None:
        """缩放一帧 BGR 图像并写入编码器。"""

        import cv2

        if frame is None or getattr(frame, "size", 0) == 0:
            raise AnnotatedVideoError("不能发布空画面")
        resized = cv2.resize(
            frame,
            (self.width, self.height),
            interpolation=cv2.INTER_AREA,
        )
        try:
            self._process.stdin.write(resized.tobytes())
        except (BrokenPipeError, OSError) as exc:
            raise AnnotatedVideoError("带框 RTP 编码管道已断开") from exc

    def close(self) -> None:
        """关闭输入并回收 FFmpeg；异常时确保不遗留子进程。"""

        if self._process.poll() is not None:
            return
        try:
            if self._process.stdin is not None and not self._process.stdin.closed:
                self._process.stdin.close()
            self._process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            self._process.terminate()
            try:
                self._process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait()
