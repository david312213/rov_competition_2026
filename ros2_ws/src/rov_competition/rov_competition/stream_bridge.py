"""QGroundControl RTP/H.264 视频到 HDMI/RTMP 的安全分流工具。"""

from __future__ import annotations

import argparse
import shutil
import signal
import subprocess
from urllib.parse import urlparse


class StreamConfigurationError(ValueError):
    """视频参数不合法或缺少 GStreamer。"""


def _port(value: str) -> int:
    """解析 1..65535 的端口参数。"""

    number = int(value)
    if not 1 <= number <= 65535:
        raise argparse.ArgumentTypeError("端口必须在 1..65535")
    return number


def build_pipeline_arguments(
    *,
    source_port: int,
    payload_type: int,
    qgc_host: str | None,
    qgc_port: int | None,
    inference_host: str | None,
    inference_port: int | None,
    rtmp_url: str | None,
    display: bool = True,
) -> list[str]:
    """创建不经过 shell 解释的 GStreamer 参数列表。

    旧脚本使用 ``shell=True`` 拼接 RTMP 地址，特殊字符可能被 shell 执行。本函数
    每个参数独立传递给 ``gst-launch-1.0``，既保留分流能力又消除注入风险。
    """

    if not 0 <= payload_type <= 127:
        raise StreamConfigurationError("RTP payload type 必须在 0..127")
    if not 1 <= source_port <= 65535:
        raise StreamConfigurationError("RTP 源端口必须在 1..65535")
    for label, host, port in (
        ("QGC", qgc_host, qgc_port),
        ("AI", inference_host, inference_port),
    ):
        if (host is None) != (port is None):
            raise StreamConfigurationError(f"{label} 主机与端口必须同时提供或同时禁用")
        if port is not None and not 1 <= port <= 65535:
            raise StreamConfigurationError(f"{label} 端口必须在 1..65535")
    arguments = [
        "gst-launch-1.0",
        "-e",
        "udpsrc",
        f"port={source_port}",
        f"caps=application/x-rtp,media=video,encoding-name=H264,payload={payload_type},clock-rate=90000",
        "!",
        "tee",
        "name=stream",
    ]
    if qgc_host is not None and qgc_port is not None:
        arguments.extend(
            [
                "stream.",
                "!",
                "queue",
                "!",
                "udpsink",
                f"host={qgc_host}",
                f"port={qgc_port}",
                "sync=false",
                "async=false",
            ]
        )
    if inference_host is not None and inference_port is not None:
        arguments.extend(
            [
                "stream.",
                "!",
                "queue",
                "!",
                "rtph264depay",
                "!",
                "h264parse",
                "config-interval=1",
                "!",
                "mpegtsmux",
                "alignment=7",
                "!",
                "udpsink",
                f"host={inference_host}",
                f"port={inference_port}",
                "sync=false",
                "async=false",
            ]
        )
    if display:
        arguments.extend(
            [
                "stream.",
                "!",
                "queue",
                "!",
                "rtph264depay",
                "!",
                "h264parse",
                "!",
                "avdec_h264",
                "!",
                "videoconvert",
                "!",
                "autovideosink",
                "sync=false",
            ]
        )
    if rtmp_url:
        parsed = urlparse(rtmp_url)
        if parsed.scheme not in {"rtmp", "rtmps"} or not parsed.netloc:
            raise StreamConfigurationError("RTMP 地址必须以 rtmp:// 或 rtmps:// 开头")
        arguments.extend(
            [
                "stream.",
                "!",
                "queue",
                "!",
                "rtph264depay",
                "!",
                "h264parse",
                "config-interval=1",
                "!",
                "video/x-h264,stream-format=avc,alignment=au",
                "!",
                "flvmux",
                "streamable=true",
                "!",
                "rtmpsink",
                f"location={rtmp_url}",
                "sync=false",
                "async=false",
            ]
        )
    if not any((qgc_host, inference_host, display, rtmp_url)):
        raise StreamConfigurationError("至少启用 QGC、AI、显示或 RTMP 中的一路输出")
    return arguments


def main() -> None:
    """启动视频桥并将 Ctrl+C 正确转发给 GStreamer。"""

    parser = argparse.ArgumentParser(description="ROV RTP/H.264 安全视频分流")
    parser.add_argument("--source-port", type=_port, default=5600)
    parser.add_argument("--payload-type", type=int, default=96)
    parser.add_argument("--qgc-host", default="127.0.0.1")
    parser.add_argument("--qgc-port", type=_port, default=5701)
    parser.add_argument("--no-qgc", action="store_true", help="禁用 QGC 转发分支")
    parser.add_argument("--inference-host", default="127.0.0.1")
    parser.add_argument("--inference-port", type=_port, default=5702)
    parser.add_argument("--no-inference", action="store_true", help="禁用 AI 转发分支")
    parser.add_argument("--rtmp", help="可选 RTMP/RTMPS 推流地址")
    parser.add_argument(
        "--no-display",
        action="store_true",
        help="不在本机显示第一视角（默认显示，供 HDMI 镜像）",
    )
    parser.add_argument("--print-only", action="store_true")
    args = parser.parse_args()

    command = build_pipeline_arguments(
        source_port=args.source_port,
        payload_type=args.payload_type,
        qgc_host=None if args.no_qgc else args.qgc_host,
        qgc_port=None if args.no_qgc else args.qgc_port,
        inference_host=None if args.no_inference else args.inference_host,
        inference_port=None if args.no_inference else args.inference_port,
        rtmp_url=args.rtmp,
        display=not args.no_display,
    )
    print(" ".join(command))
    if args.print_only:
        return
    if shutil.which("gst-launch-1.0") is None:
        raise StreamConfigurationError("未找到 gst-launch-1.0，请安装 README 所列插件")

    process = subprocess.Popen(command, start_new_session=True)

    def stop_process(_signal_number: int, _frame: object) -> None:
        """请求 GStreamer 完整写出尾部并退出。"""

        if process.poll() is None:
            process.send_signal(signal.SIGINT)

    signal.signal(signal.SIGINT, stop_process)
    signal.signal(signal.SIGTERM, stop_process)
    return_code = process.wait()
    if return_code != 0:
        raise RuntimeError(f"GStreamer 视频桥异常退出，返回码 {return_code}")


if __name__ == "__main__":
    main()
