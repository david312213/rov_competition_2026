"""把一份艇载 RTP/H.264 原始数据包复制给多个本机使用者。

默认拓扑中 QGC 直接接收 BlueOS 发到 5600 的视频，本工具只监听
BlueOS 的第二路 5700，再原样复制到 YOLO 5702 和录像器 5704。
底层命令行仍允许自定义端口，便于故障兼容；一键脚本不再经由本工具转发 QGC。
"""

from __future__ import annotations

import argparse
import ipaddress
import shutil
import signal
import subprocess
from urllib.parse import urlparse


class StreamConfigurationError(ValueError):
    """视频分流参数不合法或系统缺少 GStreamer。"""


def _port(value: str) -> int:
    """解析 1..65535 的端口参数。"""

    number = int(value)
    if not 1 <= number <= 65535:
        raise argparse.ArgumentTypeError("端口必须在 1..65535")
    return number


def _canonical_host(host: str) -> str:
    """规范化常见本机地址，便于识别重复目标和回环配置。"""

    value = host.strip().lower().strip("[]")
    if not value:
        raise StreamConfigurationError("视频输出主机不能为空")
    if value == "localhost":
        return "127.0.0.1"
    try:
        return ipaddress.ip_address(value).compressed
    except ValueError:
        return value


def _is_local_host(host: str) -> bool:
    """判断地址是否明确指向本机或通配地址。"""

    value = _canonical_host(host)
    if value in {"0.0.0.0", "::"}:
        return True
    try:
        return ipaddress.ip_address(value).is_loopback
    except ValueError:
        return False


def _multiudpsink_target(host: str, port: int) -> str:
    """生成 GStreamer ``multiudpsink clients`` 中的一项。"""

    clean_host = host.strip().strip("[]")
    if ":" in clean_host:
        clean_host = f"[{clean_host}]"
    return f"{clean_host}:{port}"


def _validate_endpoint(
    label: str,
    host: str | None,
    port: int | None,
    *,
    source_port: int,
) -> tuple[str, int] | None:
    """校验一对可选主机/端口，并阻止发回本机源端口。"""

    if (host is None) != (port is None):
        raise StreamConfigurationError(f"{label} 主机与端口必须同时提供或同时禁用")
    if host is None or port is None:
        return None
    if not 1 <= port <= 65535:
        raise StreamConfigurationError(f"{label} 端口必须在 1..65535")
    _canonical_host(host)
    if port == source_port and _is_local_host(host):
        raise StreamConfigurationError(
            f"{label} 不能发回本机源端口 {source_port}，否则会形成视频回环"
        )
    return host, port


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
    record_host: str | None = None,
    record_port: int | None = None,
) -> list[str]:
    """创建不经过 shell 解释的 GStreamer 参数列表。

    QGC、AI 和可选录像器共用一个 ``multiudpsink``。进入该分支前没有解包、解析、
    重封装或编码操作，所以各端收到的是源端口上原始 RTP 包的本机副本。
    只有可选的本机显示和 RTMP 分支会另外解码或封装，它们不改变分流包。
    """

    if not 0 <= payload_type <= 127:
        raise StreamConfigurationError("RTP payload type 必须在 0..127")
    if not 1 <= source_port <= 65535:
        raise StreamConfigurationError("RTP 源端口必须在 1..65535")

    qgc_endpoint = _validate_endpoint(
        "QGC", qgc_host, qgc_port, source_port=source_port
    )
    inference_endpoint = _validate_endpoint(
        "AI", inference_host, inference_port, source_port=source_port
    )
    record_endpoint = _validate_endpoint(
        "录像器", record_host, record_port, source_port=source_port
    )

    named_endpoints = (
        ("QGC", qgc_endpoint),
        ("AI", inference_endpoint),
        ("录像器", record_endpoint),
    )
    used: dict[tuple[str, int], str] = {}
    for label, endpoint in named_endpoints:
        if endpoint is None:
            continue
        key = (_canonical_host(endpoint[0]), endpoint[1])
        previous = used.get(key)
        if previous is not None:
            raise StreamConfigurationError(
                f"{previous} 与 {label} 不能使用同一个目标主机和端口"
            )
        used[key] = label

    if rtmp_url:
        parsed = urlparse(rtmp_url)
        if parsed.scheme not in {"rtmp", "rtmps"} or not parsed.netloc:
            raise StreamConfigurationError("RTMP 地址必须以 rtmp:// 或 rtmps:// 开头")

    endpoints = [
        endpoint
        for endpoint in (qgc_endpoint, inference_endpoint, record_endpoint)
        if endpoint is not None
    ]
    if not endpoints and not display and not rtmp_url:
        raise StreamConfigurationError(
            "至少启用 QGC、AI、录像器、显示或 RTMP 中的一路输出"
        )

    arguments = [
        "gst-launch-1.0",
        "-e",
        "udpsrc",
        f"port={source_port}",
        "caps=application/x-rtp,media=video,encoding-name=H264,"
        f"payload={payload_type},clock-rate=90000",
        "!",
        "tee",
        "name=packets",
    ]

    if endpoints:
        clients = ",".join(
            _multiudpsink_target(host, port) for host, port in endpoints
        )
        arguments.extend(
            [
                "packets.",
                "!",
                "queue",
                "!",
                "multiudpsink",
                f"clients={clients}",
                "sync=false",
                "async=false",
            ]
        )

    if display:
        arguments.extend(
            [
                "packets.",
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
        arguments.extend(
            [
                "packets.",
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
    return arguments


def main() -> None:
    """启动视频分流，并把 Ctrl+C 正确转发给 GStreamer。"""

    parser = argparse.ArgumentParser(description="ROV RTP/H.264 原始包分流")
    parser.add_argument("--source-port", type=_port, default=5700)
    parser.add_argument("--payload-type", type=int, default=96)
    # 新拓扑默认不转发 QGC；故障兼容时必须显式同时给出 host/port。
    parser.add_argument("--qgc-host")
    parser.add_argument("--qgc-port", type=_port)
    parser.add_argument("--no-qgc", action="store_true", help="禁用 QGC 转发分支")
    parser.add_argument("--inference-host", default="127.0.0.1")
    parser.add_argument("--inference-port", type=_port, default=5702)
    parser.add_argument("--no-inference", action="store_true", help="禁用 AI 转发分支")
    parser.add_argument("--record-host", help="可选原始 RTP 录像器主机")
    parser.add_argument("--record-port", type=_port, help="可选原始 RTP 录像器端口")
    parser.add_argument("--rtmp", help="可选 RTMP/RTMPS 推流地址")
    parser.add_argument(
        "--no-display",
        action="store_true",
        help="不在本机额外显示第一视角",
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
        record_host=args.record_host,
        record_port=args.record_port,
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
        """请求 GStreamer 完整退出。"""

        if process.poll() is None:
            process.send_signal(signal.SIGINT)

    signal.signal(signal.SIGINT, stop_process)
    signal.signal(signal.SIGTERM, stop_process)
    return_code = process.wait()
    if return_code != 0:
        raise RuntimeError(f"GStreamer 视频分流异常退出，返回码 {return_code}")


if __name__ == "__main__":
    main()
