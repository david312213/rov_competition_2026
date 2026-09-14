"""默认 QGC 直连、软件 5700 原始 RTP 分流的回归测试。"""

import pytest
from rov_competition.stream_bridge import (
    StreamConfigurationError,
    build_pipeline_arguments,
)


def _software_fanout_command() -> list[str]:
    """返回搜索测试中 5700 -> 5702/5704 的原始包复制。"""

    return build_pipeline_arguments(
        source_port=5700,
        payload_type=96,
        qgc_host=None,
        qgc_port=None,
        inference_host="127.0.0.1",
        inference_port=5702,
        record_host="127.0.0.1",
        record_port=5704,
        rtmp_url=None,
        display=False,
    )


def test_default_software_fanout_is_raw_rtp_only() -> None:
    """5700 必须原样复制到 YOLO/录像，不二次编码。"""

    command = _software_fanout_command()
    assert command[0] == "gst-launch-1.0"
    assert "port=5700" in command
    assert command.count("multiudpsink") == 1
    assert "clients=127.0.0.1:5702,127.0.0.1:5704" in command
    for forbidden in (
        "rtph264depay",
        "h264parse",
        "rtph264pay",
        "mpegtsmux",
        "tsdemux",
        "x264enc",
        "openh264enc",
    ):
        assert forbidden not in command


def test_record_only_mode_does_not_forward_qgc() -> None:
    """QGC 直接占用 5600，手柄录像只处理 5700 -> 5704。"""

    command = build_pipeline_arguments(
        source_port=5700,
        payload_type=96,
        qgc_host=None,
        qgc_port=None,
        inference_host=None,
        inference_port=None,
        record_host="127.0.0.1",
        record_port=5704,
        rtmp_url=None,
        display=False,
    )
    assert "clients=127.0.0.1:5704" in command
    assert "5600" not in " ".join(command)


def test_duplicate_or_loopback_targets_are_rejected() -> None:
    with pytest.raises(StreamConfigurationError, match="同一个目标"):
        build_pipeline_arguments(
            source_port=5700,
            payload_type=96,
            qgc_host=None,
            qgc_port=None,
            inference_host="localhost",
            inference_port=5702,
            record_host="127.0.0.1",
            record_port=5702,
            rtmp_url=None,
            display=False,
        )
    with pytest.raises(StreamConfigurationError, match="视频回环"):
        build_pipeline_arguments(
            source_port=5700,
            payload_type=96,
            qgc_host=None,
            qgc_port=None,
            inference_host="127.0.0.1",
            inference_port=5700,
            rtmp_url=None,
            display=False,
        )


def test_low_level_qgc_compatibility_endpoint_still_exists() -> None:
    """底层工具仍能显式分流给自定义端口，但它不是默认方案。"""

    command = build_pipeline_arguments(
        source_port=6000,
        payload_type=96,
        qgc_host="127.0.0.1",
        qgc_port=6001,
        inference_host="127.0.0.1",
        inference_port=6002,
        rtmp_url=None,
        display=False,
    )
    assert "clients=127.0.0.1:6001,127.0.0.1:6002" in command


def test_pipeline_is_an_argument_list_not_shell_text() -> None:
    url = "rtmp://example.invalid/live/key?token=a;b"
    command = build_pipeline_arguments(
        source_port=5700,
        payload_type=96,
        qgc_host=None,
        qgc_port=None,
        inference_host="127.0.0.1",
        inference_port=5702,
        rtmp_url=url,
        display=True,
    )
    assert f"location={url}" in command
    assert "shell=True" not in command
    assert "multiudpsink" in command
    assert "rtph264pay" not in command
    assert "mpegtsmux" not in command


def test_invalid_rtmp_and_partial_endpoint_are_rejected() -> None:
    with pytest.raises(StreamConfigurationError, match="RTMP"):
        build_pipeline_arguments(
            source_port=5700,
            payload_type=96,
            qgc_host=None,
            qgc_port=None,
            inference_host="127.0.0.1",
            inference_port=5702,
            rtmp_url="file:///tmp/output",
            display=False,
        )
    with pytest.raises(StreamConfigurationError, match="同时"):
        build_pipeline_arguments(
            source_port=5700,
            payload_type=96,
            qgc_host="127.0.0.1",
            qgc_port=None,
            inference_host=None,
            inference_port=None,
            rtmp_url=None,
            display=False,
        )
